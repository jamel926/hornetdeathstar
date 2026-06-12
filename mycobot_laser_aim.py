"""
mycobot_laser_aim.py
====================
myCobot280 Jetson Nano — Pointage laser (grands mouvements / initialisation)

Responsabilités de ce fichier
------------------------------
  - Paramètres DH + cinématique directe (FK) compilée Numba
  - Configuration robot (RobotConfig)
  - IK complète multi-seeds + parallélisation (solve_ik)
  - Classe LaserAimer : connexion, aim_at(), go_home()
  - Utilitaires géométriques partagés

Ce fichier est importé par laser_tracker.py qui y ajoute
le tracking temps réel (Jacobienne, Kalman, boucle 25 Hz).

Dépendances
-----------
    pip install pymycobot numpy scipy numba

Usage autonome (grands mouvements ponctuels)
--------------------------------------------
    from mycobot_laser_aim import LaserAimer, RobotConfig
    import numpy as np

    cfg   = RobotConfig(T_source2robot=np.eye(4))
    aimer = LaserAimer(cfg)
    aimer.connect()
    aimer.aim_at(150.0, 80.0, 400.0)   # coords dans le repère source
    aimer.disconnect()

Simulation sans robot
---------------------
    python mycobot_laser_aim.py --sim
"""

from __future__ import annotations

import math
import time
import logging
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from numpy.linalg import norm
from scipy.optimize import minimize

try:
    from numba import njit
    NUMBA_OK = True
except ImportError:
    NUMBA_OK = False
    def njit(*args, **kwargs):
        def wrapper(fn): return fn
        return wrapper

from pymycobot.mycobot280 import MyCobot280

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s — %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("laser_aim")


# ===========================================================================
# Paramètres DH du myCobot280 (Denavit-Hartenberg modifiés)
# ===========================================================================
#  Joint | alpha (rad) |   a (mm)  |   d (mm)  | theta_offset (rad)
#  ------|-------------|-----------|-----------|-------------------
#    1   |      0      |     0     |  131.56   |        0
#    2   |    π/2      |     0     |    0      |      −π/2
#    3   |      0      |  −110.4   |    0      |        0
#    4   |      0      |   −96.0   |   63.4    |      −π/2
#    5   |    π/2      |     0     |   75.05   |        0
#    6   |   −π/2      |     0     |   45.6    |        0

DH_FLAT = np.array([
    [ 0.0,        0.0,   131.56,   0.0       ],
    [ math.pi/2,  0.0,     0.0,  -math.pi/2  ],
    [ 0.0,      -110.4,    0.0,   0.0        ],
    [ 0.0,       -96.0,   63.4,  -math.pi/2  ],
    [ math.pi/2,   0.0,  75.05,   0.0        ],
    [-math.pi/2,   0.0,  45.6,    0.0        ],
], dtype=np.float64)

JOINT_LIMITS_DEG = [
    (-165, 165), (-165, 165), (-165, 165),
    (-165, 165), (-165, 165), (-175, 175),
]
JOINT_LIMITS_RAD = [
    (math.radians(lo), math.radians(hi))
    for lo, hi in JOINT_LIMITS_DEG
]


# ===========================================================================
# Cinématique directe (FK)
# — exportée et réutilisée par laser_tracker.py
# ===========================================================================

@njit(cache=True)
def _fk_numba(q: np.ndarray, dh: np.ndarray) -> tuple:
    """FK compilée Numba — retourne (px, py, pz, zx, zy, zz)."""
    T = np.eye(4)
    for i in range(6):
        alpha = dh[i, 0]; a = dh[i, 1]; d = dh[i, 2]; toff = dh[i, 3]
        theta = q[i] + toff
        ct = math.cos(theta); st = math.sin(theta)
        ca = math.cos(alpha); sa = math.sin(alpha)
        M = np.array([
            [ct,     -st,    0.0,  a    ],
            [st*ca,   ct*ca, -sa, -sa*d ],
            [st*sa,   ct*sa,  ca,  ca*d ],
            [0.0,     0.0,   0.0,  1.0  ],
        ])
        T = T @ M
    return T[0,3], T[1,3], T[2,3], T[0,2], T[1,2], T[2,2]


def _fk_numpy(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """FK numpy — fallback si Numba absent."""
    T = np.eye(4)
    for i in range(6):
        alpha, a, d, toff = DH_FLAT[i]
        theta = q[i] + toff
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        M = np.array([
            [ct,     -st,    0.0,  a    ],
            [st*ca,   ct*ca, -sa, -sa*d ],
            [st*sa,   ct*sa,  ca,  ca*d ],
            [0.0,     0.0,   0.0,  1.0  ],
        ])
        T = T @ M
    return T[:3, 3], T[:3, 2]


def fk(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Cinématique directe — retourne (position_mm, axe_z6).
    Dispatch automatique Numba / numpy.
    Exportée et utilisée par laser_tracker.py.
    """
    if NUMBA_OK:
        px, py, pz, zx, zy, zz = _fk_numba(q, DH_FLAT)
        return np.array([px, py, pz]), np.array([zx, zy, zz])
    return _fk_numpy(q)


# Compilation Numba au chargement du module
if NUMBA_OK:
    try:
        _fk_numba(np.zeros(6), DH_FLAT)
        log.info("Numba JIT compilé et prêt.")
    except Exception as e:
        log.warning("Numba indisponible (%s) → fallback numpy.", e)
        NUMBA_OK = False


# ===========================================================================
# Utilitaires géométriques
# — exportés et réutilisés par laser_tracker.py
# ===========================================================================

def transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Applique une transformation homogène 4×4 à un point 3D."""
    return (T @ np.array([p[0], p[1], p[2], 1.0]))[:3]


def unit_direction(target: np.ndarray, origin: np.ndarray) -> Optional[np.ndarray]:
    """
    Retourne le vecteur unitaire de origin vers target.
    Retourne None si la distance est trop faible (< 1 mm).
    """
    d = target - origin
    n = norm(d)
    if n < 1.0:
        return None
    return d / n


def pointing_error_deg(q: np.ndarray, target_dir: np.ndarray) -> float:
    """Erreur angulaire (°) entre l'axe Z6 courant et target_dir."""
    _, z_axis = fk(q)
    cos_e = float(np.clip(np.dot(z_axis, target_dir), -1.0, 1.0))
    return math.degrees(math.acos(cos_e))


# ===========================================================================
# Configuration robot
# — exportée et réutilisée par laser_tracker.py
# ===========================================================================

@dataclass
class RobotConfig:
    """
    Tous les paramètres de l'installation.
    Instanciée une seule fois et passée à LaserAimer + LaserTracker.
    """
    # Connexion série
    port:     str = "/dev/ttyTHS1"   # ou /dev/ttyUSB0
    baudrate: int = 1000000

    # Position home de l'effecteur (mm, repère robot)
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 250.0

    # Vitesse et délai pour aim_at (grands mouvements)
    speed:        int   = 20
    move_timeout: float = 3.5

    # Paramètres IK complète
    n_seeds:         int   = 12
    n_perturbations: int   = 8
    early_stop_deg:  float = 0.15
    n_workers:       int   = 4
    w_direction:     float = 10.0
    w_position:      float = 2.0
    w_limits:        float = 1.0
    w_smoothness:    float = 0.5

    # Matrice de transformation repère source → base robot (4×4 homogène)
    T_source2robot: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=float)
    )

    @property
    def home_pos(self) -> np.ndarray:
        """Position home sous forme de vecteur numpy (3,)."""
        return np.array([self.home_x, self.home_y, self.home_z])


# ===========================================================================
# IK complète multi-seeds
# — solve_ik exportée et appelée par laser_tracker.py
# ===========================================================================

def _ik_cost(q, target_dir, home_pos, w_dir, w_pos, w_lim, w_smo, q_cur_list):
    """Fonction de coût IK (module-level pour multiprocessing pickle)."""
    pos, z_axis = fk(q)
    cos_e   = float(np.clip(np.dot(z_axis, target_dir), -1.0, 1.0))
    ang_err = math.acos(cos_e)
    pos_err = norm(pos - home_pos) / 1000.0
    lim_pen = sum(
        ((q[i] - (lo + hi) / 2) / ((hi - lo) / 2)) ** 2
        for i, (lo, hi) in enumerate(JOINT_LIMITS_RAD)
    )
    q_cur = np.array(q_cur_list) if q_cur_list is not None else None
    smo   = float(norm(q - q_cur)) if q_cur is not None else 0.0
    return w_dir * ang_err + w_pos * pos_err + w_lim * lim_pen + w_smo * smo


def _optimize_seed(args) -> tuple[np.ndarray, float]:
    """Worker multiprocessing — optimise une seed unique."""
    seed, target_dir, home_pos, w_dir, w_pos, w_lim, w_smo, q_cur_list = args
    def cost(q):
        return _ik_cost(q, target_dir, home_pos, w_dir, w_pos, w_lim, w_smo, q_cur_list)
    try:
        res = minimize(cost, seed, method="L-BFGS-B", bounds=JOINT_LIMITS_RAD,
                       options={"maxiter": 800, "ftol": 1e-11, "gtol": 1e-8})
        _, z = fk(res.x)
        err  = math.degrees(math.acos(float(np.clip(np.dot(z, target_dir), -1.0, 1.0))))
        return res.x, err
    except Exception:
        return seed, 180.0


def _generate_seeds(cfg: RobotConfig, q_cur: Optional[np.ndarray]) -> list[np.ndarray]:
    """Génère les seeds pour l'IK : warmstart + cardinales + aléatoires."""
    rng   = np.random.default_rng(42)
    seeds = []
    if q_cur is not None:
        seeds.append(q_cur.copy())
        for sigma in [0.05, 0.12, 0.25]:
            for _ in range(cfg.n_perturbations // 3 + 1):
                seeds.append(q_cur + rng.normal(0.0, sigma, 6))
    seeds.append(np.zeros(6))
    for deg in [
        [0, 0,   0,  0,  90,  0], [0,  0,   0, 0, -90,  0],
        [0, 45, -45, 0,  90,  0], [0, -45,  45, 0, -90,  0],
        [90, 0,   0, 0,   0,  0], [-90, 0,  0, 0,   0,  0],
        [0, 90,   0, 0,   0,  0], [0, -90,  0, 0,   0,  0],
    ]:
        seeds.append(np.radians(deg))
    for _ in range(cfg.n_seeds):
        seeds.append(np.array([rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS_RAD]))
    return seeds


def solve_ik(
    target_dir: np.ndarray,
    home_pos:   np.ndarray,
    cfg:        RobotConfig,
    q_cur:      Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """
    IK complète multi-seeds avec parallélisation et arrêt anticipé.

    Paramètres
    ----------
    target_dir : vecteur unitaire (3,) — direction souhaitée de l'axe Z6
    home_pos   : position home de l'effecteur (mm, repère robot)
    cfg        : RobotConfig
    q_cur      : angles articulaires courants (rad) — seed warmstart

    Retourne
    --------
    Array numpy (6,) des angles en radians, ou None si échec.
    Exportée et appelée par laser_tracker.py pour les grands mouvements.
    """
    seeds  = _generate_seeds(cfg, q_cur)
    q_list = q_cur.tolist() if q_cur is not None else None
    lo_arr = [b[0] for b in JOINT_LIMITS_RAD]
    hi_arr = [b[1] for b in JOINT_LIMITS_RAD]

    args_list = [
        (np.clip(s, lo_arr, hi_arr),
         target_dir, home_pos,
         cfg.w_direction, cfg.w_position, cfg.w_limits, cfg.w_smoothness,
         q_list)
        for s in seeds
    ]

    best_q, best_err = None, 180.0
    t0 = time.perf_counter()

    try:
        with ProcessPoolExecutor(max_workers=cfg.n_workers) as ex:
            futures = {ex.submit(_optimize_seed, a): a for a in args_list}
            for f in as_completed(futures):
                q_sol, err = f.result()
                if err < best_err:
                    best_err, best_q = err, q_sol
                if best_err < cfg.early_stop_deg:
                    ex.shutdown(wait=False, cancel_futures=True)
                    break
    except Exception as e:
        log.warning("Parallélisation indispo (%s) → séquentiel.", e)
        for a in args_list:
            q_sol, err = _optimize_seed(a)
            if err < best_err:
                best_err, best_q = err, q_sol
            if best_err < cfg.early_stop_deg:
                break

    elapsed = (time.perf_counter() - t0) * 1000
    if best_q is not None:
        log.info("IK complète : err=%.3f°  temps=%.1f ms", best_err, elapsed)
    else:
        log.error("IK complète : aucune solution trouvée.")
    return best_q


# ===========================================================================
# Classe LaserAimer — grands mouvements ponctuels
# ===========================================================================

class LaserAimer:
    """
    Contrôle le laser pour des mouvements ponctuels (aim_at).
    Instanciée et partagée avec LaserTracker via robot_cfg.

    Interface publique
    ------------------
    connect()                          — ouvre le port série
    disconnect()                       — ferme le port série
    go_home()                          — retourne aux angles 0°
    aim_at(x, y, z, blocking=True)    — pointe vers (x,y,z) repère source
    aim_at_sequence(targets)           — enchaîne plusieurs cibles
    get_pointing_error_deg(x, y, z)   — erreur angulaire courante
    q_current                          — property : angles courants (rad)
    robot                              — property : instance MyCobot280
    """

    def __init__(self, cfg: RobotConfig):
        self.cfg  = cfg
        self._robot:  Optional[MyCobot280] = None
        self._q_cur:  Optional[np.ndarray] = None

    # --- Properties exposées à laser_tracker.py ---------------------------

    @property
    def q_current(self) -> Optional[np.ndarray]:
        """Angles articulaires courants en radians."""
        return self._q_cur

    @q_current.setter
    def q_current(self, q: Optional[np.ndarray]):
        self._q_cur = q

    @property
    def robot(self) -> Optional[MyCobot280]:
        """Instance MyCobot280 — partagée avec LaserTracker."""
        return self._robot

    # --- Connexion --------------------------------------------------------

    def connect(self) -> None:
        """Ouvre la connexion série et active le fresh_mode."""
        log.info("Connexion sur %s @ %d baud…", self.cfg.port, self.cfg.baudrate)
        self._robot = MyCobot280(self.cfg.port, self.cfg.baudrate)
        time.sleep(1.0)
        if self._robot.get_fresh_mode() != 1:
            self._robot.set_fresh_mode(1)
            time.sleep(0.3)
            log.info("fresh_mode activé.")
        self._sync_q()
        log.info("Connecté. Position : %s°",
                 np.round(np.degrees(self._q_cur), 1).tolist()
                 if self._q_cur is not None else "inconnue")

    def disconnect(self) -> None:
        """Ferme la connexion série."""
        self._robot = None
        log.info("Déconnecté.")

    # --- API publique -----------------------------------------------------

    def go_home(self) -> None:
        """Retourne à la position neutre (angles = 0°)."""
        if self._robot is None:
            raise RuntimeError("Robot non connecté.")
        log.info("Retour home…")
        self._robot.send_angles([0, 0, 0, 0, 0, 0], self.cfg.speed)
        self._wait_motion()
        self._q_cur = np.zeros(6)

    def aim_at(
        self,
        x_src: float,
        y_src: float,
        z_src: float,
        blocking: bool = True,
    ) -> bool:
        """
        Pointe le laser vers (x_src, y_src, z_src) dans le repère source.

        Retourne True si le mouvement a été envoyé avec succès.
        """
        if self._robot is None:
            raise RuntimeError("Robot non connecté.")

        target = transform_point(self.cfg.T_source2robot, np.array([x_src, y_src, z_src]))
        log.info("Cible robot : (%.1f, %.1f, %.1f) mm", *target)

        direction = unit_direction(target, self.cfg.home_pos)
        if direction is None:
            log.error("Cible trop proche de l'effecteur.")
            return False

        q_sol = solve_ik(direction, self.cfg.home_pos, self.cfg, self._q_cur)
        if q_sol is None:
            return False

        angles_deg = [round(math.degrees(a), 2) for a in q_sol]
        log.info("→ send_angles %s", angles_deg)
        self._robot.send_angles(angles_deg, self.cfg.speed)

        if blocking:
            self._wait_motion()

        self._q_cur = q_sol
        self._sync_q()
        return True

    def aim_at_sequence(
        self,
        targets: list[tuple[float, float, float]],
        delay_between: float = 0.3,
    ) -> list[bool]:
        """Enchaîne plusieurs pointages laser."""
        results = []
        for i, (x, y, z) in enumerate(targets):
            log.info("--- Cible %d/%d (%.1f, %.1f, %.1f) ---",
                     i + 1, len(targets), x, y, z)
            results.append(self.aim_at(x, y, z, blocking=True))
            if delay_between > 0:
                time.sleep(delay_between)
        return results

    def get_pointing_error_deg(
        self, x_src: float, y_src: float, z_src: float
    ) -> float:
        """Erreur angulaire (°) entre direction courante et cible."""
        if self._q_cur is None:
            return float("inf")
        target    = transform_point(self.cfg.T_source2robot, np.array([x_src, y_src, z_src]))
        direction = unit_direction(target, self.cfg.home_pos)
        if direction is None:
            return 0.0
        return pointing_error_deg(self._q_cur, direction)

    # --- Internes ---------------------------------------------------------

    def _sync_q(self) -> None:
        """Relit les angles réels depuis le robot (warmstart suivant)."""
        try:
            angles = self._robot.get_angles()
            if angles and len(angles) == 6:
                self._q_cur = np.radians(angles)
        except Exception as e:
            log.debug("Sync angles : %s", e)

    def _wait_motion(self) -> None:
        """Attend la fin du mouvement en interrogeant is_moving()."""
        deadline = time.time() + self.cfg.move_timeout
        time.sleep(0.3)
        while time.time() < deadline:
            try:
                if not self._robot.is_moving():
                    return
            except Exception:
                pass
            time.sleep(0.1)


# ===========================================================================
# Point d'entrée autonome (test / simulation)
# ===========================================================================

if __name__ == "__main__":
    import sys

    # ── ADAPTE ICI ──────────────────────────────────────────────────────────
    T_source2robot = np.array([
        [1, 0, 0, 200],
        [0, 1, 0,   0],
        [0, 0, 1, 100],
        [0, 0, 0,   1],
    ], dtype=float)

    cfg = RobotConfig(
        port="/dev/ttyTHS1",
        baudrate=1000000,
        home_x=0.0, home_y=0.0, home_z=250.0,
        speed=20,
        move_timeout=3.5,
        n_seeds=12,
        T_source2robot=T_source2robot,
    )
    # ────────────────────────────────────────────────────────────────────────

    if "--sim" in sys.argv:
        print("\n=== SIMULATION aim_at ===\n")
        targets_src = [
            (100.0,   0.0, 400.0),
            (  0.0, 150.0, 350.0),
            (-80.0,  80.0, 300.0),
            (200.0, -50.0, 500.0),
        ]
        print(f"{'Cible source':^30}  {'Angles (°)':^56}  {'Erreur':>7}  {'Temps':>7}")
        print("─" * 108)
        q_prev = None
        for x, y, z in targets_src:
            t0       = time.perf_counter()
            target_r = transform_point(T_source2robot, np.array([x, y, z]))
            dirn     = unit_direction(target_r, cfg.home_pos)
            if dirn is None:
                print(f"({x},{y},{z}) — trop proche"); continue
            q_sol    = solve_ik(dirn, cfg.home_pos, cfg, q_prev)
            elapsed  = (time.perf_counter() - t0) * 1000
            if q_sol is not None:
                angles = [round(math.degrees(a), 1) for a in q_sol]
                err    = pointing_error_deg(q_sol, dirn)
                print(f"({x:6.1f},{y:6.1f},{z:6.1f})  {str(angles):^56}  {err:5.3f}°  {elapsed:5.0f} ms")
                q_prev = q_sol
            else:
                print(f"({x:6.1f},{y:6.1f},{z:6.1f})  IMPOSSIBLE")

    else:
        aimer = LaserAimer(cfg)
        try:
            aimer.connect()
            aimer.go_home()
            time.sleep(1.0)
            targets = [(150.0, 50.0, 400.0), (200.0, -80.0, 350.0), (50.0, 120.0, 300.0)]
            results = aimer.aim_at_sequence(targets)
            for i, (t, ok) in enumerate(zip(targets, results)):
                print(f"  [{'✓' if ok else '✗'}] Cible {i+1} {t}")
        except KeyboardInterrupt:
            log.info("Interruption.")
        finally:
            aimer.go_home()
            aimer.disconnect()


#PYEOF
#echo "mycobot_laser_aim.py — $(wc -l < /mnt/user-data/outputs/mycobot_laser_aim.py) lignes"