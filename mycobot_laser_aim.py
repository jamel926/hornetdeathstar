"""
mycobot_laser_aim.py
====================
myCobot280 Jetson Nano — Pointage laser (grands mouvements / initialisation)

Responsabilités de ce fichier
------------------------------
  - Paramètres DH + cinématique directe (FK) compilée Numba
  - Configuration robot (RobotConfig)
  - IK complète multi-seeds + parallélisation (solveIk)
  - Classe LaserAimer : connexion, aimAt(), goHome()
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

    CFG   = RobotConfig(t_source2robot=np.eye(4))
    AIMER = LaserAimer(CFG)
    AIMER.connect()
    AIMER.aimAt(150.0, 80.0, 400.0)   # coords dans le repère source
    AIMER.disconnect()

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
#from PU_detection_couleur import detecterObjet

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

dh_flat = np.array([
    [ 0.0,        0.0,   131.56,   0.0       ],
    [ math.pi/2,  0.0,     0.0,  -math.pi/2  ],
    [ 0.0,      -110.4,    0.0,   0.0        ],
    [ 0.0,       -96.0,   63.4,  -math.pi/2  ],
    [ math.pi/2,   0.0,  75.05,   0.0        ],
    [-math.pi/2,   0.0,  45.6,    0.0        ],
], dtype=np.float64)

joint_limits_deg = [
    (-165, 165), (-165, 165), (-165, 165),
    (-165, 165), (-165, 165), (-175, 175),
]
joint_limits_rad = [
    (math.radians(LO), math.radians(HI))
    for LO, HI in joint_limits_deg
]


# ===========================================================================
# Cinématique directe (FK)
# — exportée et réutilisée par laser_tracker.py
# ===========================================================================

@njit(cache=True)
def _fkNumba(q: np.ndarray, dh: np.ndarray) -> tuple:
    """FK compilée Numba — retourne (px, py, pz, zx, zy, zz)."""
    T = np.eye(4)
    for I in range(6):
        ALPHA = dh[I, 0]; A = dh[I, 1]; D = dh[I, 2]; TOFF = dh[I, 3]
        THETA = q[I] + TOFF
        CT = math.cos(THETA); ST = math.sin(THETA)
        CA = math.cos(ALPHA); SA = math.sin(ALPHA)
        M = np.array([
            [CT,      -ST,    0.0,  A    ],
            [ST*CA,    CT*CA, -SA, -SA*D ],
            [ST*SA,    CT*SA,  CA,  CA*D ],
            [0.0,      0.0,   0.0,  1.0  ],
        ])
        T = T @ M
    return T[0,3], T[1,3], T[2,3], T[0,2], T[1,2], T[2,2]


def _fkNumpy(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """FK numpy — fallback si Numba absent."""
    T = np.eye(4)
    for I in range(6):
        ALPHA, A, D, TOFF = dh_flat[I]
        THETA = q[I] + TOFF
        CT, ST = math.cos(THETA), math.sin(THETA)
        CA, SA = math.cos(ALPHA), math.sin(ALPHA)
        M = np.array([
            [CT,      -ST,    0.0,  A    ],
            [ST*CA,    CT*CA, -SA, -SA*D ],
            [ST*SA,    CT*SA,  CA,  CA*D ],
            [0.0,      0.0,   0.0,  1.0  ],
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
        PX, PY, PZ, ZX, ZY, ZZ = _fkNumba(q, dh_flat)
        return np.array([PX, PY, PZ]), np.array([ZX, ZY, ZZ])
    return _fkNumpy(q)


# Compilation Numba au chargement du module
if NUMBA_OK:
    try:
        _fkNumba(np.zeros(6), dh_flat)
        log.info("Numba JIT compilé et prêt.")
    except Exception as E:
        log.warning("Numba indisponible (%s) → fallback numpy.", E)
        NUMBA_OK = False


# ===========================================================================
# Utilitaires géométriques
# — exportés et réutilisés par laser_tracker.py
# ===========================================================================

def transformPoint(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Applique une transformation homogène 4×4 à un point 3D."""
    return (T @ np.array([p[0], p[1], p[2], 1.0]))[:3]


def unitDirection(target: np.ndarray, origin: np.ndarray) -> Optional[np.ndarray]:
    """
    Retourne le vecteur unitaire de origin vers target.
    Retourne None si la distance est trop faible (< 1 mm).
    """
    D = target - origin
    N = norm(D)
    if N < 1.0:
        return None
    return D / N


def pointingErrorDeg(q: np.ndarray, target_dir: np.ndarray) -> float:
    """Erreur angulaire (°) entre l'axe Z6 courant et target_dir."""
    _, Z_AXIS = fk(q)
    COS_E = float(np.clip(np.dot(Z_AXIS, target_dir), -1.0, 1.0))
    return math.degrees(math.acos(COS_E))


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

    # Vitesse et délai pour aimAt (grands mouvements)
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
    t_source2robot: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=float)
    )

    @property
    def homePos(self) -> np.ndarray:
        """Position home sous forme de vecteur numpy (3,)."""
        return np.array([self.home_x, self.home_y, self.home_z])


# ===========================================================================
# IK complète multi-seeds
# — solveIk exportée et appelée par laser_tracker.py
# ===========================================================================

def _ikCost(q, target_dir, home_pos, w_dir, w_pos, w_lim, w_smo, q_cur_list):
    """Fonction de coût IK (module-level pour multiprocessing pickle)."""
    POS, Z_AXIS = fk(q)
    COS_E   = float(np.clip(np.dot(Z_AXIS, target_dir), -1.0, 1.0))
    ANG_ERR = math.acos(COS_E)
    POS_ERR = norm(POS - home_pos) / 1000.0
    LIM_PEN = sum(
        ((q[I] - (LO + HI) / 2) / ((HI - LO) / 2)) ** 2
        for I, (LO, HI) in enumerate(joint_limits_rad)
    )
    Q_CUR = np.array(q_cur_list) if q_cur_list is not None else None
    SMO   = float(norm(q - Q_CUR)) if Q_CUR is not None else 0.0
    return w_dir * ANG_ERR + w_pos * POS_ERR + w_lim * LIM_PEN + w_smo * SMO


def _optimizeSeed(args) -> tuple[np.ndarray, float]:
    """Worker multiprocessing — optimise une seed unique."""
    SEED, TARGET_DIR, HOME_POS, W_DIR, W_POS, W_LIM, W_SMO, Q_CUR_LIST = args
    def cost(q):
        return _ikCost(q, TARGET_DIR, HOME_POS, W_DIR, W_POS, W_LIM, W_SMO, Q_CUR_LIST)
    try:
        RES = minimize(cost, SEED, method="L-BFGS-B", bounds=joint_limits_rad,
                       options={"maxiter": 800, "ftol": 1e-11, "gtol": 1e-8})
        _, Z = fk(RES.x)
        ERR  = math.degrees(math.acos(float(np.clip(np.dot(Z, TARGET_DIR), -1.0, 1.0))))
        return RES.x, ERR
    except Exception:
        return SEED, 180.0


def _generateSeeds(cfg: RobotConfig, q_cur: Optional[np.ndarray]) -> list[np.ndarray]:
    """Génère les seeds pour l'IK : warmstart + cardinales + aléatoires."""
    RNG   = np.random.default_rng(42)
    SEEDS = []
    if q_cur is not None:
        SEEDS.append(q_cur.copy())
        for SIGMA in [0.05, 0.12, 0.25]:
            for _ in range(cfg.n_perturbations // 3 + 1):
                SEEDS.append(q_cur + RNG.normal(0.0, SIGMA, 6))
    SEEDS.append(np.zeros(6))
    for DEG in [
        [0, 0,   0,  0,  90,  0], [0,  0,   0, 0, -90,  0],
        [0, 45, -45, 0,  90,  0], [0, -45,  45, 0, -90,  0],
        [90, 0,   0, 0,   0,  0], [-90, 0,  0, 0,   0,  0],
        [0, 90,   0, 0,   0,  0], [0, -90,  0, 0,   0,  0],
    ]:
        SEEDS.append(np.radians(DEG))
    for _ in range(cfg.n_seeds):
        SEEDS.append(np.array([RNG.uniform(LO, HI) for LO, HI in joint_limits_rad]))
    return SEEDS


def solveIk(
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
    SEEDS  = _generateSeeds(cfg, q_cur)
    Q_LIST = q_cur.tolist() if q_cur is not None else None
    LO_ARR = [B[0] for B in joint_limits_rad]
    HI_ARR = [B[1] for B in joint_limits_rad]

    ARGS_LIST = [
        (np.clip(S, LO_ARR, HI_ARR),
         target_dir, home_pos,
         cfg.w_direction, cfg.w_position, cfg.w_limits, cfg.w_smoothness,
         Q_LIST)
        for S in SEEDS
    ]

    BEST_Q, BEST_ERR = None, 180.0
    T0 = time.perf_counter()

    try:
        with ProcessPoolExecutor(max_workers=cfg.n_workers) as EX:
            FUTURES = {EX.submit(_optimizeSeed, A): A for A in ARGS_LIST}
            for F in as_completed(FUTURES):
                Q_SOL, ERR = F.result()
                if ERR < BEST_ERR:
                    BEST_ERR, BEST_Q = ERR, Q_SOL
                if BEST_ERR < cfg.early_stop_deg:
                    EX.shutdown(wait=False, cancel_futures=True)
                    break
    except Exception as E:
        log.warning("Parallélisation indispo (%s) → séquentiel.", E)
        for A in ARGS_LIST:
            Q_SOL, ERR = _optimizeSeed(A)
            if ERR < BEST_ERR:
                BEST_ERR, BEST_Q = ERR, Q_SOL
            if BEST_ERR < cfg.early_stop_deg:
                break

    ELAPSED = (time.perf_counter() - T0) * 1000
    if BEST_Q is not None:
        log.info("IK complète : err=%.3f°  temps=%.1f ms", BEST_ERR, ELAPSED)
    else:
        log.error("IK complète : aucune solution trouvée.")
    return BEST_Q


# ===========================================================================
# Classe LaserAimer — grands mouvements ponctuels
# ===========================================================================

class LaserAimer:
    """
    Contrôle le laser pour des mouvements ponctuels (aimAt).
    Instanciée et partagée avec LaserTracker via robot_cfg.

    Interface publique
    ------------------
    connect()                          — ouvre le port série
    disconnect()                       — ferme le port série
    goHome()                           — retourne aux angles 0°
    aimAt(x, y, z, blocking=True)     — pointe vers (x,y,z) repère source
    aimAtSequence(targets)             — enchaîne plusieurs cibles
    getPointingErrorDeg(x, y, z)      — erreur angulaire courante
    qCurrent                           — property : angles courants (rad)
    robot                              — property : instance MyCobot280
    """

    def __init__(self, cfg: RobotConfig):
        self.cfg  = cfg
        self._robot:  Optional[MyCobot280] = None
        self._q_cur:  Optional[np.ndarray] = None

    # --- Properties exposées à laser_tracker.py ---------------------------

    @property
    def qCurrent(self) -> Optional[np.ndarray]:
        """Angles articulaires courants en radians."""
        return self._q_cur

    @qCurrent.setter
    def qCurrent(self, q: Optional[np.ndarray]):
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
        self._syncQ()
        log.info("Connecté. Position : %s°",
                 np.round(np.degrees(self._q_cur), 1).tolist()
                 if self._q_cur is not None else "inconnue")

    def disconnect(self) -> None:
        """Ferme la connexion série."""
        self._robot = None
        log.info("Déconnecté.")

    # --- API publique -----------------------------------------------------

    def goHome(self) -> None:
        """Retourne à la position neutre (angles = 0°)."""
        if self._robot is None:
            raise RuntimeError("Robot non connecté.")
        log.info("Retour home…")
        self._robot.send_angles([0, 0, 0, -90, 0, 0], self.cfg.speed)
        self._waitMotion()
        self._q_cur = np.zeros(6)

    def aimAt(
        self,
        x_src: float,
        y_src: float,
        z_src: float,
        blocking: bool = True,
    ) -> bool:
        """
        Pointe le laser vers (x_src, y_src, z_src) dans le repère source.

        Retourne True si le mouvement a été envoyé avec succès.

        Note sur homePos
        -----------------
        homePos est la position physique de l'effecteur (origine du laser).
        Elle est définie dans RobotConfig (home_x, home_y, home_z) et doit
        correspondre à la position réelle du bout du bras en mm dans le
        repère robot — pas nécessairement à ce que la FK retourne pour
        angles=[0,0,0,0,0,0] (qui est ~165 mm au-dessus de la base).
        Si le laser est fixe et que le bras ne bouge qu'en orientation,
        cette valeur doit être mesurée physiquement une fois pour toutes.
        """
        if self._robot is None:
            raise RuntimeError("Robot non connecté.")

        # 1. Transformer la cible dans le repère robot
        TARGET = transformPoint(self.cfg.t_source2robot, np.array([x_src, y_src, z_src]))

        # 2. Position réelle de l'effecteur calculée par FK (si on connaît q_cur)
        #    Priorité : FK sur q_cur > homePos config
        if self._q_cur is not None:
            EFFECTOR_POS, _ = fk(self._q_cur)
        else:
            EFFECTOR_POS = self.cfg.homePos

        log.info(
            "Cible source (%.1f,%.1f,%.1f) → robot (%.1f,%.1f,%.1f) mm",
            x_src, y_src, z_src, *TARGET,
        )
        log.info(
            "Effecteur (depuis %s) : (%.1f,%.1f,%.1f) mm",
            "FK" if self._q_cur is not None else "config",
            *EFFECTOR_POS,
        )

        # 3. Vecteur directeur effecteur → cible (normalisé)
        DIRECTION = unitDirection(TARGET, EFFECTOR_POS)
        if DIRECTION is None:
            log.error(
                "Cible trop proche de l'effecteur (< 1 mm). "
                "Vérifie la matrice t_source2robot et homePos."
            )
            return False

        log.info(
            "Direction laser (normalisée) : (%.4f, %.4f, %.4f)",
            *DIRECTION,
        )

        # 4. IK
        Q_SOL = solveIk(DIRECTION, EFFECTOR_POS, self.cfg, self._q_cur)
        if Q_SOL is None:
            return False

        ANGLES_DEG = [round(math.degrees(A), 2) for A in Q_SOL]
        log.info("→ send_angles %s", ANGLES_DEG)
        self._robot.send_angles(ANGLES_DEG, self.cfg.speed)

        if blocking:
            self._waitMotion()

        self._q_cur = Q_SOL
        self._syncQ()
        return True

    def aimAtSequence(
        self,
        targets: list[tuple[float, float, float]],
        delay_between: float = 0.3,
    ) -> list[bool]:
        """Enchaîne plusieurs pointages laser."""
        RESULTS = []
        for I, (X, Y, Z) in enumerate(targets):
            log.info("--- Cible %d/%d (%.1f, %.1f, %.1f) ---",
                     I + 1, len(targets), X, Y, Z)
            RESULTS.append(self.aimAt(X, Y, Z, blocking=True))
            if delay_between > 0:
                time.sleep(delay_between)
        return RESULTS

    def getPointingErrorDeg(
        self, x_src: float, y_src: float, z_src: float
    ) -> float:
        """Erreur angulaire (°) entre direction courante et cible."""
        if self._q_cur is None:
            return float("inf")
        TARGET    = transformPoint(self.cfg.t_source2robot, np.array([x_src, y_src, z_src]))
        DIRECTION = unitDirection(TARGET, self.cfg.homePos)
        if DIRECTION is None:
            return 0.0
        return pointingErrorDeg(self._q_cur, DIRECTION)

    # --- Internes ---------------------------------------------------------

    def _syncQ(self) -> None:
        """Relit les angles réels depuis le robot (warmstart suivant)."""
        try:
            ANGLES = self._robot.get_angles()
            if ANGLES and len(ANGLES) == 6:
                self._q_cur = np.radians(ANGLES)
        except Exception as E:
            log.debug("Sync angles : %s", E)

    def _waitMotion(self) -> None:
        """Attend la fin du mouvement en interrogeant is_moving()."""
        DEADLINE = time.time() + self.cfg.move_timeout
        time.sleep(0.3)
        while time.time() < DEADLINE:
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
    #
    # Correspondance des axes (caméra à l'origine du robot, Z_cam vers le bas) :
    #
    #   Repère CAMÉRA          Repère ROBOT
    #   X_cam (→ droite)   →   X_rob (→ avant)      : même sens
    #   Y_cam (↓ bas img)  →  -Y_rob (← droite rob) : inversé
    #   Z_cam (↓ profond)  →  -Z_rob (↓ bas robot)  : Z_cam pointe bas = -Z_rob
    #
    # Donc la rotation est :
    #   R = [[1,  0,  0],
    #        [0, -1,  0],
    #        [0,  0, -1]]
    #
    # Si la caméra est exactement à l'origine du robot : tx=ty=tz=0
    # Sinon mesure le décalage en mm entre l'origine caméra et la base robot.

    TX = 0.0   # décalage X caméra → base robot (mm)
    TY = 0.0   # décalage Y caméra → base robot (mm)
    TZ = 0.0   # décalage Z caméra → base robot (mm)

    T_SOURCE2ROBOT = np.array([
        [ 1,  0,  0,  TX],   # X_cam  →  X_rob
        [ 0, -1,  0,  TY],   # Y_cam  → -Y_rob  (bas image = droite robot inversé)
        [ 0,  0, -1,  TZ],   # Z_cam  → -Z_rob  (profondeur bas = -haut robot)
        [ 0,  0,  0,   1],
    ], dtype=float)

    CFG = RobotConfig(
        port="/dev/ttyTHS1",
        baudrate=1000000,
        home_x=0.0, home_y=0.0, home_z=250.0,
        speed=20,
        move_timeout=3.5,
        n_seeds=12,
        t_source2robot=T_SOURCE2ROBOT,
    )
    # ────────────────────────────────────────────────────────────────────────

    if "--sim" in sys.argv:
        print("\n=== SIMULATION aimAt ===\n")
        TARGETS_SRC = [
            (100.0,   0.0, 400.0),
            (  0.0, 150.0, 350.0),
            (-80.0,  80.0, 300.0),
            (200.0, -50.0, 500.0),
        ]
        print(f"{'Cible source':^30}  {'Angles (°)':^56}  {'Erreur':>7}  {'Temps':>7}")
        print("─" * 108)
        Q_PREV = None
        for X, Y, Z in TARGETS_SRC:
            T0       = time.perf_counter()
            TARGET_R = transformPoint(T_SOURCE2ROBOT, np.array([X, Y, Z]))
            DIRN     = unitDirection(TARGET_R, CFG.homePos)
            if DIRN is None:
                print(f"({X},{Y},{Z}) — trop proche"); continue
            Q_SOL    = solveIk(DIRN, CFG.homePos, CFG, Q_PREV)
            ELAPSED  = (time.perf_counter() - T0) * 1000
            if Q_SOL is not None:
                ANGLES = [round(math.degrees(A), 1) for A in Q_SOL]
                ERR    = pointingErrorDeg(Q_SOL, DIRN)
                print(f"({X:6.1f},{Y:6.1f},{Z:6.1f})  {str(ANGLES):^56}  {ERR:5.3f}°  {ELAPSED:5.0f} ms")
                Q_PREV = Q_SOL
            else:
                print(f"({X:6.1f},{Y:6.1f},{Z:6.1f})  IMPOSSIBLE")

    else:
        AIMER = LaserAimer(CFG)
        try:
            AIMER.connect()
            AIMER.goHome()
            time.sleep(1.0)
            TARGET = (1000.0, 800.0, 300.0)
            #TARGETS = [(150.0, 50.0, 400.0), (200.0, -80.0, 350.0), (50.0, 120.0, 300.0)]
            RESULTS = AIMER.aimAt(*TARGET, blocking=False)
            #RESULTS = AIMER.aimAtSequence(*TARGETS, 2)
            time.sleep(5)
            #for I, (T, OK) in enumerate(zip(TARGETS, RESULTS)):
            #    print(f"  [{'✓' if OK else '✗'}] Cible {I+1} {T}")
        except KeyboardInterrupt:
            log.info("Interruption.")
        finally:
            AIMER.goHome()
            AIMER.disconnect()
