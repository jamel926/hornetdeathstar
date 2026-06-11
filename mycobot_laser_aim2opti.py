"""
myCobot280 Jetson Nano — Pointage laser (optimisé)
===================================================
Auteur  : généré avec Claude
Robot   : myCobot280 Jetson Nano (6 axes)

Optimisations appliquées :
  1. FK compilée JIT avec Numba (10× plus rapide que numpy pur)
  2. Warmstart agressif : repart de la solution précédente + petites perturbations
  3. Arrêt anticipé dès qu'une solution de qualité suffisante est trouvée
  4. Parallélisation des seeds sur les 4 cœurs du Jetson Nano
  5. send_angles uniquement (send_coords non fonctionnel sur firmware JN)

Dépendances :
    pip install pymycobot numpy scipy numba
"""

from __future__ import annotations

import time
import math
import logging
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from numpy.linalg import norm
from scipy.optimize import minimize

# Numba — import conditionnel : si absent, fallback numpy transparent
try:
    from numba import njit
    NUMBA_OK = True
except ImportError:
    NUMBA_OK = False
    def njit(*args, **kwargs):          # décorateur no-op
        def wrapper(fn):
            return fn
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

# Tableaux numpy pour la FK vectorisée
_DH_ALPHA  = np.array([0.0, math.pi/2,  0.0,       0.0,      math.pi/2, -math.pi/2])
_DH_A      = np.array([0.0, 0.0,       -110.4,    -96.0,     0.0,        0.0       ])
_DH_D      = np.array([131.56, 0.0,     0.0,       63.4,     75.05,      45.6      ])
_DH_TOFF   = np.array([0.0, -math.pi/2, 0.0,      -math.pi/2, 0.0,       0.0      ])

# Version plates (pour Numba qui n'accepte pas de tableaux globaux capturés)
_DH_FLAT = np.array([
    [0.0,        0.0,    131.56,   0.0      ],
    [math.pi/2,  0.0,      0.0,   -math.pi/2],
    [0.0,       -110.4,    0.0,    0.0      ],
    [0.0,        -96.0,   63.4,   -math.pi/2],
    [math.pi/2,   0.0,   75.05,    0.0      ],
    [-math.pi/2,  0.0,   45.6,     0.0      ],
], dtype=np.float64)

JOINT_LIMITS_DEG = [(-165,165),(-165,165),(-165,165),(-165,165),(-165,165),(-175,175)]
JOINT_LIMITS_RAD = [(math.radians(lo), math.radians(hi)) for lo, hi in JOINT_LIMITS_DEG]


# ===========================================================================
# FK — version Numba JIT (compilée au premier appel, ~2 s)
# ===========================================================================

@njit(cache=True)
def _fk_numba(q: np.ndarray, dh: np.ndarray) -> tuple:
    """
    Cinématique directe compilée JIT.
    Retourne (px, py, pz, zx, zy, zz) :
      position de l'effecteur + axe Z6 dans le repère base.
    dh : tableau (6,4) = [alpha, a, d, theta_offset] par ligne.
    """
    T = np.eye(4)
    for i in range(6):
        alpha = dh[i, 0]
        a     = dh[i, 1]
        d     = dh[i, 2]
        toff  = dh[i, 3]
        theta = q[i] + toff
        ct = math.cos(theta);  st = math.sin(theta)
        ca = math.cos(alpha);  sa = math.sin(alpha)
        M = np.array([
            [ct,     -st,    0.0,   a   ],
            [st*ca,   ct*ca, -sa,  -sa*d],
            [st*sa,   ct*sa,  ca,   ca*d],
            [0.0,     0.0,   0.0,   1.0 ],
        ])
        T = T @ M
    # position (colonne 3) + axe Z (colonne 2)
    return T[0,3], T[1,3], T[2,3], T[0,2], T[1,2], T[2,2]


def fk_numpy(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fallback FK numpy (si Numba absent)."""
    T = np.eye(4)
    for i in range(6):
        alpha, a, d, toff = _DH_FLAT[i]
        theta = q[i] + toff
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        M = np.array([
            [ct,    -st,    0.0,  a   ],
            [st*ca,  ct*ca, -sa, -sa*d],
            [st*sa,  ct*sa,  ca,  ca*d],
            [0.0,    0.0,   0.0,  1.0 ],
        ])
        T = T @ M
    return T[:3, 3], T[:3, 2]


def fk(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Cinématique directe — dispatche sur Numba ou numpy selon disponibilité.
    Retourne (position_mm, axe_z6), chacun array (3,).
    """
    if NUMBA_OK:
        px, py, pz, zx, zy, zz = _fk_numba(q, _DH_FLAT)
        return np.array([px, py, pz]), np.array([zx, zy, zz])
    return fk_numpy(q)


# Premier appel Numba pour déclencher la compilation à l'import
if NUMBA_OK:
    try:
        _fk_numba(np.zeros(6), _DH_FLAT)
        log.info("Numba JIT compilé et prêt.")
    except Exception as e:
        log.warning("Numba disponible mais compilation échouée (%s) → fallback numpy.", e)
        NUMBA_OK = False


# ===========================================================================
# Configuration
# ===========================================================================

@dataclass
class RobotConfig:
    """Tous les paramètres modifiables de l'installation."""

    port: str     = "/dev/ttyTHS1"
    baudrate: int = 1000000

    # Position home de l'effecteur (mm, repère robot)
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 250.0

    speed: int         = 25
    move_timeout: float = 3.5

    # --- Paramètres IK ---
    n_seeds: int             = 12     # réduit vs version précédente (warmstart compense)
    n_perturbations: int     = 8      # perturbations autour de la solution précédente
    early_stop_deg: float    = 0.15   # arrêt anticipé si erreur < 0.15°
    pointing_tolerance_deg: float = 1.0
    n_workers: int           = 4      # cœurs Jetson Nano pour la parallélisation

    # Poids de la fonction de coût
    w_direction:  float = 10.0
    w_position:   float = 2.0
    w_limits:     float = 1.0
    w_smoothness: float = 0.5

    # Matrice de transformation source → base robot (4×4 homogène)
    T_source2robot: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=float)
    )


# ===========================================================================
# Génération des seeds (warmstart agressif)
# ===========================================================================

def _generate_seeds(cfg: RobotConfig, q_current: Optional[np.ndarray]) -> list[np.ndarray]:
    """
    Priorité aux solutions proches de la position précédente.
    Complète avec quelques seeds cardinales et aléatoires.
    """
    rng = np.random.default_rng(42)
    seeds = []

    # --- Warmstart : solution précédente + perturbations croissantes ---
    if q_current is not None:
        seeds.append(q_current.copy())                          # solution exacte
        for sigma in [0.05, 0.12, 0.25]:                        # petites → grandes
            for _ in range(cfg.n_perturbations // 3 + 1):
                p = q_current + rng.normal(0.0, sigma, 6)
                seeds.append(p)

    # --- Position neutre (toujours incluse comme fallback) ---
    seeds.append(np.zeros(6))

    # --- Configurations cardinales utiles pour le pointage ---
    for cfg_deg in [
        [0,  0,  0,  0,  90,  0],
        [0,  0,  0,  0, -90,  0],
        [0, 45, -45,  0,  90,  0],
        [0, -45, 45,  0, -90,  0],
        [90,  0,  0,  0,   0,  0],
        [-90, 0,  0,  0,   0,  0],
        [0,  90,  0,  0,   0,  0],
        [0, -90,  0,  0,   0,  0],
    ]:
        seeds.append(np.radians(cfg_deg))

    # --- Seeds aléatoires pour la diversité ---
    for _ in range(cfg.n_seeds):
        q_rand = np.array([
            rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS_RAD
        ])
        seeds.append(q_rand)

    return seeds


# ===========================================================================
# Fonction de coût IK (module-level pour la sérialisation multiprocessing)
# ===========================================================================

def _ik_cost(
    q: np.ndarray,
    target_direction: np.ndarray,
    target_position: np.ndarray,
    w_direction: float,
    w_position: float,
    w_limits: float,
    w_smoothness: float,
    q_current: Optional[np.ndarray],
) -> float:
    pos, z_axis = fk(q)

    # 1. Erreur de pointage [0, π]
    cos_err = float(np.clip(np.dot(z_axis, target_direction), -1.0, 1.0))
    angle_err = math.acos(cos_err)

    # 2. Erreur de position vs home (normalisée en mètres)
    pos_err = norm(pos - target_position) / 1000.0

    # 3. Pénalité butées articulaires
    lim_penalty = 0.0
    for i, (lo, hi) in enumerate(JOINT_LIMITS_RAD):
        center = (lo + hi) / 2.0
        half   = (hi - lo) / 2.0
        lim_penalty += ((q[i] - center) / half) ** 2

    # 4. Fluidité (distance à la solution précédente)
    smooth = float(norm(q - q_current)) if q_current is not None else 0.0

    return (
        w_direction  * angle_err
        + w_position * pos_err
        + w_limits   * lim_penalty
        + w_smoothness * smooth
    )


# Wrapper picklable pour ProcessPoolExecutor
def _optimize_seed(args: tuple) -> tuple[np.ndarray, float]:
    """Optimise une seed unique — exécuté dans un worker process."""
    (seed, target_direction, target_position,
     w_direction, w_position, w_limits, w_smoothness,
     q_current_list) = args

    q_current = np.array(q_current_list) if q_current_list is not None else None
    bounds = JOINT_LIMITS_RAD

    def cost(q):
        return _ik_cost(
            q, target_direction, target_position,
            w_direction, w_position, w_limits, w_smoothness,
            q_current,
        )

    try:
        result = minimize(
            cost, seed,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": 800, "ftol": 1e-11, "gtol": 1e-8},
        )
        _, z_axis = fk(result.x)
        cos_err = float(np.clip(np.dot(z_axis, target_direction), -1.0, 1.0))
        err_deg = math.degrees(math.acos(cos_err))
        return result.x, err_deg
    except Exception:
        return seed, 180.0      # seed invalide → erreur maximale


# ===========================================================================
# IK principale — parallélisation + arrêt anticipé
# ===========================================================================

def solve_ik(
    target_direction: np.ndarray,
    target_position: np.ndarray,
    cfg: RobotConfig,
    q_current: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """
    Cinématique inverse orientée laser avec toutes les optimisations.

    Stratégie :
      1. Warmstart : seeds centrées sur la solution précédente
      2. Parallélisation sur n_workers cœurs
      3. Arrêt anticipé dès early_stop_deg atteint
      4. Fallback séquentiel si multiprocessing indisponible
    """
    t0 = time.perf_counter()

    seeds = _generate_seeds(cfg, q_current)

    # Prépare les arguments (picklables — pas d'array numpy dans q_current_list)
    q_cur_list = q_current.tolist() if q_current is not None else None
    args_list = [
        (
            np.clip(s, [b[0] for b in JOINT_LIMITS_RAD], [b[1] for b in JOINT_LIMITS_RAD]),
            target_direction,
            target_position,
            cfg.w_direction,
            cfg.w_position,
            cfg.w_limits,
            cfg.w_smoothness,
            q_cur_list,
        )
        for s in seeds
    ]

    best_q:       Optional[np.ndarray] = None
    best_err_deg: float = 180.0
    seeds_tested: int   = 0

    # --- Tentative parallèle ---
    try:
        with ProcessPoolExecutor(max_workers=cfg.n_workers) as executor:
            futures = {executor.submit(_optimize_seed, a): a for a in args_list}
            for future in as_completed(futures):
                seeds_tested += 1
                try:
                    q_sol, err_deg = future.result()
                    if err_deg < best_err_deg:
                        best_err_deg = err_deg
                        best_q = q_sol
                    # Arrêt anticipé
                    if best_err_deg < cfg.early_stop_deg:
                        log.debug("Arrêt anticipé après %d seeds (err=%.4f°).",
                                  seeds_tested, best_err_deg)
                        executor.shutdown(wait=False, cancel_futures=True)
                        break
                except Exception as e:
                    log.debug("Future error: %s", e)

    except Exception as e:
        # Fallback séquentiel (environnements sans fork, débogage…)
        log.warning("Parallélisation indisponible (%s) → mode séquentiel.", e)
        for args in args_list:
            q_sol, err_deg = _optimize_seed(args)
            seeds_tested += 1
            if err_deg < best_err_deg:
                best_err_deg = err_deg
                best_q = q_sol
            if best_err_deg < cfg.early_stop_deg:
                log.debug("Arrêt anticipé séquentiel après %d seeds.", seeds_tested)
                break

    elapsed = (time.perf_counter() - t0) * 1000

    if best_q is None:
        log.error("IK : aucune solution trouvée.")
        return None

    log.info("IK : erreur=%.4f°  seeds=%d/%d  temps=%.1f ms  numba=%s",
             best_err_deg, seeds_tested, len(seeds), elapsed, NUMBA_OK)

    if best_err_deg > cfg.pointing_tolerance_deg:
        log.warning("IK : erreur %.3f° dépasse la tolérance %.1f°.",
                    best_err_deg, cfg.pointing_tolerance_deg)

    return best_q


# ===========================================================================
# Utilitaires géométriques
# ===========================================================================

def transform_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    p_h = np.array([point[0], point[1], point[2], 1.0])
    return (T @ p_h)[:3]


# ===========================================================================
# Interface principale
# ===========================================================================

class LaserAimer:
    """
    Contrôleur haut niveau pour pointer le laser du myCobot280.

    Exemple
    -------
    ::
        import numpy as np
        from mycobot_laser_aim import LaserAimer, RobotConfig

        T = np.array([[1,0,0,200],[0,1,0,0],[0,0,1,100],[0,0,0,1]], dtype=float)
        cfg = RobotConfig(T_source2robot=T)
        aimer = LaserAimer(cfg)
        aimer.connect()
        aimer.aim_at(150.0, 80.0, 300.0)
        aimer.disconnect()
    """

    def __init__(self, config: Optional[RobotConfig] = None):
        self.cfg    = config or RobotConfig()
        self._robot: Optional[MyCobot280] = None
        self._q_current: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Connexion
    # ------------------------------------------------------------------

    def connect(self) -> None:
        log.info("Connexion sur %s @ %d baud…", self.cfg.port, self.cfg.baudrate)
        self._robot = MyCobot280(self.cfg.port, self.cfg.baudrate)
        time.sleep(1.0)

        if self._robot.get_fresh_mode() != 1:
            self._robot.set_fresh_mode(1)
            time.sleep(0.3)
            log.info("fresh_mode activé.")

        self._sync_angles()
        log.info("Connecté. Position courante : %s°",
                 np.round(np.degrees(self._q_current), 1).tolist()
                 if self._q_current is not None else "inconnue")

    def disconnect(self) -> None:
        if self._robot is not None:
            self._robot = None
            log.info("Déconnecté.")

    # ------------------------------------------------------------------
    # API publique
    # ------------------------------------------------------------------

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

        # 1. Transformation dans le repère robot
        target = transform_point(
            self.cfg.T_source2robot, np.array([x_src, y_src, z_src])
        )
        log.info("Cible robot : (%.1f, %.1f, %.1f) mm", *target)

        # 2. Vecteur directeur
        home = np.array([self.cfg.home_x, self.cfg.home_y, self.cfg.home_z])
        direction = target - home
        d = norm(direction)
        if d < 1.0:
            log.error("Cible trop proche de l'effecteur (%.2f mm).", d)
            return False
        direction /= d

        # 3. IK optimisée
        q_sol = solve_ik(direction, home, self.cfg, self._q_current)
        if q_sol is None:
            return False

        # 4. Envoi
        angles_deg = [round(math.degrees(a), 2) for a in q_sol]
        log.info("→ send_angles %s", angles_deg)
        self._robot.send_angles(angles_deg, self.cfg.speed)

        if blocking:
            self._wait_motion()

        # 5. Mise à jour de la position courante (warmstart prochain appel)
        self._q_current = q_sol
        self._sync_angles()
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

    def go_home(self) -> None:
        """Retourne à la position neutre (angles = 0°)."""
        if self._robot is None:
            raise RuntimeError("Robot non connecté.")
        log.info("Retour home…")
        self._robot.send_angles([0, 0, 0, 0, 0, 0], self.cfg.speed)
        self._wait_motion()
        self._q_current = np.zeros(6)

    def get_pointing_error_deg(
        self, x_src: float, y_src: float, z_src: float
    ) -> float:
        """Erreur angulaire (°) entre direction courante et cible."""
        if self._q_current is None:
            return float("inf")
        target = transform_point(
            self.cfg.T_source2robot, np.array([x_src, y_src, z_src])
        )
        home = np.array([self.cfg.home_x, self.cfg.home_y, self.cfg.home_z])
        direction = target - home
        direction /= norm(direction)
        _, z_axis = fk(self._q_current)
        cos_e = float(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
        return math.degrees(math.acos(cos_e))

    # ------------------------------------------------------------------
    # Internes
    # ------------------------------------------------------------------

    def _sync_angles(self) -> None:
        """Relit les angles réels depuis le robot (warmstart suivant)."""
        if self._robot is None:
            return
        try:
            angles = self._robot.get_angles()
            if angles and len(angles) == 6:
                self._q_current = np.radians(angles)
        except Exception as e:
            log.debug("Lecture angles échouée : %s", e)

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
# Point d'entrée — démonstration / benchmark
# ===========================================================================

if __name__ == "__main__":
    import sys

    # ── ADAPTE ICI ──────────────────────────────────────────────────────────
    T_source2robot = np.array([
        [1, 0, 0, 0],
        [0, 1, 0,   0],
        [0, 0, 1, 0],
        [0, 0, 0,   1],
    ], dtype=float)

    cfg = RobotConfig(
        port="/dev/ttyTHS1",
        baudrate=1000000,
        home_x=0.0, home_y=0.0, home_z=250.0,
        speed=20,
        move_timeout=5.0,
        n_seeds=12,
        n_perturbations=8,
        early_stop_deg=0.15,
        n_workers=4,
        T_source2robot=T_source2robot,
    )
    # ────────────────────────────────────────────────────────────────────────

    # ── Mode simulation + benchmark ─────────────────────────────────────────
    if "--sim" in sys.argv:
        log.info("=== MODE SIMULATION ===")
        aimer = LaserAimer(cfg)

        targets = [
            (100.0,   0.0, 400.0),
            (  0.0, 150.0, 350.0),
            (-80.0,  80.0, 300.0),
            (200.0, -50.0, 500.0),
            (300.0,   0.0, 100.0),
        ]

        print(f"\n{'Cible':^30} {'Angles (°)':^55} {'Erreur':>8}  {'Temps':>8}")
        print("─" * 110)

        q_prev = None
        for x, y, z in targets:
            t0 = time.perf_counter()
            target_r = transform_point(T_source2robot, np.array([x, y, z]))
            home     = np.array([cfg.home_x, cfg.home_y, cfg.home_z])
            dirn     = target_r - home;  dirn /= norm(dirn)
            q_sol    = solve_ik(dirn, home, cfg, q_prev)
            elapsed  = (time.perf_counter() - t0) * 1000

            if q_sol is not None:
                angles   = [round(math.degrees(a), 1) for a in q_sol]
                _, z_ax  = fk(q_sol)
                cos_e    = float(np.clip(np.dot(z_ax, dirn), -1.0, 1.0))
                err      = math.degrees(math.acos(cos_e))
                print(f"({x:6.1f},{y:6.1f},{z:6.1f})  {str(angles):^55}  {err:6.3f}°  {elapsed:6.0f} ms")
                q_prev = q_sol
            else:
                print(f"({x:6.1f},{y:6.1f},{z:6.1f})  IMPOSSIBLE")

    # ── Mode réel ───────────────────────────────────────────────────────────
    else:
        aimer = LaserAimer(cfg)
        try:
            aimer.connect()
            aimer.go_home()
            time.sleep(1.0)

            targets = [
                (220.0, 0, 250.0),
                (0, 220.0, 250),
                ( 50.0, -120.0, 250.0),
            ]
            results = aimer.aim_at_sequence(targets, delay_between=2)
            for i, (t, ok) in enumerate(zip(targets, results)):
                print(f"  [{'✓' if ok else '✗'}] Cible {i+1} {t}")

        except KeyboardInterrupt:
            log.info("Interruption.")
        finally:
            aimer.go_home()
            aimer.disconnect()
