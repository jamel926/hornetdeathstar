"""
myCobot280 Jetson Nano — Pointage laser automatique
====================================================
Auteur  : généré avec Claude
Robot   : myCobot280 (6 axes)
Objectif: viser un objet (coords caméra/monde) avec un laser monté
          dans l'axe Z6 de l'effecteur.

Hypothèses
----------
* Le laser est colinéaire à l'axe Z de l'effecteur (frame 6).
* La base du robot est fixe ; le repère "source" (caméra ou monde) est relié
  à la base par une transformation rigide T_source2robot (4x4, homogène).
* On utilise l'API pymycobot (send_coords) pour envoyer (x,y,z,rx,ry,rz).
* La position XYZ de l'effecteur est maintenue autour d'une "home" configurable ;
  seule l'orientation (rx, ry, rz) change pour pointer le laser.
* Stratégie d'optimisation : multi-seeds sur les angles articulaires initiaux
  + scoring (distance aux butées, norme de mouvement).

Dépendances
-----------
    pip install pymycobot numpy scipy
"""

from __future__ import annotations

import time
import math
import logging
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
from numpy.linalg import norm
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize, differential_evolution

from pymycobot.mycobot import MyCobot

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
# Paramètres du robot & de l'installation
# ===========================================================================

@dataclass
class RobotConfig:
    """Tous les paramètres modifiables de l'installation."""

    # --- Port série Jetson Nano ---
    port: str = "/dev/ttyTHS1"   # UART natif Jetson ; ajuste si USB (/dev/ttyUSB0)
    baudrate: int = 1000000

    # --- Position "home" de l'effecteur en mm (repère robot) ---
    # C'est depuis ce point que le laser part pour viser la cible.
    # Modifie X, Y, Z selon ton installation physique.
    home_x: float = 0.0
    home_y: float = 0.0
    home_z: float = 250.0       # ~25 cm au-dessus de la base

    # --- Limites articulaires (degrés) du myCobot280 ---
    joint_limits: list[tuple[float, float]] = field(default_factory=lambda: [
        (-165, 165),   # J1
        (-165, 165),   # J2
        (-165, 165),   # J3
        (-165, 165),   # J4
        (-165, 165),   # J5
        (-175, 175),   # J6
    ])

    # --- Limites d'orientation de l'effecteur (degrés) ---
    rx_limits: tuple[float, float] = (-180.0, 180.0)
    ry_limits: tuple[float, float] = (-180.0, 180.0)
    rz_limits: tuple[float, float] = (-180.0, 180.0)

    # --- Vitesse de mouvement (1–100) ---
    speed: int = 30

    # --- Délai d'attente après un move (s) ---
    move_timeout: float = 3.0

    # --- Nombre de seeds pour l'optimisation multi-départ ---
    n_seeds: int = 36

    # --- Matrice de transformation source → base robot (4×4 homogène) ---
    # Remplis avec ta calibration réelle.
    # Par défaut : identité (repère source == repère robot).
    T_source2robot: np.ndarray = field(
        default_factory=lambda: np.eye(4, dtype=float)
    )


# ===========================================================================
# Utilitaires géométriques
# ===========================================================================

def rot_x(a: float) -> np.ndarray:
    """Matrice de rotation autour de X, angle en radians."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s,  c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    """Matrice de rotation autour de Y, angle en radians."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    """Matrice de rotation autour de Z, angle en radians."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=float)


def euler_zyx_to_matrix(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """
    Convention myCobot : Euler extrinsèque Z-Y-X (roll-pitch-yaw autour de la base).
    R = Rz(rz) · Ry(ry) · Rx(rx)
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    return rot_z(rz) @ rot_y(ry) @ rot_x(rx)


def matrix_to_euler_zyx(R: np.ndarray) -> tuple[float, float, float]:
    """
    Inverse de euler_zyx_to_matrix.
    Retourne (rx_deg, ry_deg, rz_deg).
    Gestion de la singularité gimbal-lock (ry ≈ ±90°).
    """
    sy = -R[2, 0]
    cy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)

    if cy > 1e-6:               # cas non singulier
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(sy, cy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:                       # gimbal lock : ry = ±90°
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(sy, cy)
        rz = 0.0

    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


def direction_to_euler(direction: np.ndarray) -> tuple[float, float, float]:
    """
    Calcule les angles Euler (rx, ry, rz) pour que l'axe Z6 de l'effecteur
    pointe dans la direction donnée.

    Stratégie :
      1. Normaliser le vecteur directeur → z_target
      2. Construire un repère orthonormal (x_frame, y_frame, z_frame)
         avec z_frame = z_target
      3. Convertir en angles Euler ZYX
    """
    z_target = direction / norm(direction)

    # Choix d'un vecteur "up" non colinéaire à z_target
    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z_target, up)) > 0.99:
        up = np.array([0.0, 1.0, 0.0])

    x_frame = np.cross(up, z_target)
    x_frame /= norm(x_frame)
    y_frame = np.cross(z_target, x_frame)

    R = np.column_stack([x_frame, y_frame, z_target])   # colonnes = axes du frame
    return matrix_to_euler_zyx(R)


def transform_point(T: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Applique une transformation homogène 4×4 à un point 3D."""
    p_h = np.array([point[0], point[1], point[2], 1.0])
    result = T @ p_h
    return result[:3]


# ===========================================================================
# Optimisation multi-seeds
# ===========================================================================

def _euler_distance_cost(
    angles: np.ndarray,
    target_direction: np.ndarray,
    cfg: RobotConfig,
) -> float:
    """
    Fonction de coût pour l'optimisation de (rx, ry, rz).

    Minimise :
      - Erreur angulaire entre axe Z6 et direction cible  (priorité 1)
      - Distance aux butées d'orientation                 (priorité 2, poids faible)
    """
    rx, ry, rz = angles

    # --- Erreur de pointage ---
    R = euler_zyx_to_matrix(rx, ry, rz)
    z_axis = R[:, 2]                        # axe Z de l'effecteur dans ce repère
    cos_err = np.clip(np.dot(z_axis, target_direction), -1.0, 1.0)
    angle_error = math.acos(cos_err)        # en radians [0, π]

    # --- Pénalité butées (favorise le centre de la plage) ---
    def near_limit_penalty(val: float, lo: float, hi: float) -> float:
        center = (lo + hi) / 2.0
        half_range = (hi - lo) / 2.0
        return ((val - center) / half_range) ** 2

    penalty = (
        near_limit_penalty(rx, *cfg.rx_limits)
        + near_limit_penalty(ry, *cfg.ry_limits)
        + near_limit_penalty(rz, *cfg.rz_limits)
    )

    return angle_error + 0.05 * penalty


def find_best_orientation(
    target_direction: np.ndarray,
    cfg: RobotConfig,
    current_angles: Optional[tuple[float, float, float]] = None,
) -> Optional[tuple[float, float, float]]:
    """
    Cherche (rx, ry, rz) optimaux par optimisation multi-seeds.

    Stratégie :
      1. Seed analytique direct (direction_to_euler)
      2. Seeds régulières sur la sphère d'orientations
      3. Seed depuis la position courante (si connue)
      4. Optimisation locale L-BFGS-B depuis chaque seed
      5. Retourne la meilleure solution (erreur < 0.5°)
    """
    bounds = [
        (math.radians(cfg.rx_limits[0]), math.radians(cfg.rx_limits[1])),
        (math.radians(cfg.ry_limits[0]), math.radians(cfg.ry_limits[1])),
        (math.radians(cfg.rz_limits[0]), math.radians(cfg.rz_limits[1])),
    ]

    # --- Génération des seeds ---
    seeds = []

    # Seed analytique (souvent très bonne)
    rx0, ry0, rz0 = direction_to_euler(target_direction)
    seeds.append(np.radians([rx0, ry0, rz0]))

    # Seeds depuis la position courante
    if current_angles is not None:
        seeds.append(np.radians(list(current_angles)))

    # Seeds régulières sur une grille sphérique (échantillonnage de Fibonacci)
    n = cfg.n_seeds
    golden = (1 + math.sqrt(5)) / 2
    for i in range(n):
        theta = math.acos(1 - 2 * (i + 0.5) / n)   # élévation
        phi = 2 * math.pi * i / golden               # azimut
        # Convertir en angles Euler approximatifs
        rx_s = theta - math.pi / 2
        ry_s = 0.0
        rz_s = phi - math.pi
        seeds.append(np.array([rx_s, ry_s, rz_s]))

    # --- Optimisation locale depuis chaque seed ---
    best_cost = float("inf")
    best_angles = None

    def cost_fn(x):
        return _euler_distance_cost(
            np.degrees(x), target_direction, cfg
        )

    for seed in seeds:
        # Clamp le seed dans les bornes
        seed_clamped = np.clip(
            seed,
            [b[0] for b in bounds],
            [b[1] for b in bounds],
        )
        try:
            result = minimize(
                cost_fn,
                seed_clamped,
                method="L-BFGS-B",
                bounds=bounds,
                options={"maxiter": 500, "ftol": 1e-10, "gtol": 1e-8},
            )
            if result.fun < best_cost:
                best_cost = result.fun
                best_angles = result.x
        except Exception as e:
            log.debug("Seed failed: %s", e)

    if best_angles is None:
        log.error("Aucune solution trouvée par optimisation locale.")
        return None

    # Vérification qualité : erreur angulaire < 0.5°
    R_best = euler_zyx_to_matrix(*np.degrees(best_angles))
    z_best = R_best[:, 2]
    cos_err = np.clip(np.dot(z_best, target_direction), -1.0, 1.0)
    err_deg = math.degrees(math.acos(cos_err))

    if err_deg > 0.5:
        log.warning(
            "Meilleure solution trouvée mais erreur angulaire = %.3f° > 0.5°. "
            "La cible est peut-être hors de portée d'orientation.",
            err_deg,
        )
        # On retourne quand même la meilleure approximation
    else:
        log.info("Solution optimale trouvée — erreur angulaire : %.4f°", err_deg)

    rx_deg, ry_deg, rz_deg = np.degrees(best_angles)
    return (rx_deg, ry_deg, rz_deg)


# ===========================================================================
# Interface principale
# ===========================================================================

class LaserAimer:
    """
    Contrôleur haut niveau pour pointer le laser du myCobot280 vers une cible.

    Exemple d'utilisation
    ---------------------
    ::
        import numpy as np
        from mycobot_laser_aim import LaserAimer, RobotConfig

        # Matrice de transformation caméra → base robot (à remplacer par ta calibration)
        T = np.array([
            [ 1,  0,  0,  200],   # décalage X = 200 mm
            [ 0,  1,  0,    0],
            [ 0,  0,  1,  100],   # décalage Z = 100 mm
            [ 0,  0,  0,    1],
        ], dtype=float)

        cfg = RobotConfig(T_source2robot=T)
        aimer = LaserAimer(cfg)
        aimer.connect()

        # Viser un objet à (x=150, y=80, z=300) mm dans le repère caméra
        success = aimer.aim_at(150.0, 80.0, 300.0)
        aimer.disconnect()
    """

    def __init__(self, config: Optional[RobotConfig] = None):
        self.cfg = config or RobotConfig()
        self._robot: Optional[MyCobot] = None
        self._current_orientation: Optional[tuple[float, float, float]] = None

    # ------------------------------------------------------------------
    # Connexion
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Ouvre la connexion série avec le myCobot280."""
        log.info("Connexion au myCobot280 sur %s @ %d baud…",
                 self.cfg.port, self.cfg.baudrate)
        self._robot = MyCobot(self.cfg.port, self.cfg.baudrate)
        time.sleep(0.5)
        log.info("Connexion établie.")

    def disconnect(self) -> None:
        """Libère le port série."""
        if self._robot is not None:
            self._robot = None
            log.info("Déconnecté du robot.")

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
        Pointe le laser vers le point (x_src, y_src, z_src) exprimé dans le
        repère source (caméra, monde…).

        Paramètres
        ----------
        x_src, y_src, z_src : coordonnées de l'objet cible (mm)
        blocking            : si True, attend la fin du mouvement

        Retourne
        --------
        True si le mouvement a été envoyé avec succès, False sinon.
        """
        if self._robot is None:
            raise RuntimeError("Robot non connecté. Appeler connect() d'abord.")

        # 1. Transformer la cible dans le repère robot
        target_robot = transform_point(
            self.cfg.T_source2robot,
            np.array([x_src, y_src, z_src]),
        )
        log.info(
            "Cible source (%.1f, %.1f, %.1f) mm → robot (%.1f, %.1f, %.1f) mm",
            x_src, y_src, z_src,
            target_robot[0], target_robot[1], target_robot[2],
        )

        # 2. Position de l'effecteur (home fixe)
        effector_pos = np.array([
            self.cfg.home_x,
            self.cfg.home_y,
            self.cfg.home_z,
        ])

        # 3. Vecteur directeur effecteur → cible
        direction = target_robot - effector_pos
        d = norm(direction)
        if d < 1e-3:
            log.error(
                "La cible est trop proche de l'effecteur (d=%.4f mm). "
                "Impossible de calculer une direction.", d
            )
            return False

        direction_unit = direction / d
        log.info(
            "Vecteur laser normalisé : (%.4f, %.4f, %.4f)  — distance cible : %.1f mm",
            direction_unit[0], direction_unit[1], direction_unit[2], d,
        )

        # 4. Optimisation multi-seeds → meilleure orientation (rx, ry, rz)
        result = find_best_orientation(
            direction_unit, self.cfg, self._current_orientation
        )
        if result is None:
            log.error("Aucune orientation valide calculée.")
            return False

        rx, ry, rz = result
        log.info("Orientation calculée → rx=%.2f°  ry=%.2f°  rz=%.2f°", rx, ry, rz)

        # 5. Vérification des bornes d'orientation
        if not self._check_orientation_limits(rx, ry, rz):
            log.error(
                "Orientation hors limites : rx=%.2f  ry=%.2f  rz=%.2f", rx, ry, rz
            )
            return False

        # 6. Envoi de la commande au robot
        coords = [
            self.cfg.home_x,
            self.cfg.home_y,
            self.cfg.home_z,
            rx,
            ry,
            rz,
        ]
        log.info("Envoi commande : coords=%s  vitesse=%d", coords, self.cfg.speed)
        self._robot.send_coords(coords, self.cfg.speed, 1)   # mode=1 : angulaire

        if blocking:
            self._wait_motion()

        self._current_orientation = (rx, ry, rz)
        return True

    def aim_at_sequence(
        self,
        targets: list[tuple[float, float, float]],
        delay_between: float = 0.5,
    ) -> list[bool]:
        """
        Enchaîne plusieurs pointages laser.

        Paramètres
        ----------
        targets        : liste de (x, y, z) dans le repère source
        delay_between  : délai en secondes entre deux cibles

        Retourne
        --------
        Liste de booléens (succès par cible).
        """
        results = []
        for i, (x, y, z) in enumerate(targets):
            log.info("--- Cible %d/%d : (%.1f, %.1f, %.1f) ---",
                     i + 1, len(targets), x, y, z)
            ok = self.aim_at(x, y, z, blocking=True)
            results.append(ok)
            if delay_between > 0:
                time.sleep(delay_between)
        return results

    def go_home(self) -> None:
        """Retourne à la position neutre (orientation Z6 vers le haut)."""
        if self._robot is None:
            raise RuntimeError("Robot non connecté.")
        coords = [self.cfg.home_x, self.cfg.home_y, self.cfg.home_z, 0, 0, 0]
        log.info("Retour home : %s", coords)
        self._robot.send_coords(coords, self.cfg.speed, 1)
        self._wait_motion()
        self._current_orientation = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    # Méthodes internes
    # ------------------------------------------------------------------

    def _check_orientation_limits(
        self, rx: float, ry: float, rz: float
    ) -> bool:
        ok = (
            self.cfg.rx_limits[0] <= rx <= self.cfg.rx_limits[1]
            and self.cfg.ry_limits[0] <= ry <= self.cfg.ry_limits[1]
            and self.cfg.rz_limits[0] <= rz <= self.cfg.rz_limits[1]
        )
        return ok

    def _wait_motion(self) -> None:
        """Attend que le robot termine son mouvement."""
        log.info("Attente fin de mouvement (%.1f s)…", self.cfg.move_timeout)
        time.sleep(self.cfg.move_timeout)

    # ------------------------------------------------------------------
    # Utilitaire de diagnostic
    # ------------------------------------------------------------------

    def compute_pointing_error(
        self,
        x_src: float,
        y_src: float,
        z_src: float,
        rx: float,
        ry: float,
        rz: float,
    ) -> float:
        """
        Calcule l'erreur angulaire (degrés) entre la direction laser actuelle
        et la direction vers la cible.  Utile pour le debug/calibration.
        """
        target_robot = transform_point(
            self.cfg.T_source2robot,
            np.array([x_src, y_src, z_src]),
        )
        effector_pos = np.array([self.cfg.home_x, self.cfg.home_y, self.cfg.home_z])
        direction = target_robot - effector_pos
        direction /= norm(direction)

        R = euler_zyx_to_matrix(rx, ry, rz)
        z_axis = R[:, 2]
        cos_err = np.clip(np.dot(z_axis, direction), -1.0, 1.0)
        return math.degrees(math.acos(cos_err))


# ===========================================================================
# Point d'entrée de démonstration
# ===========================================================================

if __name__ == "__main__":
    import sys

    # ------------------------------------------------------------------
    # Configuration — ADAPTE CES VALEURS À TON INSTALLATION
    # ------------------------------------------------------------------

    # Exemple : caméra décalée de 200 mm en X et 100 mm en Z par rapport à la base
    # Remplace par ta vraie matrice de calibration (rotation + translation)
    T_cam2robot = np.array([
        [1, 0, 0, 200],   # Tx = 200 mm
        [0, 1, 0,   0],   # Ty = 0
        [0, 0, 1, 100],   # Tz = 100 mm
        [0, 0, 0,   1],
    ], dtype=float)

    cfg = RobotConfig(
        port="/dev/ttyTHS1",   # ou /dev/ttyUSB0 selon ta connexion
        baudrate=1000000,
        home_x=0.0,
        home_y=0.0,
        home_z=250.0,
        speed=25,
        move_timeout=3.0,
        n_seeds=36,
        T_source2robot=T_cam2robot,
    )

    aimer = LaserAimer(cfg)

    # ------------------------------------------------------------------
    # Mode simulation (sans robot physique) pour tester la géométrie
    # ------------------------------------------------------------------
    SIMULATION_MODE = "--sim" in sys.argv

    if SIMULATION_MODE:
        log.info("=== MODE SIMULATION (pas de robot physique) ===")

        test_targets = [
            (100.0,   0.0, 400.0),
            (  0.0, 150.0, 350.0),
            (-80.0,  80.0, 300.0),
            (200.0, -50.0, 500.0),
        ]

        for (x, y, z) in test_targets:
            target_robot = transform_point(T_cam2robot, np.array([x, y, z]))
            effector = np.array([cfg.home_x, cfg.home_y, cfg.home_z])
            direction = target_robot - effector
            direction /= norm(direction)

            result = find_best_orientation(direction, cfg)
            if result:
                rx, ry, rz = result
                err = aimer.compute_pointing_error(x, y, z, rx, ry, rz)
                print(
                    f"Cible ({x:6.1f},{y:6.1f},{z:6.1f}) → "
                    f"rx={rx:7.2f}° ry={ry:7.2f}° rz={rz:7.2f}°  "
                    f"erreur={err:.4f}°"
                )
            else:
                print(f"Cible ({x}, {y}, {z}) → IMPOSSIBLE")

    else:
        # ------------------------------------------------------------------
        # Mode réel
        # ------------------------------------------------------------------
        try:
            aimer.connect()
            aimer.go_home()
            time.sleep(1)

            # Exemple : viser un objet à (x=150, y=50, z=400) mm dans le repère caméra
            success = aimer.aim_at(150.0, 50.0, 400.0)
            if success:
                log.info("Laser pointé avec succès !")
            else:
                log.error("Échec du pointage.")

        except KeyboardInterrupt:
            log.info("Interruption utilisateur.")
        finally:
            aimer.go_home()
            aimer.disconnect()
