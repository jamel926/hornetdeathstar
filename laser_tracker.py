"""
laser_tracker.py
================
myCobot280 Jetson Nano — Tracking laser temps réel (petits mouvements)

Ce fichier importe mycobot_laser_aim.py pour :
  - La FK (fk), les constantes DH, JOINT_LIMITS_RAD
  - La configuration robot (RobotConfig)
  - L'IK complète (solve_ik) utilisée pour les grands mouvements
  - Le connexion robot (LaserAimer)
  - Les utilitaires géométriques (transform_point, unit_direction, pointing_error_deg)

Ce fichier ajoute :
  - Filtre de Kalman 3D (position + vitesse)
  - Jacobienne numérique de l'axe Z6
  - IK allégée (fallback Jacobienne, 1 seed, peu d'itérations)
  - TrackerConfig : paramètres spécifiques au tracking
  - LaserTracker  : boucle temps réel 25 Hz

Architecture de décision à chaque frame
----------------------------------------
    detecterObjet()
         │
         ▼
    Kalman / Simple  ← filtrage + prédiction latence
         │
         ▼
    δ > move_threshold_mm ?
     ├─ OUI → solve_ik() complète  (grand mouvement)
     └─ NON → Jacobienne           (tracking rapide ~1–2 ms)
                   │
                   └─ err > jacobian_diverge_deg ?
                         → IK allégée (fallback)

Deux modes de prédiction :
    PredictionMode.SIMPLE  : suit la position brute (défaut)
    PredictionMode.KALMAN  : filtre de Kalman, anticipe la latence du bras

Dépendances
-----------
    pip install pymycobot numpy scipy numba
    (mycobot_laser_aim.py doit être dans le même dossier)

Commandes
---------
    python laser_tracker.py --sim            # simulation mode simple
    python laser_tracker.py --sim --kalman   # simulation mode Kalman
    python laser_tracker.py                  # robot réel mode simple
    python laser_tracker.py --kalman         # robot réel mode Kalman
"""

from __future__ import annotations

import math
import time
import logging
import threading
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional

import numpy as np
from numpy.linalg import norm, pinv
from scipy.optimize import minimize

# ---------------------------------------------------------------------------
# Import depuis mycobot_laser_aim
# Tout ce qui est partagé vient d'ici — pas de duplication de code.
# ---------------------------------------------------------------------------
from mycobot_laser_aim import (
    # Cinématique
    fk,
    DH_FLAT,
    JOINT_LIMITS_RAD,
    # Configuration
    RobotConfig,
    # IK complète (grands mouvements)
    solve_ik,
    # Classe de connexion + aim_at
    LaserAimer,
    # Utilitaires géométriques
    transform_point,
    unit_direction,
    pointing_error_deg,
)

from PU_detection_couleur import detecterObjet, libererCamera

log = logging.getLogger("tracker")


# ===========================================================================
# Mode de prédiction
# ===========================================================================

class PredictionMode(Enum):
    SIMPLE = auto()   # suit la position brute détectée
    KALMAN = auto()   # filtre de Kalman + compensation latence bras


# ===========================================================================
# Configuration du tracker
# ===========================================================================

@dataclass
class TrackerConfig:
    """
    Paramètres spécifiques au tracking temps réel.
    Complète RobotConfig (qui reste dans mycobot_laser_aim.py).
    """
    # Seuil de basculement tracking rapide → IK complète (mm)
    move_threshold_mm:    float = 10.0

    # Seuil de divergence Jacobienne → IK allégée (degrés)
    jacobian_diverge_deg: float = 3.0

    # Vitesse robot en tracking (plus élevée que aim_at)
    tracking_speed:       int   = 50

    # Fréquence cible de la boucle (Hz)
    loop_hz:              float = 25.0

    # Pas de différentiation pour la Jacobienne numérique (rad)
    jacobian_eps:         float = 1e-4

    # Amplitude max d'un pas Jacobien (rad)
    jacobian_step_size:   float = 0.4

    # IK allégée (fallback Jacobienne)
    ik_light_maxiter:     int   = 80
    ik_light_ftol:        float = 1e-7

    # Mode de prédiction
    prediction_mode: PredictionMode = PredictionMode.SIMPLE

    # Paramètres Kalman
    kalman_process_noise: float = 0.1    # bruit du modèle Q
    kalman_measure_noise: float = 5.0    # bruit caméra R (mm²)
    kalman_latency_s:     float = 0.05   # latence bras à compenser (s)

    # Fenêtre des stats temps réel
    stats_window: int = 30


# ===========================================================================
# Filtre de Kalman 3D (position + vitesse)
# ===========================================================================

class KalmanFilter3D:
    """
    Filtre de Kalman pour position 3D avec modèle vitesse constante.

    État  : [x, y, z, vx, vy, vz]ᵀ  — position mm + vitesse mm/s
    Mesure: [x, y, z]ᵀ
    """

    def __init__(self, cfg: TrackerConfig):
        self.cfg = cfg
        self._ok = False
        self.x   = np.zeros(6)
        self.P   = np.eye(6) * 500.0
        # Matrice d'observation : on mesure (x,y,z) seulement
        self.H   = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.R   = np.eye(3) * cfg.kalman_measure_noise

    def _build_FQ(self, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Matrice de transition F et bruit de processus Q pour un pas dt."""
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        q  = self.cfg.kalman_process_noise
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        Q  = np.zeros((6, 6))
        for i in range(3):
            Q[i,   i  ] = q * dt4 / 4.0
            Q[i,   i+3] = q * dt3 / 2.0
            Q[i+3, i  ] = q * dt3 / 2.0
            Q[i+3, i+3] = q * dt2
        return F, Q

    def update(self, measurement: np.ndarray, dt: float) -> np.ndarray:
        """Met à jour le filtre et retourne la position estimée (mm)."""
        if not self._ok:
            self.x[:3] = measurement
            self._ok   = True
            return measurement.copy()
        F, Q     = self._build_FQ(max(dt, 1e-3))
        xp       = F @ self.x
        Pp       = F @ self.P @ F.T + Q
        S        = self.H @ Pp @ self.H.T + self.R
        K        = Pp @ self.H.T @ np.linalg.inv(S)
        self.x   = xp + K @ (measurement - self.H @ xp)
        self.P   = (np.eye(6) - K @ self.H) @ Pp
        return self.x[:3].copy()

    def predict_ahead(self, dt_ahead: float) -> np.ndarray:
        """Prédit la position dans dt_ahead secondes (compensation latence bras)."""
        if not self._ok:
            return self.x[:3].copy()
        return self.x[:3] + self.x[3:] * dt_ahead

    def reset(self) -> None:
        self._ok = False
        self.x[:]= 0.0
        self.P   = np.eye(6) * 500.0


# ===========================================================================
# Jacobienne numérique + pas de contrôle
# ===========================================================================

def _jacobian_z(q: np.ndarray, eps: float) -> np.ndarray:
    """
    Jacobienne numérique (3×6) de l'axe Z6 par différences finies centrées.
    J[i,j] = ∂(z_axis[i]) / ∂(q[j])
    """
    J = np.zeros((3, 6))
    for j in range(6):
        qp = q.copy(); qp[j] += eps; _, zp = fk(qp)
        qm = q.copy(); qm[j] -= eps; _, zm = fk(qm)
        J[:, j] = (zp - zm) / (2.0 * eps)
    return J


def jacobian_step(
    q:          np.ndarray,
    z_cur:      np.ndarray,
    target_dir: np.ndarray,
    step:       float,
    eps:        float,
) -> tuple[np.ndarray, float]:
    """
    Un pas de contrôle Jacobien pseudo-inverse.
    Retourne (q_new, erreur_deg).
    """
    dz    = target_dir - z_cur
    J     = _jacobian_z(q, eps)
    dq    = pinv(J) @ dz
    dn    = norm(dq)
    if dn > step:
        dq = dq * (step / dn)
    lo = [b[0] for b in JOINT_LIMITS_RAD]
    hi = [b[1] for b in JOINT_LIMITS_RAD]
    q_new = np.clip(q + dq, lo, hi)
    _, z_new = fk(q_new)
    err   = math.degrees(math.acos(float(np.clip(np.dot(z_new, target_dir), -1.0, 1.0))))
    return q_new, err


# ===========================================================================
# IK allégée (fallback Jacobienne)
# ===========================================================================

def _ik_light(
    target_dir: np.ndarray,
    q_seed:     np.ndarray,
    cfg:        TrackerConfig,
) -> tuple[Optional[np.ndarray], float]:
    """
    IK ultra-légère : 1 seed (position précédente), peu d'itérations.
    Fallback quand la Jacobienne diverge trop.
    Retourne (q_solution, erreur_deg).
    """
    def cost(q):
        _, z = fk(q)
        return math.acos(float(np.clip(np.dot(z, target_dir), -1.0, 1.0)))
    try:
        res = minimize(cost, q_seed, method="L-BFGS-B",
                       bounds=JOINT_LIMITS_RAD,
                       options={"maxiter": cfg.ik_light_maxiter,
                                "ftol":    cfg.ik_light_ftol})
        _, z  = fk(res.x)
        err   = math.degrees(math.acos(float(np.clip(np.dot(z, target_dir), -1.0, 1.0))))
        return res.x, err
    except Exception as e:
        log.debug("IK allégée échouée : %s", e)
        return None, 180.0


# ===========================================================================
# Stats temps réel
# ===========================================================================

class TrackingStats:
    """Métriques collectées en temps réel sur une fenêtre glissante."""

    def __init__(self, window: int = 30):
        self.times   = deque(maxlen=window)
        self.errors  = deque(maxlen=window)
        self.methods = deque(maxlen=window)

    def record(self, ms: float, err: float, method: str) -> None:
        self.times.append(ms)
        self.errors.append(err)
        self.methods.append(method)

    def summary(self) -> str:
        if not self.times:
            return "Pas encore de données."
        m      = list(self.methods)
        counts = {k: m.count(k) for k in set(m)}
        return (
            f"Latence moy : {np.mean(self.times):.1f} ms  |  "
            f"Erreur moy  : {np.mean(self.errors):.2f}°  |  "
            f"Méthodes : {counts}"
        )


# ===========================================================================
# Tracker principal
# ===========================================================================

class LaserTracker:
    """
    Tracker laser temps réel pour myCobot280.

    Reçoit une LaserAimer déjà connectée (ou crée la connexion elle-même)
    et démarre une boucle à ~25 Hz qui appelle detecterObjet() à chaque frame.

    Branchement automatique :
      δ > move_threshold_mm  →  solve_ik() complet   (grand mouvement)
      δ ≤ move_threshold_mm  →  Jacobienne           (tracking rapide)
                                    ↓ diverge ?
                                 IK allégée           (fallback)

    Exemple minimal
    ---------------
    ::
        from mycobot_laser_aim import RobotConfig, LaserAimer
        from laser_tracker import LaserTracker, TrackerConfig, PredictionMode
        import numpy as np

        def detecterObjet():
            # ta détection YOLO / RealSense…
            return (x_mm, y_mm, z_mm)   # repère caméra, ou None si perdu

        robot_cfg   = RobotConfig(T_source2robot=np.eye(4))
        tracker_cfg = TrackerConfig(prediction_mode=PredictionMode.KALMAN)

        tracker = LaserTracker(robot_cfg, tracker_cfg, detecterObjet)
        tracker.start()          # connexion + home + démarrage boucle
        # … le bras suit l'objet …
        tracker.stop()           # arrêt + home + déconnexion
    """

    def __init__(
        self,
        robot_cfg:   RobotConfig,
        tracker_cfg: TrackerConfig,
        detect_fn:   Callable[[], Optional[tuple[float, float, float]]],
        aimer:       Optional[LaserAimer] = None,
    ):
        """
        Paramètres
        ----------
        robot_cfg   : configuration robot (partagée avec LaserAimer)
        tracker_cfg : paramètres du tracking temps réel
        detect_fn   : fonction de détection → (x,y,z) ou None
        aimer       : LaserAimer déjà instanciée (optionnel).
                      Si None, LaserTracker en crée une en interne.
        """
        self.rcfg   = robot_cfg
        self.tcfg   = tracker_cfg
        self.detect = detect_fn
        self.stats  = TrackingStats(tracker_cfg.stats_window)

        # LaserAimer : créé en interne ou fourni de l'extérieur
        self._aimer       = aimer if aimer is not None else LaserAimer(robot_cfg)
        self._owns_aimer  = aimer is None   # True = on gère connect/disconnect

        self._kalman      = KalmanFilter3D(tracker_cfg)
        self._last_target: Optional[np.ndarray] = None
        self._last_t:      float = 0.0
        self._initialized: bool  = False

        self._running = False
        self._thread: Optional[threading.Thread] = None

    # --- Propriétés raccourcis -------------------------------------------

    @property
    def _robot(self):
        return self._aimer.robot

    @property
    def _q_cur(self) -> Optional[np.ndarray]:
        return self._aimer.q_current

    @_q_cur.setter
    def _q_cur(self, q: Optional[np.ndarray]):
        self._aimer.q_current = q

    # --- Démarrage / arrêt -----------------------------------------------

    def start(self) -> None:
        """
        Connecte le robot (si nécessaire), va au home,
        puis démarre la boucle de tracking dans un thread dédié.
        """
        log.info("Démarrage tracker (mode=%s)…", self.tcfg.prediction_mode.name)

        if self._owns_aimer:
            self._aimer.connect()

        self._aimer.go_home()
        time.sleep(0.5)

        self._running = True
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("Tracker démarré.")

    def stop(self) -> None:
        """
        Arrête la boucle, retourne au home,
        déconnecte le robot (si connexion interne).
        """
        log.info("Arrêt du tracker…")
        self._running = False
        if self._thread:
            self._thread.join(timeout=4.0)

        try:
            self._aimer.go_home()
        except Exception as e:
            log.warning("Erreur go_home à l'arrêt : %s", e)

        if self._owns_aimer:
            self._aimer.disconnect()

        # Libération de la caméra OpenCV
        try:
            libererCamera()
        except Exception as e:
            log.debug("Libération caméra : %s", e)

        log.info("Tracker arrêté.")
        log.info("Stats finales : %s", self.stats.summary())

    # --- Boucle principale -----------------------------------------------

    def _loop(self) -> None:
        dt_target  = 1.0 / self.tcfg.loop_hz
        self._last_t = time.perf_counter()
        while self._running:
            t0 = time.perf_counter()
            try:
                self._tick()
            except Exception as e:
                log.error("Erreur tick : %s", e, exc_info=True)
            sleep = dt_target - (time.perf_counter() - t0)
            if sleep > 0:
                time.sleep(sleep)

    def _tick(self) -> None:
        """Un cycle complet de détection + décision + envoi."""
        t_now = time.perf_counter()
        dt    = t_now - self._last_t
        self._last_t = t_now

        # 1. Détection
        raw = self.detect()
        if raw is None:
            self.stats.record(0.0, 0.0, "skip_no_detect")
            return

        # 2. Transformation repère robot
        target_r = transform_point(
            self.rcfg.T_source2robot, np.array(raw, dtype=float)
        )

        # 3. Filtrage / prédiction
        if self.tcfg.prediction_mode == PredictionMode.KALMAN:
            self._kalman.update(target_r, max(dt, 1e-3))
            target_use = self._kalman.predict_ahead(self.tcfg.kalman_latency_s)
        else:
            target_use = target_r

        # 4. Décision mouvement
        t_calc = time.perf_counter()

        if not self._initialized or self._last_target is None:
            err    = self._do_ik_full(target_use)
            method = "aim_at_init"
        else:
            delta = float(norm(target_use - self._last_target))
            if delta > self.tcfg.move_threshold_mm:
                err    = self._do_ik_full(target_use)
                method = "aim_at_large"
            else:
                err, method = self._do_jacobian(target_use)

        ms = (time.perf_counter() - t_calc) * 1000
        self._last_target = target_use.copy()
        self.stats.record(ms, err, method)

        log.debug("[%s] dt=%.0fms  err=%.2f°  calc=%.1fms  %s",
                  self.tcfg.prediction_mode.name, dt * 1000, err, ms, method)

        # Affichage périodique des stats (toutes les stats_window frames)
        if len(self.stats.times) % self.tcfg.stats_window == 0:
            log.info(self.stats.summary())

    # --- Méthodes de mouvement ------------------------------------------

    def _do_ik_full(self, target: np.ndarray) -> float:
        """IK complète via solve_ik — pour grands mouvements."""
        direction = unit_direction(target, self.rcfg.home_pos)
        if direction is None:
            return 0.0

        q_sol = solve_ik(direction, self.rcfg.home_pos, self.rcfg, self._q_cur)
        if q_sol is None:
            return 180.0

        angles_deg = [round(math.degrees(a), 2) for a in q_sol]
        self._robot.send_angles(angles_deg, self.rcfg.speed)

        # Attente partielle (non bloquante sur toute la durée)
        time.sleep(self.rcfg.move_timeout * 0.6)
        self._sync_q()
        self._initialized = True

        if self._q_cur is None:
            return 180.0
        return pointing_error_deg(self._q_cur, direction)

    def _do_jacobian(self, target: np.ndarray) -> tuple[float, str]:
        """Tracking rapide Jacobienne + fallback IK allégée."""
        if self._q_cur is None:
            return self._do_ik_full(target), "aim_at_fallback"

        direction = unit_direction(target, self.rcfg.home_pos)
        if direction is None:
            return 0.0, "skip_close"

        _, z_cur   = fk(self._q_cur)
        q_new, err = jacobian_step(
            self._q_cur, z_cur, direction,
            self.tcfg.jacobian_step_size,
            self.tcfg.jacobian_eps,
        )
        method = "jacobian"

        # Fallback si la Jacobienne diverge
        if err > self.tcfg.jacobian_diverge_deg:
            log.debug("Jacobienne diverge (%.2f°) → IK allégée.", err)
            q_l, el = _ik_light(direction, self._q_cur, self.tcfg)
            if q_l is not None and el < err:
                q_new, err, method = q_l, el, "ik_light"
            else:
                method = "jacobian_degraded"

        angles_deg = [round(math.degrees(a), 2) for a in q_new]
        self._robot.send_angles(angles_deg, self.tcfg.tracking_speed)
        self._q_cur = q_new
        return err, method

    def _sync_q(self) -> None:
        """Relit les angles réels depuis le robot pour le warmstart suivant."""
        try:
            angles = self._robot.get_angles()
            if angles and len(angles) == 6:
                self._q_cur = np.radians(angles)
        except Exception as e:
            log.debug("Sync angles : %s", e)


# ===========================================================================
# Point d'entrée autonome (simulation / robot réel)
# ===========================================================================

if __name__ == "__main__":
    import sys

    # ── ADAPTE ICI ──────────────────────────────────────────────────────────
    #
    # T_source2robot : transformation homogène 4×4
    #   repère SOURCE (caméra) → repère BASE du robot
    #
    # Situation physique :
    #   • Caméra fixe, vue de dessus, à 460 mm au-dessus de la table
    #   • Le robot est posé sur la table
    #   • La caméra est approximativement au-dessus de la zone de travail
    #
    # Repère CAMÉRA (retourné par detecterObjet) :
    #   X_cam : vers la droite de l'image
    #   Y_cam : vers le bas de l'image
    #   Z_cam : vers le bas (= 460 mm, distance caméra→table) — fixe
    #
    # Repère ROBOT (base myCobot280) :
    #   X_rob : vers l'avant du robot
    #   Y_rob : vers la gauche du robot
    #   Z_rob : vers le haut
    #
    # Pour une caméra au-dessus regardant vers le bas :
    #   X_cam  →  X_rob  (ou -X_rob selon orientation physique)
    #   Y_cam  → -Y_rob  (image Y vers le bas, robot Y vers le haut)
    #   Z_cam  → -Z_rob  (caméra Z vers le bas, robot Z vers le haut)
    #
    # La translation [tx, ty, tz] est le décalage entre l'axe optique
    # de la caméra et la base du robot, mesuré dans le repère robot (mm).
    #
    # ⚠️  VALEURS À MESURER SUR TA VRAIE INSTALLATION :
    #   tx = décalage X entre axe caméra et base robot (mm)
    #   ty = décalage Y entre axe caméra et base robot (mm)
    #   tz = 0  (la table est le plan Z=0 du robot, caméra au-dessus)
    #
    # Exemple ci-dessous : caméra centrée au-dessus du robot,
    # légèrement décalée de 50 mm en avant (X) et 0 mm en Y.
    # Remplace tx/ty par tes mesures réelles.
    #
    tx = 50.0   # ← MESURE : décalage X axe-optique → base robot (mm)
    ty = 0.0    # ← MESURE : décalage Y axe-optique → base robot (mm)
    tz = 0.0    # table = plan Z=0 du robot

    T_source2robot = np.array([
        [ 1,  0,  0,  tx],   # X_cam → X_rob
        [ 0, -1,  0,  ty],   # Y_cam → -Y_rob (image inversée vs robot)
        [ 0,  0, -1,  tz],   # Z_cam → -Z_rob (bas vs haut)
        [ 0,  0,  0,   1],
    ], dtype=float)
    #
    # ── FIN ADAPTE ICI ──────────────────────────────────────────────────────

    robot_cfg = RobotConfig(
        port="/dev/ttyTHS1",
        baudrate=1000000,
        home_x=0.0, home_y=0.0, home_z=250.0,
        speed=20,
        move_timeout=3.0,
        n_seeds=12,
        T_source2robot=T_source2robot,
    )

    pred_mode = PredictionMode.KALMAN if "--kalman" in sys.argv else PredictionMode.SIMPLE

    tracker_cfg = TrackerConfig(
        move_threshold_mm    = 10.0,
        jacobian_diverge_deg = 3.0,
        tracking_speed       = 50,
        loop_hz              = 25.0,
        prediction_mode      = pred_mode,
        kalman_process_noise = 0.1,
        kalman_measure_noise = 5.0,
        kalman_latency_s     = 0.05,
    )

    # Objet simulé : cercle horizontal + oscillation Z
    _t0 = time.time()
    """
    def detecterObjet() -> Optional[tuple[float, float, float]]:
        Remplace par ta vraie fonction YOLO / RealSense / etc.
        t = time.time() - _t0
        return (
            80.0 * math.cos(0.5 * t),
            80.0 * math.sin(0.5 * t),
            400.0 + 20.0 * math.sin(t),
        )

    """
    # ────────────────────────────────────────────────────────────────────────

    # ── Mode simulation ──────────────────────────────────────────────────────
    if "--sim" in sys.argv:
        print(f"\n=== SIMULATION laser_tracker — mode : {pred_mode.name} ===\n")
        kalman = KalmanFilter3D(tracker_cfg)
        last_t = time.time()

        hdr = f"{'#':>4}  {'Brut (mm)':^28}  {'Filtré (mm)':^28}  {'Prédit (mm)':^28}  {'dt ms':>6}"
        print(hdr); print("─" * len(hdr))

        for i in range(60):
            time.sleep(1.0 / 25)
            raw   = detecterObjet()
            now   = time.time(); dt = now - last_t; last_t = now
            raw_r = transform_point(T_source2robot, np.array([raw[0], raw[1], raw[2]]))

            if pred_mode == PredictionMode.KALMAN:
                filt = kalman.update(raw_r, dt)
                pred = kalman.predict_ahead(tracker_cfg.kalman_latency_s)
            else:
                filt = pred = raw_r

            def fmt(v): return f"({v[0]:6.1f},{v[1]:6.1f},{v[2]:6.1f})"
            print(f"{i+1:>4}  {fmt(raw_r)}  {fmt(filt)}  {fmt(pred)}  {dt*1000:6.1f}")

        print("\nSimulation terminée.")

    # ── Mode réel ────────────────────────────────────────────────────────────
    else:
        print(f"\n=== TRACKING RÉEL — {pred_mode.name} ===  Ctrl+C pour arrêter\n")
        tracker = LaserTracker(robot_cfg, tracker_cfg, detecterObjet)
        try:
            tracker.start()
            while True:
                time.sleep(5.0)
                print(tracker.stats.summary())
        except KeyboardInterrupt:
            print("\nArrêt…")
        finally:
            tracker.stop()