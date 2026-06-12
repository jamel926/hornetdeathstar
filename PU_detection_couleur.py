import cv2
import numpy as np

# =============================================================================
# PARAMÈTRES À ADAPTER
# =============================================================================

# Couleur cible
COLOR = "vert"

# Hauteur de la caméra au-dessus du plan de la table (mm)
# MESURE RÉELLE : 460 mm (caméra à 46 cm de la base robot / table)
HAUTEUR_CAMERA_MM = 460.0

# Facteur de conversion pixel → mm à ta distance de travail
# (calibré à 30 cm : 0.34 mm/pixel — à recalibrer si distance différente)
MM_PAR_PIXEL = 0.34

# Index de la caméra (0 = webcam par défaut)
CAMERA_INDEX = 0

# Aire minimale du contour pour valider une détection (pixels²)
AIRE_MIN = 500

# =============================================================================
# Plages HSV par couleur
# =============================================================================
HSV_RANGES = {
    "rouge":  (  0,  10),
    "orange": ( 10,  20),
    "jaune":  ( 20,  35),
    "vert":   ( 35,  85),
    "cyan":   ( 85, 100),
    "bleu":   (100, 130),
    "violet": (130, 155),
    "rose":   (155, 170),
}

# =============================================================================
# État global de la caméra
# — la caméra est ouverte une seule fois et réutilisée à chaque appel.
# =============================================================================
_cap: cv2.VideoCapture | None = None


def _get_cap() -> cv2.VideoCapture:
    """Ouvre la caméra si ce n'est pas déjà fait, retourne l'objet VideoCapture."""
    global _cap
    if _cap is None or not _cap.isOpened():
        _cap = cv2.VideoCapture(CAMERA_INDEX)
        if not _cap.isOpened():
            raise RuntimeError(
                f"Impossible d'ouvrir la caméra (index {CAMERA_INDEX})"
            )
    return _cap


def libererCamera() -> None:
    """Libère la caméra proprement (à appeler en fin de programme)."""
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
    cv2.destroyAllWindows()


# =============================================================================
# Fonction principale — NON BLOQUANTE
# Appelée à chaque frame par laser_tracker.py (~25 Hz).
# Retourne (x_mm, y_mm, z_mm) dans le repère CAMÉRA, ou None si rien détecté.
#
# Repère caméra (vue de dessus) :
#   X : positif vers la droite de l'image
#   Y : positif vers le bas de l'image
#   Z : positif vers le bas (= distance caméra → table)
#       → on retourne HAUTEUR_CAMERA_MM comme valeur fixe
# =============================================================================

def detecterObjet() -> tuple[float, float, float] | None:
    """
    Capture une frame, détecte l'objet coloré, retourne sa position 3D
    dans le repère caméra (mm), ou None si l'objet n'est pas visible.

    Coordonnées retournées :
      x_mm, y_mm : position dans le plan image, centrée sur l'axe optique
      z_mm       : HAUTEUR_CAMERA_MM (distance caméra → plan de la table)

    Cette fonction est non-bloquante : elle retourne None immédiatement
    si aucun objet n'est détecté (ne boucle plus en interne).
    """
    cap = _get_cap()
    ret, frame = cap.read()

    if not ret or frame is None:
        print("[detection] Impossible de lire une frame.")
        return None

    # Conversion BGR → HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Récupération de la plage HSV pour la couleur choisie
    if COLOR not in HSV_RANGES:
        raise ValueError(f"Couleur inconnue : '{COLOR}'. Choix : {list(HSV_RANGES)}")

    tmin, tmax = HSV_RANGES[COLOR]
    lower = np.array([tmin, 100, 100])
    upper = np.array([tmax, 255, 255])

    # Masque couleur
    mask = cv2.inRange(hsv, lower, upper)

    # Recherche des contours
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return None

    # Plus grand contour
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < AIRE_MIN:
        return None

    # Centre par moments (plus précis que le centre du bounding rect)
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    h_img, w_img = frame.shape[:2]

    # Coordonnées en pixels, centrées sur l'axe optique
    x_centre = cx - w_img // 2
    y_centre = cy - h_img // 2

    # Conversion pixel → mm dans le plan de la table
    x_mm = x_centre * MM_PAR_PIXEL
    y_mm = y_centre * MM_PAR_PIXEL

    # Z = hauteur fixe caméra → table (plan de travail)
    z_mm = HAUTEUR_CAMERA_MM

    print(
        f"[detection] pixel=({x_centre:+d}, {y_centre:+d}) | "
        f"mm=({x_mm:+.1f}, {y_mm:+.1f}, {z_mm:.0f})"
    )

    # Visualisation (optionnel — commenter pour gagner du temps CPU)
    x, y, w, h = cv2.boundingRect(contour)
    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
    cv2.imshow("Camera", frame)
    cv2.waitKey(1)  # non-bloquant

    return (x_mm, y_mm, z_mm)