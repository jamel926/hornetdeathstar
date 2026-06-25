import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot


# ==============================================================
# Configuration du robot
# ==============================================================
PORT = "/dev/ttyTHS1"   # Port série Jetson Nano (adapter si besoin : /dev/ttyUSB0)
BAUD = 1000000

# Hauteur fixe du bout du bras au-dessus de la cible (en mm)
# La caméra est à 30 cm du sol, on veut positionner le haut du bras
# juste au-dessus de l'objet → Z = 30 mm au-dessus du sol
Z_CIBLE = 30

# Vitesse de déplacement (1-100)
VITESSE = 30

# Pose de repos (angles en degrés) — à ajuster selon votre installation
POSE_REPOS = [0, 0, 0, 0, 0, 0]


# ==============================================================
# Initialisation du robot
# ==============================================================
def init_robot():
    mc = MyCobot(PORT, BAUD)
    time.sleep(0.5)
    mc.send_angles(POSE_REPOS, VITESSE)
    time.sleep(2)
    print("Robot initialisé et en position de repos.")
    return mc


# ==============================================================
# Déplacement du robot vers la cible
# ==============================================================
def deplacer_vers_cible(mc, x_mm, y_mm):
    """
    Déplace le bout du bras (end-effector) aux coordonnées (x_mm, y_mm)
    dans le repère de la caméra, à la hauteur Z_CIBLE mm du sol.

    Repère caméra (origin = centre image, Y vers le haut) →
    Repère robot MyCobot 280 (origin = base, X vers l'avant, Y vers la gauche) :
      - x_robot  =  x_mm  (axe avant/arrière dans notre installation)
      - y_robot  =  y_mm  (axe gauche/droite)
      - z_robot  =  Z_CIBLE
    Adapter les signes / offsets selon l'orientation réelle de votre montage.

    L'orientation de l'effecteur est maintenue verticale (RX=0, RY=0, RZ=0).
    """
    x_robot = x_mm
    y_robot = y_mm
    z_robot = Z_CIBLE

    print(f"Déplacement vers → X={x_robot:.1f} mm, Y={y_robot:.1f} mm, Z={z_robot} mm")

    # send_coords attend [x, y, z, rx, ry, rz] en mm et degrés
    mc.send_coords([x_robot, y_robot, z_robot, 0, 0, 0], VITESSE, 1)
    # mode 1 = interpolation linéaire
    time.sleep(3)  # attendre la fin du mouvement

    print("Position atteinte.")


# ==============================================================
# Détection de l'objet par couleur
# ==============================================================
def detecterObjet():
    color = input("Quelle couleur détecter ? (rouge, rose, vert, cyan, bleu, jaune, violet, orange) : ")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Impossible d'ouvrir la caméra")
        exit()

    x_mm, y_mm = None, None

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Conversion BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Plages HSV selon la couleur choisie
        plages = {
            "rouge":  (0,   10),
            "orange": (10,  20),
            "jaune":  (20,  35),
            "vert":   (35,  85),
            "cyan":   (85, 100),
            "bleu":   (100, 130),
            "violet": (130, 155),
            "rose":   (155, 170),
        }
        if color not in plages:
            print(f"Couleur '{color}' non reconnue.")
            break

        Tmin, Tmax = plages[color]
        lower_color = np.array([Tmin, 100, 100])
        upper_color = np.array([Tmax, 255, 255])

        # Masque + contours
        mask = cv2.inRange(hsv, lower_color, upper_color)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)

            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                M = cv2.moments(contour)

                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Conversion pixel → mm
                    # À 30 cm du sol : 1 pixel ≈ 0.34 mm
                    mm_par_pixel = 0.34
                    h_img, w_img = frame.shape[:2]

                    x_centre = cx - w_img // 2       # pixels, origine centre
                    y_centre = h_img // 2 - cy        # pixels, Y vers le haut

                    x_mm = x_centre * mm_par_pixel
                    y_mm = y_centre * mm_par_pixel

                    print(
                        f"Position pixel: ({x_centre}, {y_centre}) | "
                        f"Position réelle: ({x_mm:.1f} mm, {y_mm:.1f} mm) | "
                        f"Taille: {w}x{h} px"
                    )

                    # Affichage
                    cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
                    cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)
                    cv2.putText(
                        frame,
                        f"({x_mm:.0f}, {y_mm:.0f}) mm",
                        (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                        1
                    )

        cv2.imshow("Camera", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        # Appuyer sur ESPACE pour valider la position et envoyer le robot
        if key == ord(' ') and x_mm is not None:
            print("Position validée, envoi du robot…")
            break

    cap.release()
    cv2.destroyAllWindows()
    return x_mm, y_mm


# ==============================================================
# Programme principal
# ==============================================================
def main():
    # 1. Détecter l'objet
    x_mm, y_mm = detecterObjet()

    if x_mm is None or y_mm is None:
        print("Aucun objet détecté. Arrêt.")
        return

    print(f"\nObjet détecté à : X={x_mm:.1f} mm, Y={y_mm:.1f} mm (repère caméra)")

    # 2. Initialiser le robot
    mc = init_robot()

    # 3. Déplacer le robot
    deplacer_vers_cible(mc, x_mm, y_mm)

    # 4. Retour à la position de repos
    input("\nAppuyer sur Entrée pour retourner en position de repos…")
    mc.send_angles(POSE_REPOS, VITESSE)
    time.sleep(2)
    print("Robot en position de repos. Fin du programme.")


if __name__ == "__main__":
    main()