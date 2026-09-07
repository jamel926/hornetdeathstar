"""
=========================================================
ROBOT MYCOBOT 280 - CINÉMATIQUE INVERSE + ORIENTATION
=========================================================
thomas 

Objectif :
---------
Contrôler un bras MyCobot 280 à partir de coordonnées
cartésiennes X, Y, Z.

Le programme calcule :
- J1 : rotation de la base
- J2 : épaule
- J3 : coude
- J4 : orientation de l'outil
- J5 : inclinaison de l'outil
- J6 : rotation finale de l'outil

ATTENTION :
----------
La cinématique utilisée ici est un modèle simplifié.
Les longueurs L1 et L2 doivent être adaptées/calibrées
pour correspondre précisément au robot utilisé.
"""

# =========================================================
# IMPORTS
# =========================================================

from pymycobot.mycobot280 import MyCobot280
import time
import math


# =========================================================
# 1. INITIALISATION DU ROBOT
# =========================================================

PORT = "/dev/ttyTHS1"
BAUDRATE = 1000000

try:
    mc = MyCobot280(PORT, BAUDRATE)

    mc.power_on()
    time.sleep(2)

    print("-----------------------------------------")
    print("Initialisation du robot...")
    print("Connexion série établie")
    print("Alimentation des moteurs activée")
    print("-----------------------------------------")

    print("Robot prêt")
    print("Angles actuels :", mc.get_angles())

except Exception as e:
    print("❌ Erreur lors de l'initialisation du robot :")
    print(e)
    raise SystemExit


# =========================================================
# 2. PARAMÈTRES PHYSIQUES
# =========================================================

# Modèle géométrique simplifié
L1 = 100.0       # longueur bras supérieur en mm
L2 = 110.0       # longueur avant-bras en mm

# Vitesse des mouvements
SPEED = 50


# =========================================================
# 3. LIMITES CARTÉSIENNES
# =========================================================

X_MIN = -250.0
X_MAX = 250.0

Y_MIN = -190.0
Y_MAX = 190.0

Z_MIN = -50.0
Z_MAX = 250.0


# =========================================================
# 4. LIMITES DES ARTICULATIONS
# =========================================================

J1_MIN = -168.0
J1_MAX = 168.0

J2_MIN = -150.0
J2_MAX = 150.0

J3_MIN = -150.0
J3_MAX = 150.0

J4_MIN = -150.0
J4_MAX = 150.0

J5_MIN = -150.0
J5_MAX = 150.0

J6_MIN = -180.0
J6_MAX = 180.0


# =========================================================
# 5. FONCTION CLAMP
# =========================================================

def clamp(value, min_value, max_value):
    """
    Limite une valeur entre min_value et max_value.
    """
    return max(min_value, min(max_value, value))


# =========================================================
# 6. VALIDATION DES COORDONNÉES
# =========================================================

def check_coordinates(x, y, z):
    """
    Vérifie que les coordonnées demandées sont dans
    les limites cartésiennes définies.
    """

    if not (X_MIN <= x <= X_MAX):
        print(
            f"❌ Erreur : X doit être compris entre "
            f"{X_MIN} et {X_MAX} mm."
        )
        return False

    if not (Y_MIN <= y <= Y_MAX):
        print(
            f"❌ Erreur : Y doit être compris entre "
            f"{Y_MIN} et {Y_MAX} mm."
        )
        return False

    if not (Z_MIN <= z <= Z_MAX):
        print(
            f"❌ Erreur : Z doit être compris entre "
            f"{Z_MIN} et {Z_MAX} mm."
        )
        return False

    return True


# =========================================================
# 7. CINÉMATIQUE INVERSE - POSITION
# =========================================================

def ik_position(x, y, z):
    """
    Calcule J1, J2 et J3 à partir de X, Y, Z.

    Retour :
        (j1, j2, j3)
    ou
        None si la position est impossible.
    """

    # -----------------------------------------------------
    # J1 : rotation de la base
    # -----------------------------------------------------

    j1 = math.degrees(math.atan2(y, x))

    if not (J1_MIN <= j1 <= J1_MAX):
        print("❌ J1 dépasse les limites du robot.")
        return None

    # -----------------------------------------------------
    # Distance horizontale
    # -----------------------------------------------------

    r = math.sqrt(x**2 + y**2)

    # -----------------------------------------------------
    # Distance entre la base et la cible
    # -----------------------------------------------------

    d = math.sqrt(r**2 + z**2)

    # -----------------------------------------------------
    # Vérification de la portée
    # -----------------------------------------------------

    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    if d > max_reach:
        print(
            f"❌ Position hors portée."
            f"\nDistance demandée : {d:.2f} mm"
            f"\nPortée maximale : {max_reach:.2f} mm"
        )
        return None

    if d < min_reach:
        print(
            f"❌ Position trop proche de la base."
            f"\nDistance demandée : {d:.2f} mm"
            f"\nDistance minimale : {min_reach:.2f} mm"
        )
        return None

    # -----------------------------------------------------
    # Éviter une division par zéro
    # -----------------------------------------------------

    if d < 0.001:
        print("❌ Position impossible : distance trop faible.")
        return None

    # -----------------------------------------------------
    # Angle de la cible dans le plan vertical
    # -----------------------------------------------------

    target_angle = math.atan2(z, r)

    # -----------------------------------------------------
    # Calcul de J3
    #
    # Loi des cosinus
    # -----------------------------------------------------

    cos_elbow = (
        L1**2 + L2**2 - d**2
    ) / (2 * L1 * L2)

    # Protection contre les petites erreurs numériques
    cos_elbow = clamp(cos_elbow, -1.0, 1.0)

    elbow_angle = math.degrees(math.acos(cos_elbow))

    j3 = elbow_angle - 180.0

    # -----------------------------------------------------
    # Calcul de J2
    # -----------------------------------------------------

    cos_shoulder = (
        L1**2 + d**2 - L2**2
    ) / (2 * L1 * d)

    cos_shoulder = clamp(cos_shoulder, -1.0, 1.0)

    shoulder_angle = math.acos(cos_shoulder)

    j2 = math.degrees(target_angle - shoulder_angle)

    # -----------------------------------------------------
    # Vérification des limites articulaires
    # -----------------------------------------------------

    if not (J2_MIN <= j2 <= J2_MAX):
        print(
            f"❌ J2 hors limites : {j2:.2f}°"
        )
        return None

    if not (J3_MIN <= j3 <= J3_MAX):
        print(
            f"❌ J3 hors limites : {j3:.2f}°"
        )
        return None

    return j1, j2, j3


# =========================================================
# 8. ORIENTATION DE L'OUTIL
# =========================================================

def ik_orientation_3d(x, y, z):
    """
    Calcule une orientation simplifiée de l'outil.

    Dans cette version :
    - J4 = 0°
    - J5 = orientation vers la cible
    - J6 = 0°

    Il ne s'agit PAS d'une résolution complète
    d'orientation 3D.
    """

    # Distance horizontale
    r_xy = math.sqrt(x**2 + y**2)

    # -----------------------------------------------------
    # J4
    # -----------------------------------------------------

    j4 = 0.0

    # -----------------------------------------------------
    # J5
    # -----------------------------------------------------

    j5 = math.degrees(math.atan2(z, r_xy))

    # -----------------------------------------------------
    # J6
    # -----------------------------------------------------

    j6 = 0.0

    return j4, j5, j6


# =========================================================
# 9. VÉRIFICATION DES ANGLES
# =========================================================

def check_joint_limits(angles):
    """
    Vérifie que tous les angles sont dans les limites
    autorisées.

    Retour :
        True  -> OK
        False -> erreur
    """

    limits = [
        (J1_MIN, J1_MAX),
        (J2_MIN, J2_MAX),
        (J3_MIN, J3_MAX),
        (J4_MIN, J4_MAX),
        (J5_MIN, J5_MAX),
        (J6_MIN, J6_MAX)
    ]

    for i, (angle, (min_angle, max_angle)) in enumerate(
        zip(angles, limits),
        start=1
    ):

        if not (min_angle <= angle <= max_angle):
            print(
                f"❌ J{i} hors limites : "
                f"{angle:.2f}° "
                f"(limites : {min_angle}° à {max_angle}°)"
            )
            return False

    return True


# =========================================================
# 10. SAISIE SÉCURISÉE D'UN NOMBRE
# =========================================================

def read_float(message):
    """
    Demande un nombre à l'utilisateur.
    Évite qu'une mauvaise saisie fasse planter
    le programme.
    """

    while True:

        try:
            return float(input(message))

        except ValueError:
            print("❌ Veuillez entrer un nombre valide.")


# =========================================================
# 11. BOUCLE PRINCIPALE
# =========================================================

try:

    while True:

        print("\n=========================================")
        print("     NOUVELLE DEMANDE DE DÉPLACEMENT")
        print("=========================================")

        cmd = input(
            "\nEntrée pour bouger / q pour quitter : "
        )

        if cmd.lower() == "q":
            break

        # -------------------------------------------------
        # Saisie des coordonnées
        # -------------------------------------------------

        print("\nVeuillez saisir la position souhaitée.")
        print("Coordonnées en millimètres.")

        print(
            f"X : {X_MIN} à {X_MAX} mm"
        )

        print(
            f"Y : {Y_MIN} à {Y_MAX} mm"
        )

        print(
            f"Z : {Z_MIN} à {Z_MAX} mm"
        )

        x = read_float("X (mm) : ")
        y = read_float("Y (mm) : ")
        z = read_float("Z (mm) : ")

        # -------------------------------------------------
        # Validation coordonnées
        # -------------------------------------------------

        if not check_coordinates(x, y, z):
            continue

        print("-----------------------------------------")
        print("Coordonnées reçues :")
        print(f"X = {x:.2f} mm")
        print(f"Y = {y:.2f} mm")
        print(f"Z = {z:.2f} mm")
        print("-----------------------------------------")

        # =================================================
        # CALCUL IK POSITION
        # =================================================

        print("\nCalcul de la cinématique inverse...")

        pos = ik_position(x, y, z)

        if pos is None:
            print("❌ Calcul IK impossible.")
            continue

        j1, j2, j3 = pos

        print("\nAngles de position :")
        print(f"J1 = {j1:.2f}°")
        print(f"J2 = {j2:.2f}°")
        print(f"J3 = {j3:.2f}°")

        # =================================================
        # CALCUL ORIENTATION
        # =================================================

        print("\nCalcul de l'orientation...")

        j4, j5, j6 = ik_orientation_3d(
            x, y, z
        )

        print("\nOrientation :")
        print(f"J4 = {j4:.2f}°")
        print(f"J5 = {j5:.2f}°")
        print(f"J6 = {j6:.2f}°")

        # =================================================
        # CONSTRUCTION DES ANGLES
        # =================================================

        angles = [
            j1,
            j2,
            j3,
            j4,
            j5,
            j6
        ]

        # -------------------------------------------------
        # Vérification AVANT envoi
        # -------------------------------------------------

        if not check_joint_limits(angles):

            print(
                "❌ Mouvement annulé : "
                "un ou plusieurs angles sont invalides."
            )

            continue

        # Arrondi pour éviter des valeurs inutiles
        angles = [
            round(angle, 2)
            for angle in angles
        ]

        # =================================================
        # AFFICHAGE FINAL
        # =================================================

        print("\n=========================================")
        print("ANGLES QUI SERONT ENVOYÉS AU ROBOT")
        print("=========================================")

        for i, angle in enumerate(angles, start=1):
            print(f"J{i} = {angle:.2f}°")

        # =================================================
        # CONFIRMATION DE SÉCURITÉ
        # =================================================

        confirmation = input(
            "\nEnvoyer cette commande au robot ? (o/n) : "
        )

        if confirmation.lower() != "o":
            print("⚠️ Mouvement annulé par l'utilisateur.")
            continue

        # =================================================
        # ENVOI AU ROBOT
        # =================================================

        print("\nEnvoi des angles au robot...")

        try:

            mc.send_angles(
                angles,
                SPEED
            )

            print("✅ Commande envoyée.")
            print("Le robot est en mouvement...")

        except Exception as e:

            print("❌ Erreur pendant l'envoi de la commande :")
            print(e)
            continue

        # =================================================
        # ATTENTE
        # =================================================

        time.sleep(2)

        # =================================================
        # RETOUR D'ÉTAT
        # =================================================

        try:

            current_angles = mc.get_angles()
            current_coords = mc.get_coords()

            print("\n-----------------------------------------")
            print("État du robot :")
            print("Angles actuels :", current_angles)
            print("Coordonnées actuelles :", current_coords)
            print("-----------------------------------------")

        except Exception as e:

            print(
                "⚠️ Impossible de récupérer "
                "l'état actuel du robot :"
            )
            print(e)


# =========================================================
# 12. ARRÊT DU PROGRAMME
# =========================================================

except KeyboardInterrupt:

    print("\n\n⚠️ Programme interrompu par l'utilisateur.")


finally:
    print("-----------------------------------------")
    print("Arrêt du programme...")
    print("-----------------------------------------")

    # Désactivation des moteurs
    try:
        mc.power_off()
        print("Moteurs désactivés.")
    except Exception as e:
        print(
            "⚠️ Impossible de désactiver les moteurs :"
        )
        print(e)

    print("\nProgramme terminé.")
