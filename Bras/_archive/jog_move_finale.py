"""
=========================================================
ROBOT MYCOBOT 280 - CINÉMATIQUE INVERSE + ORIENTATION
=========================================================

Objectif :
---------
Ce programme permet de contrôler un bras robotique MyCobot 280
en utilisant des coordonnées cartésiennes (X, Y, Z).

Le robot calcule automatiquement :
- la position des articulations (J1, J2, J3)
- l'orientation de l'outil (J4, J5, J6)

Auteur : projet apprentissage robotique
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
"""
On initialise la connexion avec le robot via le port série.
Ensuite on active les moteurs.
"""

mc = MyCobot280("/dev/ttyTHS1", 1000000)

mc.power_on()
time.sleep(2)
print("-----------------------------------------")
print("Initialisation du robot...")
print("Connexion série établie")
print("Alimentation des moteurs activée")
print("-----------------------------------------")

print("Robot prêt")
print("Le robot est maintenant prêt à recevoir des commandes.")
print("Angles actuels :", mc.get_angles())


# =========================================================
# 2. PARAMÈTRES PHYSIQUES DU ROBOT
# =========================================================
"""
Ces valeurs représentent la géométrie simplifiée du bras :
- L1 : longueur du bras supérieur
- L2 : longueur de l'avant-bras
"""

L1 = 135.0 # mm
L2 = 160.0 # mm

# vitesse des mouvements
SPEED = 20


# =========================================================
# 3. FONCTION DE SÉCURITÉ (CLAMP)
# =========================================================
def clamp(value, min_value, max_value):
    """
    Empêche une valeur de dépasser les limites autorisées.

    Pourquoi c'est important ?
    --------------------------
    Les moteurs du robot ont des limites mécaniques.
    Si on dépasse ces limites → erreur ou blocage.

    Exemple :
    clamp(200, -150, 150) → 150
    """
    return max(min_value, min(max_value, value))


# =========================================================
# 4. CINÉMATIQUE INVERSE (POSITION X, Y, Z → J1 J2 J3)
# =========================================================
def ik_position(x, y, z):
    """
    Transforme une position 3D (X, Y, Z)
    en angles articulaires du robot.

    Sortie :
    -------
    [J1, J2, J3]
    """

    # -----------------------------
    # J1 : rotation de la base
    # -----------------------------
    # orientation horizontale vers la cible
    j1 = math.degrees(math.atan2(y, x))
    j1 = clamp(j1, -168, 168)

    # distance horizontale au point
    r = math.sqrt(x**2 + y**2)

    # distance totale au point
    d = math.sqrt(r**2 + z**2)

    # vérification de portée
    if d > (L1 + L2):
        print("❌ Position hors portée du robot")
        return None

    # angle direction cible
    angle = math.degrees(math.atan2(z, r))

    # -----------------------------
    # J3 : angle du coude
    # -----------------------------
    cos_elbow = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_elbow = clamp(cos_elbow, -1, 1)

    j3 = math.degrees(math.acos(cos_elbow)) - 180

    # -----------------------------
    # J2 : angle de l'épaule
    # -----------------------------
    cos_shoulder = (L1**2 + d**2 - L2**2) / (2 * L1 * d)
    cos_shoulder = clamp(cos_shoulder, -1, 1)

    j2 = angle - math.degrees(math.acos(cos_shoulder))

    return j1, j2, j3


# =========================================================
# 5. ORIENTATION DE L’OUTIL (J4 J5 J6)
# =========================================================
def ik_orientation_3d(x, y, z):
    """
    Définit l'orientation de la pince vers la cible.

    IMPORTANT :
    ----------
    Ici on simplifie volontairement :
    - J4 et J6 sont fixes pour éviter instabilité mécanique
    - J5 suit l'inclinaison vers la cible
    """

    # distance horizontale XY
    r_xy = math.sqrt(x**2 + y**2)

    # -----------------------------
    # J4 : rotation outil (fixe)
    # -----------------------------
    j4 = 0

    # -----------------------------
    # J5 : inclinaison vers cible
    # -----------------------------
    j5 = math.degrees(math.atan2(z, r_xy))

    # -----------------------------
    # J6 : rotation finale outil (fixe)
    # -----------------------------
    j6 = 0

    return j4, j5, j6


# =========================================================
# 6. BOUCLE PRINCIPALE UTILISATEUR
# =========================================================
"""
L'utilisateur entre une position X Y Z.
Le robot calcule automatiquement les angles
et effectue le mouvement.
"""

while True:

    # -----------------------------
    # arrêt programme
    # -----------------------------
    print("\n=========================================")
    print("Nouvelle demande de déplacement")
    print("=========================================")
    cmd = input("\nEntrée pour bouger / q pour quitter : ")
    if cmd.lower() == "q":
        break

    # -----------------------------
    # saisie utilisateur
    # -----------------------------
    print("\nVeuillez saisir la position souhaitée du robot.")
    print("Les coordonnées sont exprimées en millimètres.")
    print("Contraintes de sécurité :")
    print("X : entre -250 mm et 250 mm")
    print("Y : entre -190 mm et 190 mm")
    print("Z : entre -50 mm et 250 mm")

    x = float(input("X (mm) : "))
    y = float(input("Y (mm) : "))
    z = float(input("Z (mm) : "))

    # -----------------------------
    # vérification des coordonnées
    # -----------------------------
    if x < -250 or x > 250:
        print(" Erreur : X doit être compris entre -250 mm et 250 mm.")
        continue

    if y < -190 or y > 190:
        print(" Erreur : Y doit être compris entre -190 mm et 190 mm.")
        continue

    if z < -50 or z > 250:
        print(" Erreur : Z doit être compris entre -50 mm et 250 mm.")
        continue

    print("-----------------------------------------")
    print("Coordonnées reçues :")
    print("X =", x, "mm")
    print("Y =", y, "mm")
    print("Z =", z, "mm")
    print("-----------------------------------------")

    print("\nCalcul IK en cours...")

    # -----------------------------
    # calcul position (J1 J2 J3)
    # -----------------------------
    print("Calcul des angles des articulations J1, J2 et J3...")
    pos = ik_position(x, y, z)

    if pos is None:
        continue

    j1, j2, j3 = pos

    print("Calcul terminé.")
    print("J1 =", round(j1, 2), "°")
    print("J2 =", round(j2, 2), "°")
    print("J3 =", round(j3, 2), "°")

    # -----------------------------
    # calcul orientation (J4 J5 J6)
    # -----------------------------
    print("\nCalcul de l'orientation de l'outil...")
    j4, j5, j6 = ik_orientation_3d(x, y, z)

    print("Orientation calculée :")
    print("J4 =", round(j4, 2), "°")
    print("J5 =", round(j5, 2), "°")
    print("J6 =", round(j6, 2), "°")

    # -----------------------------
    # sécurité finale des angles
    # -----------------------------
    angles = [
        clamp(j1, -168, 168),
        clamp(j2, -140, 140),
        clamp(j3, -150, 150),
        clamp(j4, -150, 150),
        clamp(j5, -155, 160),
        clamp(j6, -180, 180)
    ]

    # affichage debug
    print("\nAngles envoyés au robot :", angles)
    print("Envoi des angles au robot...")

    # envoi au robot
    mc.send_angles(angles, SPEED)

    print("Commande envoyée.")
    print("Le robot est en mouvement...")

    # pause pour mouvement
    time.sleep(2)

    # retour état robot
    print("Position réelle :", mc.get_coords())


# =========================================================
# 7. FIN DU PROGRAMME
# =========================================================
print("-----------------------------------------")
print("Extinction du programme...")
print("Merci d'avoir utilisé le robot MyCobot 280.")
print("-----------------------------------------")
print("\nProgramme terminé")
