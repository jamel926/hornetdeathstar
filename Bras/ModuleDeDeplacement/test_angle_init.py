"""
=========================================================
ROBOT MYCOBOT 280 - COMMANDE DIRECTE J1 À J6
=========================================================

Objectif :
---------
L'utilisateur commande directement les 6 articulations
du MyCobot 280 en entrant :

    J1, J2, J3, J4, J5, J6

Les valeurs sont exprimées en degrés.

Exemple :
    J1 = 2
    J2 = 48
    J3 = 144
    J4 = 75
    J5 = 1
    J6 = 1

Le robot reçoit directement :

    [2.0, 48.0, 144.0, 75.0, 1.0, 1.0]


=========================================================
IMPORTANT
=========================================================

Pour les premiers tests, la vitesse est volontairement
faible.

Commencer avec de petits déplacements par rapport à la
position actuelle du robot.

=========================================================
"""


# =========================================================
# 1. IMPORTS
# =========================================================

from pymycobot.mycobot280 import MyCobot280
import time


# =========================================================
# 2. PARAMÈTRES
# =========================================================

PORT = "/dev/ttyTHS1"
BAUDRATE = 1000000

# Vitesse volontairement faible pour les tests
SPEED = 20

# Temps d'attente après une commande
WAIT_TIME = 5


# =========================================================
# 3. LIMITES DES ARTICULATIONS
# =========================================================

"""
Limites utilisées dans le programme.

Attention :
ces limites servent uniquement à empêcher l'utilisateur
d'entrer des valeurs manifestement trop grandes.

Elles ne remplacent pas les limites mécaniques réelles
du robot.
"""

LIMITES = {
    "J1": (-168, 168),
    "J2": (-150, 150),
    "J3": (-150, 150),
    "J4": (-150, 150),
    "J5": (-150, 150),
    "J6": (-180, 180)
}


# =========================================================
# 4. FONCTION DE VÉRIFICATION
# =========================================================

def verifier_angles(angles):
    """
    Vérifie les 6 angles avant de les envoyer au robot.

    Retourne :
        True  -> angles valides
        False -> au moins un angle invalide
    """

    noms = ["J1", "J2", "J3", "J4", "J5", "J6"]

    for i in range(6):

        nom = noms[i]
        angle = angles[i]

        minimum, maximum = LIMITES[nom]

        if angle < minimum or angle > maximum:

            print()
            print("❌ ERREUR DE LIMITE")
            print(
                f"{nom} = {angle}°"
            )
            print(
                f"Limite autorisée : {minimum}° à {maximum}°"
            )

            return False

    return True


# =========================================================
# 5. INITIALISATION DU ROBOT
# =========================================================

print()
print("=========================================")
print("     INITIALISATION DU MYCOBOT 280")
print("=========================================")

print()
print("Connexion au robot...")
print("Port :", PORT)
print("Baudrate :", BAUDRATE)

try:

    mc = MyCobot280(PORT, BAUDRATE)

except Exception as e:

    print()
    print("❌ Impossible de se connecter au robot.")
    print("Erreur :", e)

    exit()


print("✅ Connexion série établie.")


# =========================================================
# 6. ACTIVATION DES MOTEURS
# =========================================================

print()
print("Activation des moteurs...")

try:

    mc.power_on()

except Exception as e:

    print("❌ Erreur lors de l'activation des moteurs.")
    print("Erreur :", e)
    exit()


time.sleep(2)

print("✅ Moteurs activés.")


# =========================================================
# 7. LECTURE DE LA POSITION ACTUELLE
# =========================================================

print()
print("-----------------------------------------")
print("Position actuelle du robot")
print("-----------------------------------------")

try:

    angles_actuels = mc.get_angles()

    print("Angles actuels :", angles_actuels)

except Exception as e:

    print("❌ Impossible de lire les angles.")
    print("Erreur :", e)

    angles_actuels = None


try:

    coords_actuelles = mc.get_coords()

    print("Coordonnées actuelles :", coords_actuelles)

except Exception as e:

    print("❌ Impossible de lire les coordonnées.")
    print("Erreur :", e)


print()
print("=========================================")
print("           ROBOT PRÊT")
print("=========================================")


# =========================================================
# 8. BOUCLE PRINCIPALE
# =========================================================

while True:

    print()
    print()
    print("=========================================")
    print("       NOUVELLE COMMANDE")
    print("=========================================")

    print()
    print("Appuyez sur Entrée pour entrer les angles.")
    print("Tapez q puis Entrée pour quitter.")

    commande = input("> ")

    # -----------------------------------------------------
    # Quitter
    # -----------------------------------------------------

    if commande.lower() == "q":

        break


    # =====================================================
    # SAISIE DES ANGLES
    # =====================================================

    print()
    print("-----------------------------------------")
    print("Entrez les angles en degrés")
    print("-----------------------------------------")

    print()
    print("Limites utilisées :")
    print("J1 : -168° à +168°")
    print("J2 : -150° à +150°")
    print("J3 : -150° à +150°")
    print("J4 : -150° à +150°")
    print("J5 : -150° à +150°")
    print("J6 : -180° à +180°")

    print()


    # -----------------------------------------------------
    # Lecture utilisateur
    # -----------------------------------------------------

    try:

        j1 = float(input("J1 (°) : "))
        j2 = float(input("J2 (°) : "))
        j3 = float(input("J3 (°) : "))
        j4 = float(input("J4 (°) : "))
        j5 = float(input("J5 (°) : "))
        j6 = float(input("J6 (°) : "))

    except ValueError:

        print()
        print("❌ ERREUR")
        print("Veuillez entrer uniquement des nombres.")

        continue


    # =====================================================
    # CRÉATION DE LA LISTE
    # =====================================================

    angles = [
        j1,
        j2,
        j3,
        j4,
        j5,
        j6
    ]


    # =====================================================
    # AFFICHAGE
    # =====================================================

    print()
    print("-----------------------------------------")
    print("Angles demandés")
    print("-----------------------------------------")

    print(f"J1 = {j1}°")
    print(f"J2 = {j2}°")
    print(f"J3 = {j3}°")
    print(f"J4 = {j4}°")
    print(f"J5 = {j5}°")
    print(f"J6 = {j6}°")

    print()
    print("Liste envoyée au robot :")
    print(angles)


    # =====================================================
    # VÉRIFICATION
    # =====================================================

    print()
    print("Vérification des angles...")

    if not verifier_angles(angles):

        print()
        print("❌ Déplacement annulé.")

        continue

    print("✅ Angles valides.")


    # =====================================================
    # CONFIRMATION
    # =====================================================

    print()
    print("-----------------------------------------")
    print("ATTENTION")
    print("-----------------------------------------")

    print("Le robot va recevoir :")
    print()
    print(angles)
    print()
    print("Vitesse :", SPEED)

    confirmation = input(
        "Confirmer le déplacement ? (o/n) : "
    )


    if confirmation.lower() != "o":

        print()
        print("Déplacement annulé par l'utilisateur.")

        continue


    # =====================================================
    # ENVOI AU ROBOT
    # =====================================================

    print()
    print("-----------------------------------------")
    print("ENVOI DE LA COMMANDE")
    print("-----------------------------------------")

    print("Envoi des angles...")

    try:

        mc.send_angles(angles, SPEED)

        print("✅ Commande envoyée au robot.")

    except Exception as e:

        print()
        print("❌ ERREUR pendant l'envoi de la commande.")
        print("Erreur :", e)

        continue


    # =====================================================
    # ATTENTE DU MOUVEMENT
    # =====================================================

    print()
    print("Le robot est en mouvement...")
    print(
        f"Attente de {WAIT_TIME} secondes..."
    )

    time.sleep(WAIT_TIME)


    # =====================================================
    # LECTURE DES ANGLES APRÈS MOUVEMENT
    # =====================================================

    print()
    print("-----------------------------------------")
    print("ÉTAT DU ROBOT APRÈS LE MOUVEMENT")
    print("-----------------------------------------")

    try:

        angles_apres = mc.get_angles()

        print()
        print("Angles demandés :")
        print(angles)

        print()
        print("Angles mesurés :")
        print(angles_apres)

    except Exception as e:

        print(
            "❌ Impossible de lire les angles après le mouvement."
        )
        print("Erreur :", e)


    # =====================================================
    # LECTURE DES COORDONNÉES
    # =====================================================

    try:

        coords_apres = mc.get_coords()

        print()
        print("Coordonnées actuelles :")
        print(coords_apres)

    except Exception as e:

        print(
            "❌ Impossible de lire les coordonnées."
        )
        print("Erreur :", e)


# =========================================================
# 9. FIN DU PROGRAMME
# =========================================================

print()
print("=========================================")
print("         ARRÊT DU PROGRAMME")
print("=========================================")

print()
print("Désactivation des moteurs...")

try:

    mc.power_off()

except Exception as e:

    print("Erreur lors de l'arrêt des moteurs :", e)


print()
print("Merci d'avoir utilisé le MyCobot 280.")
print("Programme terminé.")
