"""
=========================================================
ROBOT MYCOBOT 280 - DEPLACEMENT EN COORDONNEES CARTESIENNES
=========================================================

Objectif :
---------
Amener la tete du bras SUR un point (X, Y, Z) exprime
dans le repere du robot, en s'appuyant sur la cinematique
inverse du firmware (send_coords / solve_inv_kinematics)
plutot que sur un modele geometrique approche.

ATTENTION - ce module ne vise pas, il positionne :
-------------------------------------------------
La fonction allerAuPoint() envoie la tete du bras EXACTEMENT
sur le point demande. Ce n'est PAS ce qu'il faut faire d'une
position de frelon : le bras irait le percuter au lieu de le
viser.

    Aller au point   : la tete se pose sur P.
    Viser le point   : la tete reste a distance de P, et
                       l'axe du laser passe par P.

La vraie visee s'ecrira PAR-DESSUS ce module :
    1. choisir une distance de tir d et une direction u
    2. calculer le point d'emission E = P - d * u
    3. appeler allerAuPoint(mc, E, orientation_de_u)

Ce module reste donc la brique de bas niveau : il valide la
consigne, verifie la faisabilite avant de bouger, et mesure
l'ecart reellement atteint.

Pourquoi passer par le firmware :
--------------------------------
Le robot sait resoudre lui-meme sa cinematique inverse,
en tenant compte des 6 axes, des offsets de poignet et de
la hauteur de la base. Le modele a 2 segments code a la
main dans jog_move_finale.py et test_merlin.py ignore
tout cela, et ses longueurs L1/L2 ne sont pas calibrees.

Deroulement d'une visee :
------------------------
    1. verification du point dans les limites cartesiennes
    2. resolution des angles AVANT de bouger, et controle
       de ces angles contre les limites articulaires
    3. deplacement
    4. relecture de la position atteinte et mesure de
       l'ecart avec la consigne

L'etape 2 est ce qui rend ce module utilisable pour des
premiers tests : on voit ce que le robot compte faire
avant qu'il ne le fasse.

Utilisation comme module :
-------------------------
    from PU_Visee import allerAuPoint
    from PU_PositionInitiale import connecterRobot

    mc = connecterRobot()
    allerAuPoint(mc, [150.0, 0.0, 200.0])

Utilisation en autonome :
------------------------
    python3 PU_Visee.py

ATTENTION :
----------
Ce module ne fait que DEPLACER le bras : il n'allume
jamais le laser. Degager l'espace autour du robot et
laisser le laser desarme pendant les tests.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import math
import time

from PU_PositionInitiale import (
    connecterRobot,
    allerPositionInitiale,
    verifierAngles,
    NOMS_ARTICULATIONS
)


# =========================================================
# 2. PARAMETRES DU MOUVEMENT
# =========================================================

# Vitesse volontairement faible pour les premiers tests.
VITESSE_VISEE = 20        # int, 1 a 100 (impose par pymycobot)

# Duree maximale accordee au bras pour atteindre le point.
TIMEOUT_VISEE = 15        # secondes

"""
Mode de deplacement :
    0 - angulaire : le robot choisit librement sa
        trajectoire. Plus tolerant, c'est le mode des
        premiers tests.
    1 - lineaire  : la tete suit une droite. Plus lisible
        mais echoue plus souvent (singularites).
"""
MODE_ANGULAIRE = 0
MODE_LINEAIRE = 1
MODE_VISEE = MODE_ANGULAIRE

# Ecarts toleres entre la consigne et la position mesuree.
TOLERANCE_POSITION = 5.0      # mm
TOLERANCE_ORIENTATION = 5.0   # degres


# =========================================================
# 3. LIMITES CARTESIENNES
# =========================================================

"""
Limites officielles du MyCobot 280.

Ces valeurs sont celles utilisees par pymycobot lui-meme
(pymycobot/robot_info.py). Un point hors de ces bornes
fait lever une MyCobot280DataException a l'envoi.

ATTENTION : ces bornes decrivent une boite, pas l'espace
de travail reel. Un point peut etre dans la boite et
rester inatteignable par le bras. C'est exactement ce que
detecte calculerAngles().
"""

COORDS_MIN = [-281.45, -281.45, -70.0, -180.0, -180.0, -180.0]
COORDS_MAX = [281.45, 281.45, 412.67, 180.0, 180.0, 180.0]

NOMS_COORDONNEES = ["X", "Y", "Z", "RX", "RY", "RZ"]

# Unite de chaque coordonnee, pour l'affichage
UNITES_COORDONNEES = ["mm", "mm", "mm", "deg", "deg", "deg"]


# =========================================================
# 4. LECTURE DE LA POSITION
# =========================================================

def lireCoords(mc):
    """Retourne les 6 coordonnees courantes de la tete, ou None si la lecture n'aboutit pas."""

    try:
        coords = mc.get_coords()

    except Exception as e:
        print("Erreur de lecture des coordonnees :", e)
        return None

    if not isinstance(coords, list) or len(coords) != 6:
        return None

    return coords


# =========================================================
# 5. VERIFICATION D'UN POINT
# =========================================================

def verifierCoords(coords):
    """Verifie que le point demande tient dans les limites cartesiennes du MyCobot 280."""

    # -----------------------------------------------------
    # Forme de la consigne
    # -----------------------------------------------------

    if not isinstance(coords, (list, tuple)):
        print("ERREUR : les coordonnees doivent etre une liste, recu", type(coords))
        return False

    if len(coords) != 6:
        print("ERREUR : il faut exactement 6 coordonnees (X Y Z RX RY RZ), recu", len(coords))
        return False

    # -----------------------------------------------------
    # Valeur de chaque coordonnee
    # -----------------------------------------------------

    for i in range(6):

        valeur = coords[i]
        nom = NOMS_COORDONNEES[i]

        if isinstance(valeur, bool) or not isinstance(valeur, (int, float)):
            print("ERREUR :", nom, "doit etre un nombre, recu", repr(valeur))
            return False

        if valeur < COORDS_MIN[i] or valeur > COORDS_MAX[i]:
            print("ERREUR DE LIMITE")
            print(nom, "=", valeur, UNITES_COORDONNEES[i])
            print("Limite autorisee :", COORDS_MIN[i], "a", COORDS_MAX[i], UNITES_COORDONNEES[i])
            return False

    return True


# =========================================================
# 6. RESOLUTION AVANT MOUVEMENT
# =========================================================

def calculerAngles(mc, coords):
    """Demande au firmware les angles correspondant au point vise, sans bouger le bras."""

    """
    C'est le controle de faisabilite : si le firmware ne
    sait pas resoudre le point, ou si la solution sort des
    limites articulaires, on le sait AVANT d'envoyer le
    moindre ordre de deplacement.
    """

    try:
        angles_actuels = mc.get_angles()

        if not isinstance(angles_actuels, list) or len(angles_actuels) != 6:
            print("Position articulaire courante illisible, resolution impossible.")
            return None

        angles = mc.solve_inv_kinematics(coords, angles_actuels)

    except Exception as e:
        print("Erreur pendant la resolution de la cinematique inverse :", e)
        return None

    # -----------------------------------------------------
    # Reponse exploitable ?
    # -----------------------------------------------------

    if not isinstance(angles, list) or len(angles) != 6:
        print("Le robot n'a pas su resoudre ce point (reponse :", angles, ").")
        print("Le point est probablement hors de l'espace de travail.")
        return None

    # -----------------------------------------------------
    # Solution compatible avec les limites articulaires ?
    # -----------------------------------------------------

    if not verifierAngles(angles):
        print("La solution trouvee sort des limites articulaires du bras.")
        return None

    return angles


# =========================================================
# 7. MESURE DE L'ECART
# =========================================================

def ecartAngulaire(angle_a, angle_b):
    """Retourne l'ecart entre deux angles en degres, ramene dans l'intervalle 0 a 180."""

    """
    Sans ce repliement, une orientation de +179 et une
    orientation de -179 sembleraient distantes de 358
    degres alors qu'elles ne different que de 2.
    """

    ecart = (angle_a - angle_b) % 360.0

    if ecart > 180.0:
        ecart = 360.0 - ecart

    return ecart


def ecartVisee(coords_mesurees, coords_cibles):
    """Retourne l'ecart de position (mm) et l'ecart d'orientation (degres) entre deux poses."""

    # Distance euclidienne sur X, Y, Z
    ecart_position = math.sqrt(
        sum(
            (coords_mesurees[i] - coords_cibles[i]) ** 2
            for i in range(3)
        )
    )

    # Plus grand ecart sur RX, RY, RZ
    ecart_orientation = max(
        ecartAngulaire(coords_mesurees[i], coords_cibles[i])
        for i in range(3, 6)
    )

    return ecart_position, ecart_orientation


# =========================================================
# 8. VISEE
# =========================================================

def allerAuPoint(mc, point, orientation=None, vitesse=VITESSE_VISEE,
          mode=MODE_VISEE, timeout=TIMEOUT_VISEE):
    """Amene la tete du bras sur le point demande et retourne True si la position est atteinte."""

    """
    Rappel : cette fonction POSITIONNE la tete sur le point,
    elle ne vise pas une cible situee a distance. Voir
    l'avertissement en tete de fichier.
    """

    # -----------------------------------------------------
    # Construction de la consigne complete
    # -----------------------------------------------------

    """
    Par defaut on conserve l'orientation courante de
    l'outil : pour un premier test, faire varier la
    position ET l'orientation en meme temps rend toute
    erreur impossible a attribuer.
    """

    if not isinstance(point, (list, tuple)) or len(point) not in (3, 6):
        print("ERREUR : le point doit etre [X, Y, Z] ou [X, Y, Z, RX, RY, RZ].")
        return False

    if len(point) == 6:
        coords = list(point)

    else:

        if orientation is None:
            coords_actuelles = lireCoords(mc)

            if coords_actuelles is None:
                print("Orientation courante illisible, visee annulee.")
                return False

            orientation = coords_actuelles[3:6]

        if len(orientation) != 3:
            print("ERREUR : l'orientation doit contenir 3 valeurs (RX, RY, RZ).")
            return False

        coords = list(point) + list(orientation)

    # -----------------------------------------------------
    # Verification du point
    # -----------------------------------------------------

    if not verifierCoords(coords):
        print("Visee annulee.")
        return False

    coords = [float(valeur) for valeur in coords]

    print("Point vise :")
    for i in range(6):
        print("  ", NOMS_COORDONNEES[i], "=", round(coords[i], 2), UNITES_COORDONNEES[i])

    # -----------------------------------------------------
    # Faisabilite, avant tout mouvement
    # -----------------------------------------------------

    angles = calculerAngles(mc, coords)

    if angles is None:
        print("Visee annulee : point inatteignable.")
        return False

    print("Angles calcules par le robot :")
    for i in range(6):
        print("  ", NOMS_ARTICULATIONS[i], "=", round(angles[i], 2), "deg")

    # -----------------------------------------------------
    # Deplacement
    # -----------------------------------------------------

    try:

        """
        On ne superpose pas une visee a un mouvement deja
        lance : la trajectoire resultante serait
        imprevisible.
        """

        if mc.is_moving() == 1:
            print("Un mouvement est en cours, arret avant la visee.")
            mc.stop()
            time.sleep(0.5)

        print("Deplacement en cours (vitesse", int(vitesse), ", mode", mode, ")...")

        # La vitesse doit imperativement etre un entier
        mc.sync_send_coords(coords, int(vitesse), mode, timeout)

    except Exception as e:
        print("Erreur pendant le deplacement :", e)
        return False

    # -----------------------------------------------------
    # Verification de l'arrivee
    # -----------------------------------------------------

    """
    sync_send_coords retourne 1 meme lorsque le timeout
    expire sans que le point soit atteint. La seule preuve
    fiable est de relire la position mesuree.
    """

    coords_mesurees = lireCoords(mc)

    if coords_mesurees is None:
        print("Position atteinte inconnue : le robot ne repond pas.")
        return False

    ecart_position, ecart_orientation = ecartVisee(coords_mesurees, coords)

    print("Position mesuree :", [round(valeur, 2) for valeur in coords_mesurees])
    print("Ecart de position    :", round(ecart_position, 2), "mm")
    print("Ecart d'orientation  :", round(ecart_orientation, 2), "deg")

    if ecart_position > TOLERANCE_POSITION:
        print("Point NON atteint (tolerance :", TOLERANCE_POSITION, "mm).")
        return False

    if ecart_orientation > TOLERANCE_ORIENTATION:
        print("Orientation NON atteinte (tolerance :", TOLERANCE_ORIENTATION, "deg).")
        return False

    print("Point atteint.")
    return True


# =========================================================
# 9. SAISIE UTILISATEUR
# =========================================================

def lireNombre(message):
    """Demande un nombre a l'operateur jusqu'a ce que la saisie soit valide."""

    while True:

        try:
            return float(input(message))

        except ValueError:
            print("Veuillez entrer un nombre valide.")


# =========================================================
# 10. EXECUTION AUTONOME
# =========================================================

def main():
    """Boucle de test : l'operateur saisit un point, le robot le vise, l'ecart est mesure."""

    print()
    print("=========================================")
    print("      VISEE CARTESIENNE - TESTS")
    print("=========================================")
    print()
    print("ATTENTION : degager l'espace autour du bras")
    print("et verifier que le laser est desarme.")
    print()

    # -----------------------------------------------------
    # Connexion
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    # -----------------------------------------------------
    # Depart depuis un repere connu
    # -----------------------------------------------------

    """
    Une mesure d'ecart n'a de sens que si le bras part
    toujours du meme endroit.
    """

    print()
    depart = input("Passer par la position initiale avant les tests ? (o/n) : ")

    if depart.lower() == "o":

        if not allerPositionInitiale(mc):
            print()
            print("Position initiale non atteinte, tests interrompus.")
            return 1

    print()
    print("Position courante :", lireCoords(mc))

    # -----------------------------------------------------
    # Boucle de test
    # -----------------------------------------------------

    while True:

        print()
        print("=========================================")
        print("           NOUVELLE VISEE")
        print("=========================================")
        print()
        print("Entree pour deplacer la tete sur un point / q pour quitter.")

        commande = input("> ")

        if commande.lower() == "q":
            break

        # -------------------------------------------------
        # Saisie du point
        # -------------------------------------------------

        print()
        print("Coordonnees dans le repere du robot, en mm.")
        print("X :", COORDS_MIN[0], "a", COORDS_MAX[0])
        print("Y :", COORDS_MIN[1], "a", COORDS_MAX[1])
        print("Z :", COORDS_MIN[2], "a", COORDS_MAX[2])
        print()
        print("Rappel : ces bornes decrivent une boite,")
        print("tous les points ne sont pas atteignables.")
        print()

        x = lireNombre("X (mm) : ")
        y = lireNombre("Y (mm) : ")
        z = lireNombre("Z (mm) : ")

        # -------------------------------------------------
        # Visee
        # -------------------------------------------------

        print()
        print("-----------------------------------------")

        """
        L'orientation de l'outil est laissee telle quelle :
        on ne fait varier que la position tant que les
        premiers tests ne sont pas concluants.
        """

        succes = allerAuPoint(mc, [x, y, z])

        print("-----------------------------------------")

        if succes:
            print("Visee reussie.")
        else:
            print("Visee echouee.")

    # -----------------------------------------------------
    # Fin des tests
    # -----------------------------------------------------

    """
    Les moteurs restent volontairement alimentes : un
    power_off relacherait les servos et le bras
    retomberait sous son propre poids.
    """

    print()
    print("Fin des tests. Les moteurs restent alimentes.")

    return 0


if __name__ == "__main__":

    try:
        code_retour = main()

    except KeyboardInterrupt:
        print()
        print("Programme interrompu par l'operateur.")
        code_retour = 1

    print()
    print("Programme termine.")

    raise SystemExit(code_retour)
