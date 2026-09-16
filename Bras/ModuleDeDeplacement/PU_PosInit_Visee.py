"""
=========================================================
ROBOT MYCOBOT 280 - SEQUENCE COMPLETE
=========================================================

Objectif :
---------
Script unique regroupant :
  - PU_PositionInitiale.py  (connexion, securites, mise en
    position initiale)
  - PU_PositionVisee2.py    (verification des angles de
    visee, mouvement vers la position visee)
  - test_mouvement_laser.py (sequence complete : position
    initiale -> visee -> laser -> retour position initiale)

Deroule :
--------
1. Connexion au robot (une seule fois, une seule liaison
   serie reutilisee partout).
2. Mise en position initiale (avec securites : statut des
   servos + vitesse pendant le mouvement, arret + recul en
   cas de probleme).
3. Saisie des angles J4/J5 voulus pour la visee.
4. Verification/correction de ces angles (zone reglementee).
5. Mouvement vers la position visee (memes securites).
6. Laser allume 5 secondes.
7. Retour en position initiale.

ATTENTION :
----------
Degager l'espace autour du bras et desarmer le laser
avant toute execution.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

from pymycobot.mycobot280 import MyCobot280
import time


# =========================================================
# 2. PARAMETRES DE CONNEXION
# =========================================================

PORT = "/dev/ttyTHS1"
BAUDRATE = 1000000


# =========================================================
# 3. PARAMETRES DU MOUVEMENT
# =========================================================

VITESSE_INIT = 100        # int, 1 a 100 (impose par pymycobot)

# Duree maximale accordee au bras pour atteindre la pose.
TIMEOUT_INIT = 15        # secondes

# Ecart angulaire tolere entre la consigne et la position
# reellement mesuree a l'arrivee.
TOLERANCE_ANGLE = 2.0    # degres

# Broche du laser.
PIN_LASER = 33


# =========================================================
# 4. LIMITES ARTICULAIRES
# =========================================================

"""
Limites officielles du MyCobot 280.

Ces valeurs sont celles utilisees par pymycobot lui-meme
(pymycobot/robot_info.py). Un angle hors de ces bornes
fait lever une MyCobot280DataException a l'envoi.

Attention : J2 et J5 ne sont PAS symetriques.
"""

ANGLES_MIN = [-168.0, -140.0, -150.0, -150.0, -155.0, -180.0]
ANGLES_MAX = [168.0, 140.0, 150.0, 150.0, 160.0, 180.0]

NOMS_ARTICULATIONS = ["J1", "J2", "J3", "J4", "J5", "J6"]


# =========================================================
# 5. POSITIONS DE REFERENCE
# =========================================================

"""
Pose de depart du bras, en degres.

Pour les relever a nouveau : placer le bras a la main
dans la pose voulue (moteurs relaches via
release_all_servos) puis lire mc.get_angles().
"""

ANGLES_POSITION_INITIALE = [
    0.0,      # J1 - rotation de la base
    50.0,     # J2 - epaule
    -145.0,   # J3 - coude
    40.0,     # J4 - poignet
    0.0,      # J5 - inclinaison de l'outil
    0.0       # J6 - rotation finale de l'outil
]

ANGLES_POSITION_VERTICALE = [
    0.0,      # J1 - rotation de la base
    0.0,      # J2 - epaule
    0.0,      # J3 - coude
    0.0,      # J4 - poignet
    0.0,      # J5 - inclinaison de l'outil
    0.0       # J6 - rotation finale de l'outil
]


# =========================================================
# 6. CONNEXION AU ROBOT
# =========================================================

def connecterRobot(port=PORT, baudrate=BAUDRATE):
    """Ouvre la liaison serie, alimente les moteurs et retourne l'objet robot (None si echec)."""

    print("-----------------------------------------")
    print("Connexion au robot...")
    print("Port :", port)
    print("Baudrate :", baudrate)

    # -----------------------------------------------------
    # Ouverture du port serie
    # -----------------------------------------------------

    try:
        mc = MyCobot280(port, baudrate)

    except Exception as e:
        print("Impossible d'ouvrir la liaison serie.")
        print("Erreur :", e)
        return None

    print("Liaison serie etablie.")

    # -----------------------------------------------------
    # Alimentation des moteurs
    # -----------------------------------------------------

    try:
        mc.power_on()

    except Exception as e:
        print("Impossible d'alimenter les moteurs.")
        print("Erreur :", e)
        return None

    # Laisser le temps aux servos de repondre
    time.sleep(2)

    # -----------------------------------------------------
    # Verification de l'alimentation
    # -----------------------------------------------------

    try:
        etat = mc.is_power_on()

    except Exception as e:
        print("Impossible de lire l'etat d'alimentation.")
        print("Erreur :", e)
        return None

    if etat != 1:
        print("Les moteurs ne sont pas alimentes (is_power_on =", etat, ").")
        return None

    print("Moteurs alimentes.")

    # -----------------------------------------------------
    # Verification de la communication
    # -----------------------------------------------------

    """
    Le firmware peut repondre None ou une liste vide tant
    que la carte Atom n'est pas prete. On refuse de rendre
    la main dans ce cas : mieux vaut echouer ici que de
    lancer un mouvement a l'aveugle.
    """

    angles = lireAngles(mc)

    if angles is None:
        print("Le robot ne renvoie pas sa position, communication incomplete.")
        return None

    print("Communication verifiee.")
    print("-----------------------------------------")

    return mc


# =========================================================
# 7. LECTURE DE LA POSITION
# =========================================================

def lireAngles(mc):
    """Retourne les 6 angles courants du robot, ou None si la lecture n'aboutit pas."""

    try:
        angles = mc.get_angles()

    except Exception as e:
        print("Erreur de lecture des angles :", e)
        return None

    if not isinstance(angles, list) or len(angles) != 6:
        return None

    return angles


# =========================================================
# 8. VERIFICATION DES ANGLES
# =========================================================

def verifierAngles(angles):
    """Verifie que les 6 angles demandes respectent les limites du MyCobot 280."""

    # -----------------------------------------------------
    # Forme de la consigne
    # -----------------------------------------------------

    if not isinstance(angles, (list, tuple)):
        print("ERREUR : les angles doivent etre une liste, recu", type(angles))
        return False

    if len(angles) != 6:
        print("ERREUR : il faut exactement 6 angles, recu", len(angles))
        return False

    # -----------------------------------------------------
    # Valeur de chaque articulation
    # -----------------------------------------------------

    """
    On refuse une consigne hors limites, on ne la rabote
    pas. Un angle hors limites dans une position initiale
    est une erreur de configuration : la corriger en
    silence enverrait le bras ailleurs que prevu.
    """

    for i in range(6):

        angle = angles[i]
        nom = NOMS_ARTICULATIONS[i]

        if isinstance(angle, bool) or not isinstance(angle, (int, float)):
            print("ERREUR :", nom, "doit etre un nombre, recu", repr(angle))
            return False

        if angle < ANGLES_MIN[i] or angle > ANGLES_MAX[i]:
            print("ERREUR DE LIMITE")
            print(nom, "=", angle, "degres")
            print("Limite autorisee :", ANGLES_MIN[i], "a", ANGLES_MAX[i], "degres")
            return False

    return True


# =========================================================
# 9. COMPARAISON A UNE POSE
# =========================================================

def ecartMaximal(angles_mesures, angles_cibles):
    """Retourne le plus grand ecart, en degres, entre la pose mesuree et la pose visee."""

    ecarts = [
        abs(angles_mesures[i] - angles_cibles[i])
        for i in range(6)
    ]

    return max(ecarts)


def estEnPositionInitiale(mc, angles=None, tolerance=TOLERANCE_ANGLE):
    """Indique si le bras se trouve deja dans la position initiale, sans le faire bouger."""

    if angles is None:
        angles = ANGLES_POSITION_INITIALE

    angles_mesures = lireAngles(mc)

    if angles_mesures is None:
        return False

    return ecartMaximal(angles_mesures, angles) <= tolerance


# =========================================================
# SECURITE
# =========================================================

def statuts_moteurs(mc):
    ### Recupere la liste de statuts des moteurs : [0,0,0,0,0,0] : 0 si OK, ou valeur si pb
    a = mc.get_servo_status()
    return(a)


def detection_problemes(a):
    ### Test la liste de statuts pour savoir si il y a un probleme
    if a == [0, 0, 0, 0, 0, 0]:
        res = False
        print("Aucun probleme de statuts des moteurs")
    else:
        res = True
        print("Probleme de statuts detecte")
    return(res)


def Statuts(j):
    ### Exploite les valeurs des problemes de statuts pour determiner leur type
    res = ["voltage", "sensor", "temperature", "current", "angle", "overload"]
    return res[j]


def Explanation_problemes(a):
    ### Affichage des problemes de statuts
    for i in range(0, 5):
        if a[i] != 0:
            print("Probleme de statuts : J", i + 1)
            v = a[i][0]
            print(Statuts(v), "probleme")


def Arret_urgence(mc):
    print("======ARRET D'URGENCE======")
    mc.stop()


def blocage_vitesse_moteur(mc, angle_cible):
    speed = mc.get_servo_speeds()
    angle_actuel = lireAngles(mc)
    ecart_minimal = 3
    print(f"vitesse  ={speed}\n angle = {angle_actuel}")
    if speed is None:
        return False
    for i in range(6):
        if i != 2:  # J3 en panne sur le robot 1, on l'ignore
            ecart = abs(angle_actuel[i] - angle_cible[i])
            if ecart > ecart_minimal and abs(speed[i]) < 150:
                print(f"Blocage sur le moteur J{i+1}")
                return True
    return False


def ReculerPositionPrec(mc, angles_prec, vitesse):
    if angles_prec is None:
        print(f"pas de postion précédente connue, le bras reste à l'arrêt")
        return
    print(f"retour à la position précédente, {angles_prec}")
    mc.send_angles(angles_prec, int(vitesse))


# =========================================================
# 10. MISE EN POSITION INITIALE
# =========================================================

def allerPositionInitiale(mc, angles=None, vitesse=VITESSE_INIT, timeout=TIMEOUT_INIT):
    """Amene le bras dans la position initiale et confirme qu'il l'a bien atteinte."""

    if angles is None:
        angles = ANGLES_POSITION_INITIALE

    # -----------------------------------------------------
    # Verification de la consigne
    # -----------------------------------------------------

    if not verifierAngles(angles):
        print("Mise en position initiale annulee.")
        return False

    # Consigne propre pour le robot
    angles = [float(angle) for angle in angles]

    print("Position initiale visee :")
    for i in range(6):
        print("  ", NOMS_ARTICULATIONS[i], "=", angles[i], "degres")

    # -----------------------------------------------------
    # Deja en position ? (evite d'attendre un mouvement qui
    # ne demarrera jamais si le bras y est deja)
    # -----------------------------------------------------

    if estEnPositionInitiale(mc, angles, TOLERANCE_ANGLE):
        print()
        print("Position initiale similaire a la position actuelle, pas de mouvement necessaire.")
        return True

    try:

        # -------------------------------------------------
        # Arret d'un eventuel mouvement en cours
        # -------------------------------------------------

        """
        On ne superpose pas une mise en position a un
        mouvement deja lance : la trajectoire resultante
        serait imprevisible.
        """

        if mc.is_moving() == 1:
            print("Un mouvement est en cours, arret avant initialisation.")
            mc.stop()
            time.sleep(0.5)

        # -------------------------------------------------
        # Envoi de la consigne
        # -------------------------------------------------

        print("Deplacement en cours (vitesse", int(vitesse), ")...")
        print(f"début du mvt")
        # La vitesse doit imperativement etre un entier

        mc.send_angles(angles, int(vitesse))
        debut = time.time()
        # Attendre que le mouvement démarre (QUE IS MOVING PASSE À 1)
        while mc.is_moving() != 1:
            time.sleep(0.05)

        print("Mouvement détecté")
        derniers_angles = lireAngles(mc)

        # Surveillance pendant le mouvement
        while mc.is_moving() == 1:
            a = statuts_moteurs(mc)
            a[2] = 0  # pour ignoer J3 en panne pour le robot 1

            if detection_problemes(a):
                Explanation_problemes(a)
                print("Vous avez merdé les loustics")
                Arret_urgence(mc)
                ReculerPositionPrec(mc, derniers_angles, vitesse)
                return False

            time.sleep(0.1)
            if time.time() - debut > 0.3:
                if blocage_vitesse_moteur(mc, angle_cible=angles):
                    Arret_urgence(mc)
                    ReculerPositionPrec(mc, derniers_angles, vitesse)
                    return False

    except Exception as e:
        print("Erreur pendant le deplacement :", e)
        return False

    # -----------------------------------------------------
    # Verification de l'arrivee
    # -----------------------------------------------------

    angles_mesures = lireAngles(mc)

    if angles_mesures is None:
        print("Position atteinte inconnue : le robot ne repond pas.")
        return False

    ecart = ecartMaximal(angles_mesures, angles)

    print("Angles mesures :", angles_mesures)
    print("Ecart maximal :", round(ecart, 2), "degres")

    if ecart > TOLERANCE_ANGLE:
        print("La position initiale n'est PAS atteinte.")
        print("Tolerance :", TOLERANCE_ANGLE, "degres")
        return False

    print("Position initiale atteinte.")
    return True


# =========================================================
# 11. VERIFICATION DES ANGLES DE VISEE
# =========================================================

def Verification_Angle_SECU(angle):
    J1, J2, J3, J4, J5, J6 = angle
    if J4 > 95.0:
        print("L'angle J4 voulu est hors zone reglementée")
        print("changement de l'angle pour Angl_Max=95°")
        J4 = 95.0
    if J4 < 0.0:
        print("L'angle J4 voulu est hors zone reglementée")
        print("changement de l'angle pour Angl_Min=0°")
        J4 = 0.0
    if J5 < -80.0:
        print("L'angle J5 voulu est hors zone reglementé")
        print("changement de l'angle pour Angl_Max=-80°")
        J5 = -80
    if J5 > 80.0:
        print("L'angle J5 voulu est hors zone reglementé")
        print("changement de l'angle pour Angl_Max=80°")
        J5 = 80.0
    angle = (J1, J2, J3, J4, J5, J6)
    return(angle)


# =========================================================
# 12. MISE EN POSITION VISEE
# =========================================================

def aller_en_position_visee(mc, angles=None, vitesse=VITESSE_INIT, timeout=TIMEOUT_INIT):
    """Amene le bras dans la position visee et confirme qu'il l'a bien atteinte."""

    if angles is None:
        angles = ANGLES_POSITION_INITIALE

    # -----------------------------------------------------
    # Verification de la consigne
    # -----------------------------------------------------

    if not verifierAngles(angles):
        print("Mise en position visee annulee.")
        return False

    # Consigne propre pour le robot
    angles = [float(angle) for angle in angles]

    print("Position visee :")
    for i in range(6):
        print("  ", NOMS_ARTICULATIONS[i], "=", angles[i], "degres")

    # -----------------------------------------------------
    # Deja en position ? (evite d'attendre un mouvement qui
    # ne demarrera jamais si le bras y est deja, par exemple
    # en cas de relance avec les memes valeurs de J4/J5)
    # -----------------------------------------------------

    if estEnPositionInitiale(mc, angles, TOLERANCE_ANGLE):
        print()
        print("Position visee similaire a la position actuelle, pas de mouvement necessaire.")
        return True

    try:

        # -------------------------------------------------
        # Arret d'un eventuel mouvement en cours
        # -------------------------------------------------

        if mc.is_moving() == 1:
            print("Un mouvement est en cours, arret avant initialisation.")
            mc.stop()
            time.sleep(0.5)

        # -------------------------------------------------
        # Envoi de la consigne
        # -------------------------------------------------

        print("Deplacement en cours (vitesse", int(vitesse), ")...")
        print(f"début du mvt")

        mc.send_angles(angles, int(vitesse))
        debut = time.time()
        time.sleep(0.05)
        # Attendre que le mouvement démarre (QUE IS MOVING PASSE À 1)
        while mc.is_moving() != 1:
            time.sleep(0.05)

        print("Mouvement détecté")
        derniers_angles = lireAngles(mc)

        # Surveillance pendant le mouvement
        while mc.is_moving() == 1:
            a = statuts_moteurs(mc)
            a[2] = 0  # pour ignoer J3 en panne pour le robot 1

            if detection_problemes(a):
                Explanation_problemes(a)
                print("Vous avez merdé les loustics")
                Arret_urgence(mc)
                ReculerPositionPrec(mc, derniers_angles, vitesse)
                return False

            time.sleep(0.1)
            if time.time() - debut > 0.3:
                if blocage_vitesse_moteur(mc, angle_cible=angles):
                    Arret_urgence(mc)
                    ReculerPositionPrec(mc, derniers_angles, vitesse)
                    return False

    except Exception as e:
        print("Erreur pendant le deplacement :", e)
        return False

    # -----------------------------------------------------
    # Verification de l'arrivee
    # -----------------------------------------------------

    angles_mesures = lireAngles(mc)

    if angles_mesures is None:
        print("Position atteinte inconnue : le robot ne repond pas.")
        return False

    ecart = ecartMaximal(angles_mesures, angles)

    print("Angles mesures :", angles_mesures)
    print("Ecart maximal :", round(ecart, 2), "degres")

    if ecart > TOLERANCE_ANGLE:
        print("La position visee n'est PAS atteinte.")
        print("Tolerance :", TOLERANCE_ANGLE, "degres")
        return False

    print("Position visee atteinte.")
    return True


# =========================================================
# 13. SEQUENCE DE MISE EN POSITION INITIALE (avec option operateur)
# =========================================================

def Mise_en_Position_Initiale(mc, vitesse=VITESSE_INIT, utilisateur=True):
    """
    Sequence complete de mise en position initiale.

    Reutilise la connexion `mc` deja ouverte (une seule
    liaison serie pour tout le programme) au lieu d'en
    ouvrir une nouvelle a chaque appel.

    Retourne 0 en cas de succes, 1 en cas d'echec.
    """

    print()
    print("Position actuelle :", lireAngles(mc))
    print("Position visee    :", ANGLES_POSITION_INITIALE)

    if estEnPositionInitiale(mc):
        print()
        print("Le bras est deja en position initiale.")
        return 0

    # -----------------------------------------------------
    # Confirmation de l'operateur si utilisateur = True
    # -----------------------------------------------------

    if utilisateur:
        print()
        confirmation = input("Lancer la mise en position initiale ? (o/n) : ")

        if confirmation.lower() != "o":
            print("Mise en position initiale annulee par l'operateur.")
            return 1

    # -----------------------------------------------------
    # Mouvement
    # -----------------------------------------------------

    print()
    print("-----------------------------------------")

    succes = allerPositionInitiale(mc, ANGLES_POSITION_INITIALE, vitesse)

    print("-----------------------------------------")
    print()

    if succes:
        print()
        print("=========================================")
        print("   POSITION DU BRAS INITIALISE ET PRET A L'EMPLOI ")
        print("=========================================")
        print()
        return 0

    print("Echec de la mise en position initiale.")
    print("Les moteurs restent alimentes, le bras n'a pas ete relache.")
    return 1


# =========================================================
# 14. EXECUTION AUTONOME - SEQUENCE COMPLETE
# =========================================================

def main():
    """Position initiale -> saisie J4/J5 -> position visee -> laser -> retour position initiale."""

    # -----------------------------------------------------
    # Connexion (une seule fois pour tout le programme)
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    # -----------------------------------------------------
    # LASER
    # -----------------------------------------------------

    print("Configuration G33 en sortie")
    mc.set_pin_mode(PIN_LASER, 0)
    time.sleep(0.1)
    print("G33 = 0 -> LASER ETEINT")
    time.sleep(1)

    # -----------------------------------------------------
    # Mise en position initiale
    # -----------------------------------------------------

    resultat = Mise_en_Position_Initiale(mc, VITESSE_INIT, True)

    if resultat != 0:
        print()
        print("Impossible de continuer : la mise en position initiale a echoue.")
        return 1

    time.sleep(2)

    # -----------------------------------------------------
    # Saisie de la position visee
    # -----------------------------------------------------

    VAL_J4 = float(input("Valeur de J4 voulue : "))
    VAL_J5 = float(input("Valeur de J5 voulue : "))

    ANGLES_POSITION_VISEES = [
        0.0,      # J1 - rotation de la base
        50.0,     # J2 - epaule
        -145.0,   # J3 - coude
        VAL_J4,   # J4 - poignet
        VAL_J5,   # J5 - inclinaison de l'outil
        0.0       # J6 - rotation finale de l'outil
    ]

    ANGLES = Verification_Angle_SECU(ANGLES_POSITION_VISEES)

    print()
    print("Position actuelle :", lireAngles(mc))
    print("Position visee    :", ANGLES_POSITION_VISEES)

    # -----------------------------------------------------
    # Mouvement vers la position visee
    # -----------------------------------------------------

    print()
    print("-----------------------------------------")
    print(ANGLES)
    succes = aller_en_position_visee(mc, ANGLES, 50)
    print("-----------------------------------------")

    if not succes:
        print()
        print("Position visee non atteinte, le laser ne sera pas allume.")
        Mise_en_Position_Initiale(mc, VITESSE_INIT, False)
        return 1

    # -----------------------------------------------------
    # Laser
    # -----------------------------------------------------

    mc.set_pin_mode(PIN_LASER, 1)
    time.sleep(0.1)
    mc.set_digital_output(PIN_LASER, 0)
    print("G33 = 1 -> LASER ALLUME")
    time.sleep(5)

    print("G33 = 0 -> LASER ETEINT")
    mc.set_digital_output(PIN_LASER, 1)
    time.sleep(1)

    print("Fin du laser, merci bonsoir")

    # -----------------------------------------------------
    # Remise en position initiale
    # -----------------------------------------------------

    Mise_en_Position_Initiale(mc, VITESSE_INIT, False)

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
