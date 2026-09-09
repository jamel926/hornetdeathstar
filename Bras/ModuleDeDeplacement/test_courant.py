"""
gdb
=========================================================
ROBOT MYCOBOT 280 - MISE EN POSITION INITIALE
=========================================================

Objectif :
---------
Amener le bras dans une pose de depart connue, exprimee
directement en angles articulaires J1 a J6 (en degres).

C'est le prealable de toute sequence de visee : tant que
le robot n'est pas dans un repere connu, les coordonnees
calculees par la stereovision ne veulent rien dire.

Utilisation comme module :
-------------------------
    from PU_PositionInitiale import connecteroRobot, allerPositionInitiale

    mc = connecterRobot()
    if mc is not None and allerPositionInitiale(mc):
        print("Bras pret")

Utilisation en autonome :
------------------------
    python3 PU_PositionInitiale.py

ATTENTION :
----------
Degager l'espace autour du bras et desarmer le laser
avant toute execution.

Angles mesures : [2.54, 48.42, -124.36, 74.26, 0.35, 0.0]
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

# Vitesse volontairement faible : la mise en position
# initiale part d'une pose inconnue, le bras peut avoir
# un long chemin a parcourir.
VITESSE_INIT = 20        # int, 1 a 100 (impose par pymycobot)

# Duree maximale accordee au bras pour atteindre la pose.
TIMEOUT_INIT = 15        # secondes

# Ecart angulaire tolere entre la consigne et la position
# reellement mesuree a l'arrivee.
TOLERANCE_ANGLE = 2.0    # degres


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
# 5. POSITION INITIALE
# =========================================================

"""
Pose de depart du bras, en degres.

Valeurs relevees sur le demonstrateur.o

Pour les relever a nouveau : placer le bras a la main
dans la pose voulue (moteurs relaches via
release_all_servos) puis lire mc.get_angles().
"""
'''
ANGLES_POSITION_INITIALE = [
    2.0,      # J1 - rotation de la base
    48.0,     # J2 - epaule
    -144.0,   # J3 - coude
    75.0,     # J4 - poignet
    1.0,      # J5 - inclinaison de l'outil
    1.0       # J6 - rotation finale de l'outil
]'''
ANGLES_POSITION_INITIALE = [
    45.0,      # J1 - rotation de la base
    0.0,     # J2 - epaule
    -136,   # J3 - coude
    51.0,     # J4 - poignet
    1.0,      # J5 - inclinaison de l'outil
    51.0       # J6 - rotation finale de l'outil
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
# 10. Paul
# =========================================================
def GetStatus(mc): 
    a = mc.get_servo_status()
    return a


def Problem(a): 
    if a != [0,0,0,0,0,0]: 
        for i in range(0, 6): 
            if a[i] != [0]:
                print(f"Voltage Problem sur le moteur {i+1}")
            elif a[i] != [1]:
                print(f"Sensor Problem sur le moteur {i+1}")
            elif a[i] != [2]:
                print(f"Temperature Problem sur le moteur {i+1}")
            elif a[i] != [3]:
                print(f"current Problem sur le moteur {i+1}")
            elif a[i] != [4]:
                print(f"angle Problem sur le moteur {i+1}")
            elif a[i] != [5]:
                print(f"angle Problem sur le moteur {i+1}")
        return True

# =========================================================
# SECURITE
# =========================================================


def statuts_moteurs(mc):
   ### Recupère la liste de statuts des moteurs : [0,0,0,0,0,0] : 0 si OK, ou valeur  si pb
   a = mc.get_servo_status()
   return(a)


def detection_problemes(a):
   ### Test la liste de statuts pour savoir si il y a un problème
   if a == [0,0,0,0,0,0]:
       res = False
       print("Aucun problème de statuts des moteurs")
   else :
       res = True
       print("Problème de statuts détecté")
   return(res)


def Statuts(j):
   ### Exploite les valeurs des problèmes de statuts pour déterminer leur type
   res= ["voltage", "sensor", "temperature", "current", "angle", "overload"]
   return res[j]


def Explanation_problemes(a):
   ### Affichage des problèmes de statuts
   for i in range(0,5) :
       if a[i] !=0 :
           print("Problème de statuts : J",i+1)
           v = a[i][0]
           print(Statuts(v),"problème")

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
        if i !=2:  
            ecart = abs(angle_actuel[i]-angle_cible[i])
            if ecart > ecart_minimal and abs(speed[i])<250: 
                print(f"Blocage sur le moteur J{i+1}")
                return True
    return False 



# =========================================================
# 10bis. MISE EN POSITION INITIALE
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


       # Surveillance pendant le mouvement
        while mc.is_moving() == 1:
           print(f"vitesse======{mc.get_servo_speeds()}")
           a = statuts_moteurs(mc)
           a[2]=0 # pour ignoer J3 en panne pour le robot 1

           print("STATUTS :", a)
           print("PROBLEME :", detection_problemes(a))

           if detection_problemes(a):
               Explanation_problemes(a)
               print("Vous avez merdé les loustics")
               Arret_urgence(mc)
              
               return False
           else:
               print("Pas de problème c'est ok la team")

           time.sleep(0.1)
           if time.time() - debut > 0.3:
               if blocage_vitesse_moteur(mc, angle_cible=angles):
                   Arret_urgence(mc)
                   print(f"caca")
                   return False


    
        


    except Exception as e:
        print("Erreur pendant le deplacement :", e)
        return False

    # -----------------------------------------------------
    # Verification de l'arrivee
    # -----------------------------------------------------

    """
    sync_send_angles retourne 1 meme lorsque le timeout
    expire sans que la pose soit atteinte. La seule preuve
    fiable est donc de relire la position mesuree.
    """

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
# 11. EXECUTION AUTONOME
# =========================================================

def main():
    """Sequence complete de mise en position initiale, avec confirmation de l'operateur."""

    print()
    print("=========================================")
    print("   MISE EN POSITION INITIALE DU BRAS")
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
    # Etat de depart
    # -----------------------------------------------------

    print()
    print("Position actuelle :", lireAngles(mc))
    print("Position visee    :", ANGLES_POSITION_INITIALE)

    if estEnPositionInitiale(mc):
        print()
        print("Le bras est deja en position initiale.")
        return 0

    # -----------------------------------------------------
    # Confirmation de l'operateur
    # -----------------------------------------------------

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

    succes = allerPositionInitiale(mc)

    print("-----------------------------------------")

    # -----------------------------------------------------
    # Compte rendu
    # -----------------------------------------------------

    """
    Les moteurs restent volontairement alimentes.

    Un power_off relacherait les servos et le bras
    retomberait sous son propre poids : ce serait perdre
    la position que l'on vient tout juste d'atteindre.
    """

    print()

    if succes:
        print("Bras pret. Les moteurs restent alimentes.")
        return 0

    print("Echec de la mise en position initiale.")
    print("Les moteurs restent alimentes, le bras n'a pas ete relache.")
    return 1


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

