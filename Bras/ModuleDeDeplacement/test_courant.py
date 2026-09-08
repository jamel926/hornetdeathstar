"""
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
import contextlib
import io
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
VITESSE_INIT = 10        # int, 1 a 100 (impose par pymycobot)
# Duree maximale accordee au bras pour atteindre la pose.
TIMEOUT_INIT = 15        # secondes
# Ecart angulaire tolere entre la consigne et la position
# reellement mesuree a l'arrivee.
TOLERANCE_ANGLE = 2.0    # degres
# Frequence de surveillance des servos pendant le mouvement.
INTERVALLE_SURVEILLANCE = 0.05   # secondes
# Detection de blocage mecanique : get_servo_status() ne
# declenche que sur une protection electrique/thermique du
# firmware (seuil eleve). Une perturbation qui empeche le
# bras d'avancer sans faire grimper le courant jusqu'a ce
# seuil ne serait pas vue. On la detecte donc en verifiant
# que le bras progresse reellement vers sa cible.
SEUIL_STAGNATION = 0.3       # degres : en dessous, on considere qu'il n'y a pas eu de progres
DUREE_BLOCAGE_MAX = 0.15      # secondes sans progres avant de considerer le bras bloque
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
# Articulation actuellement en panne connue (J3), exclue des
# controles de securite le temps qu'elle soit reparee : sans
# ca, get_servo_status() renvoie en permanence une erreur sur
# elle, ce qui declenche mc.stop() a chaque appel et empeche
# tout mouvement, meme sur les articulations saines.
# Remettre a None une fois J3 repare, pour reactiver le
# controle complet dessus.
ARTICULATION_IGNOREE = 2   # index : 0=J1, 1=J2, 2=J3, 3=J4, 4=J5, 5=J6
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

ANGLES_POSITION_INITIALE = [
    2.0,      # J1 - rotation de la base
    48.0,     # J2 - epaule
    -144.0,   # J3 - coude
    75.0,     # J4 - poignet
    1.0,      # J5 - inclinaison de l'outil
    1.0       # J6 - rotation finale de l'outil
]
'''
ANGLES_POSITION_INITIALE = [
    20.0,      # J1 - rotation de la base
    60.0,     # J2 - epaule
    30.0,   # J3 - coude
    20.0,     # J4 - poignet
    10.0,      # J5 - inclinaison de l'outil
    10.0       # J6 - rotation finale de l'outil
]
'''
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
def lireStatutServos(mc):
    """
    Equivalent de mc.get_servo_status(), sans le print
    automatique que pymycobot fait pour chaque erreur trouvee
    (pratique ici : J3 est deja connu en panne et spammerait
    la console a chaque appel, plusieurs fois par seconde).
    """
    with contextlib.redirect_stdout(io.StringIO()):
        return mc.get_servo_status()
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
def ecartMaximal(angles_mesures, angles_cibles, ignorer=None):
    """Retourne le plus grand ecart, en degres, entre la pose mesuree et la pose visee."""
    ecarts = [
        abs(angles_mesures[i] - angles_cibles[i])
        for i in range(6)
        if i != ignorer
    ]
    return max(ecarts)
def estEnPositionInitiale(mc, angles=None, tolerance=TOLERANCE_ANGLE):
    """Indique si le bras se trouve deja dans la position initiale, sans le faire bouger."""
    if angles is None:
        angles = ANGLES_POSITION_INITIALE
    angles_mesures = lireAngles(mc)
    if angles_mesures is None:
        return False
    return ecartMaximal(angles_mesures, angles, ignorer=ARTICULATION_IGNOREE) <= tolerance
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
    ### Controle des incidents avant le mouvement :
    a=lireStatutServos(mc)
    print("servo_status : ",a) #renvoie [0,0,[i],0,0,0] si 0 alors pas de pb sur J correspondant  et si [i] = errreur de type i
    print()
    a_verifie = list(a)
    if ARTICULATION_IGNOREE is not None:
        a_verifie[ARTICULATION_IGNOREE] = 0
    if a_verifie != [0,0,0,0,0,0] :
        mc.stop()
        print("Bras à l'arrêt - Incident sur une des articulations")
        for i in range(0,6):
            if i == ARTICULATION_IGNOREE:
                continue
            if a[i]!=0 :
                print("Incident sur J",i+1)
                if a[i]==[0]:
                    print("Voltage Problem sur J",i+1)
                if a[i]==[1]:
                    print("Sensor Problem sur J",i+1)
                if a[i]==[2]:
                    print("Temperature Problem sur J",i+1)
                if a[i]==[3]:
                    print("Current Problem sur J",i+1)
                if a[i]==[4]:
                    print("Angle Problem sur J",i+1)
                if a[i]==[5]:
                    print("OverLoad Problem sur J",i+1)
        return False
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
        # Envoi de la consigne (non-bloquant)
        # -------------------------------------------------
        """
        On utilise send_angles (non-bloquant) et non
        sync_send_angles (bloquant) : sync_send_angles ne
        rend la main qu'a l'arrivee ou au timeout, ce qui
        empeche toute surveillance des servos PENDANT le
        deplacement. Avec send_angles, on garde la main et
        on peut verifier get_servo_status en continu, pour
        arreter immediatement le bras si un incident
        apparait en cours de trajet (pas seulement avant).
        """
        print("Deplacement en cours (vitesse", int(vitesse), ")...")
        mc.send_angles(angles, int(vitesse))
        debut = time.time()
        angles_precedents = lireAngles(mc)
        # Un chrono de "dernier progres" PAR articulation, et non un
        # seul chrono global : sinon, un joint bloque est masque des
        # qu'un autre joint continue de bouger (ecartMaximal ne
        # renvoie que le plus grand ecart parmi les 6).
        temps_dernier_progres = [time.time()] * 6
        while mc.is_moving() == 1:
            if time.time() - debut > timeout:
                print("Timeout atteint pendant le deplacement.")
                mc.stop()
                return False
            ### Controle des incidents pendant le mouvement (protection electrique/thermique) :
            a = lireStatutServos(mc)
            a_verifie = list(a)
            if ARTICULATION_IGNOREE is not None:
                a_verifie[ARTICULATION_IGNOREE] = 0
            if a_verifie != [0,0,0,0,0,0] :
                mc.stop()
                print("Bras à l'arrêt - Incident sur une des articulations")
                for i in range(0,6):
                    if i == ARTICULATION_IGNOREE:
                        continue
                    if a[i]!=0 :
                        print("Incident sur J",i+1)
                        if a[i]==[0]:
                            print("Voltage Problem sur J",i+1)
                        if a[i]==[1]:
                            print("Sensor Problem sur J",i+1)
                        if a[i]==[2]:
                            print("Temperature Problem sur J",i+1)
                        if a[i]==[3]:
                            print("Current Problem sur J",i+1)
                        if a[i]==[4]:
                            print("Angle Problem sur J",i+1)
                        if a[i]==[5]:
                            print("OverLoad Problem sur J",i+1)
                return False
            ### Controle de blocage mecanique (une articulation ne progresse plus vers sa cible) :
            """
            Une perturbation qui ne fait pas grimper le
            courant jusqu'au seuil de get_servo_status() (une
            simple gene, un contact leger) n'apparait pas dans
            "a" ci-dessus. On la detecte ici en comparant la
            position mesuree d'un cycle a l'autre, ARTICULATION
            PAR ARTICULATION : un seul chrono global masquerait
            un joint bloque tant qu'un autre joint continue de
            bouger (ecartMaximal ne renvoie que le plus grand
            ecart parmi les 6, donc un joint immobile passe
            inapercu si un autre progresse).
            Un joint deja arrive a sa cible (dans la tolerance)
            n'est pas bloque, il a juste fini : on ne le compte
            pas comme un incident.
            """
            angles_actuels = lireAngles(mc)
            if angles_actuels is not None:
                for i in range(6):
                    if i == ARTICULATION_IGNOREE:
                        continue
                    deja_arrive = abs(angles_actuels[i] - angles[i]) <= TOLERANCE_ANGLE
                    a_progresse = (angles_precedents is None) or (abs(angles_actuels[i] - angles_precedents[i]) > SEUIL_STAGNATION)
                    if deja_arrive or a_progresse:
                        temps_dernier_progres[i] = time.time()
                    if time.time() - temps_dernier_progres[i] > DUREE_BLOCAGE_MAX:
                        mc.stop()
                        print("Bras à l'arrêt - blocage detecte sur", NOMS_ARTICULATIONS[i])
                        return False
                angles_precedents = angles_actuels
            time.sleep(INTERVALLE_SURVEILLANCE)
    except Exception as e:
        print("Erreur pendant le deplacement :", e)
        return False
    # -----------------------------------------------------
    # Verification de l'arrivee
    # -----------------------------------------------------
    """
    On relit la position mesuree pour confirmer l'arrivee,
    puisque send_angles ne garantit rien a lui seul.
    """
    angles_mesures = lireAngles(mc)
    if angles_mesures is None:
        print("Position atteinte inconnue : le robot ne repond pas.")
        return False
    ecart = ecartMaximal(angles_mesures, angles, ignorer=ARTICULATION_IGNOREE)
    print("Angles mesures :", angles_mesures)
    print("Ecart maximal :", round(ecart, 2), "degres")
    if ecart > TOLERANCE_ANGLE:
        print("La position initiale n'est PAS atteinte.")
        print("Tolerance :", TOLERANCE_ANGLE, "degres")
        return False
    print("Position initiale atteinte.")
    return True
# =========================================================
# 11. SURVEILLANCE EN POSITION STATIONNAIRE
# =========================================================
def verifierPositionStationnaire(mc, angles=None, tolerance=TOLERANCE_ANGLE):
    """
    A appeler en boucle pendant que le bras est cense rester
    immobile (phase de visee, laser actif).
    Contrairement a la detection utilisee pendant un
    deplacement (angle actuel vs angle precedent), on ne peut
    pas se baser sur "pas de progres" ici : "pas de progres"
    est justement l'etat normal quand le bras est immobile.
    On compare donc a la position FIXE qu'on est censes
    tenir, pas au cycle precedent.
    Arrete le bras et retourne False au moindre probleme
    (incident servo ou ecart de position). Retourne True si
    tout va bien.
    """
    if angles is None:
        angles = ANGLES_POSITION_INITIALE
    ### Controle des incidents (protection electrique/thermique) :
    a = lireStatutServos(mc)
    a_verifie = list(a)
    if ARTICULATION_IGNOREE is not None:
        a_verifie[ARTICULATION_IGNOREE] = 0
    if a_verifie != [0,0,0,0,0,0] :
        mc.stop()
        print("Bras à l'arrêt - Incident sur une des articulations")
        for i in range(0,6):
            if i == ARTICULATION_IGNOREE:
                continue
            if a[i]!=0 :
                print("Incident sur J",i+1)
                if a[i]==[0]:
                    print("Voltage Problem sur J",i+1)
                if a[i]==[1]:
                    print("Sensor Problem sur J",i+1)
                if a[i]==[2]:
                    print("Temperature Problem sur J",i+1)
                if a[i]==[3]:
                    print("Current Problem sur J",i+1)
                if a[i]==[4]:
                    print("Angle Problem sur J",i+1)
                if a[i]==[5]:
                    print("OverLoad Problem sur J",i+1)
        return False
    ### Controle de l'ecart par rapport a la position a tenir :
    angles_mesures = lireAngles(mc)
    if angles_mesures is None:
        print("Position inconnue : le robot ne repond pas.")
        return False
    ecart = ecartMaximal(angles_mesures, angles, ignorer=ARTICULATION_IGNOREE)
    if ecart > tolerance:
        mc.stop()
        print("Bras à l'arrêt - le bras a ete deplace de sa position (ecart :", round(ecart, 2), "degres)")
        return False
    return True
# =========================================================
# 12. EXECUTION AUTONOME
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
    ### Controle des incidents :
    a=lireStatutServos(mc)
    print("servo_status : ",a) #renvoie [0,0,[i],0,0,0] si 0 alors pas de pb sur J correspondant  et si [i] = errreur de type i
    print()
    a_verifie = list(a)
    if ARTICULATION_IGNOREE is not None:
        a_verifie[ARTICULATION_IGNOREE] = 0
    if a_verifie != [0,0,0,0,0,0] :
        mc.stop()
        print("Bras à l'arrêt - Incident sur une des articulations")
        for i in range(0,6):
            if i == ARTICULATION_IGNOREE:
                continue
            if a[i]!=0 :
                print("Incident sur J",i+1)
                if a[i]==[0]:
                    print("Voltage Problem sur J",i+1)
                if a[i]==[1]:
                    print("Sensor Problem sur J",i+1)
                if a[i]==[2]:
                    print("Temperature Problem sur J",i+1)
                if a[i]==[3]:
                    print("Current Problem sur J",i+1)
                if a[i]==[4]:
                    print("Angle Problem sur J",i+1)
                if a[i]==[5]:
                    print("OverLoad Problem sur J",i+1)
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


