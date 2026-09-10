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
    from PU_PositionInitiale import connecterRobot, allerPositionInitiale

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
"""


# =========================================================
# 1. IMPORTS
# =========================================================

from pymycobot.mycobot280 import MyCobot280
import time
from PU_PositionInitiale import (
    connecterRobot,
    lireAngles,
    verifierAngles,
    allerPositionInitiale,
    estEnPositionInitiale,
    ANGLES_POSITION_INITIALE,
    NOMS_ARTICULATIONS,
    VITESSE_INIT,
    TIMEOUT_INIT,
    ecartMaximal,
    TOLERANCE_ANGLE,
    PORT,
    BAUDRATE,
    statuts_moteurs,
    detection_problemes, 
    Explanation_problemes, 
    Arret_urgence, 
    blocage_vitesse_moteur, 
    ReculerPositionPrec, 
)

def Mise_en_Position_Initiale(vitesse):
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
    # Mouvement
    # -----------------------------------------------------

    print()
    print("-----------------------------------------")

    succes = allerPositionInitiale(mc,ANGLES_POSITION_INITIALE,vitesse)

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
        print()
        print("=========================================")
        print("   POSITION DU BRAS INITIALISE ET PRET A L'EMPLOI ")
        print("=========================================")
        print()
        return 0

    print("Echec de la mise en position initiale.")
    print("Les moteurs restent alimentes, le bras n'a pas ete relache.")
    return


def Verification_Angle_SECU(angle):
    J1,J2,J3,J4,J5,J6 = angle
    if J4 > 95.0 : 
        print("L'angle J4 voulu est hors zone reglementée")
        print("changement de l'angle pour Angl_Max=95°")
        J4=95.0
    if J4 < 0.0 : 
        print("L'angle J4 voulu est hors zone reglementée")
        print("changement de l'angle pour Angl_Min=0°")
        J4=0.0
    if J5 <-80.0 : 
        print("L'angle J5 voulu est hors zone reglementé")
        print("changement de l'angle pour Angl_Max=-80°")
        J5=-80
    if J5 >80.0 : 
        print("L'angle J5 voulu est hors zone reglementé")
        print("changement de l'angle pour Angl_Max=80°")
        J5=80.0
    angle=(J1,J2,J3,J4,J5,J6)
    return(angle)

def aller_en_position_visee(mc, angles=None, vitesse=VITESSE_INIT, timeout=TIMEOUT_INIT):
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
        derniers_angles = lireAngles(mc)


       # Surveillance pendant le mouvement
        while mc.is_moving() == 1:
           a = statuts_moteurs(mc)
           a[2]=0 # pour ignoer J3 en panne pour le robot 1

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
    Mise_en_Position_Initiale(50)

    VAL_J1 = 0.0 #float(input("Valeur de J1 voulue : "))
    VAL_J2 = 50.0 #float(input("Valeur de J2 voulue : "))
    VAL_J3 = -145.0 #float(input("Valeur de J3 voulue : "))
    VAL_J4 = float(input("Valeur de J4 voulue : "))
    VAL_J5 = float(input("Valeur de J5 voulue : "))
    VAL_J6 = 0.0 #float(input("Valeur de J6 voulue : "))

    ANGLES_POSITION_VISEES= [
    VAL_J1,      # J1 - rotation de la base
    VAL_J2,     # J2 - epaule
    VAL_J3,   # J3 - coude
    VAL_J4,     # J4 - poignet
    VAL_J5,      # J5 - inclinaison de l'outil
    VAL_J6       # J6 - rotation finale de l'outil
]
    ANGLES=Verification_Angle_SECU(ANGLES_POSITION_VISEES)
    mc = MyCobot280(PORT, BAUDRATE)
    print()
    print("Position actuelle :", lireAngles(mc))
    print("Position visee    :", ANGLES_POSITION_VISEES)

    # -----------------------------------------------------
    # Mouvement
    # -----------------------------------------------------
    print()
    print("-----------------------------------------")
    print(ANGLES)
    succes = aller_en_position_visee(mc,ANGLES)
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
