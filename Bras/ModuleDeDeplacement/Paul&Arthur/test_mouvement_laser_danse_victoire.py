from pymycobot import MyCobot280
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
    BAUDRATE
)
VITESSE_INIT=100

from PU_PositionVisee2 import (
    Verification_Angle_SECU,
    aller_en_position_visee
)

def Mise_en_Position_Initiale(vitesse=VITESSE_INIT, utilisateur = bool):
    
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
    # Confirmation de l'operateur si utilisateur = True
    # -----------------------------------------------------
    if utilisateur==True :
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

    """
    Amene le bras dans la position initiale et confirme qu'il l'a bien atteinte.
    """

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

        # La vitesse doit imperativement etre un entier
        mc.send_angles(angles, int(vitesse), timeout)

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


def danse_victoire(mc, vitesse=100):
    """
    Danse de victoire rapide et marrante.
    Tous les moteurs sauf J6 bougent.
    J6 reste a 0 degre.
    """
    print()
    print("=========================================")
    print("     DANSE DE VICTOIRE RAPIDE !!!")
    print("=========================================")

    # Choregraphie volontairement dynamique.
    # J6 reste toujours a 0.
    poses = [
        [-35.0,  35.0, -125.0,  35.0,  25.0, 0.0],
        [ 35.0,  65.0, -155.0, -35.0, -25.0, 0.0],
        [-35.0,  65.0, -155.0,  35.0,  25.0, 0.0],
        [ 35.0,  35.0, -125.0, -35.0, -25.0, 0.0],
        [-25.0,  55.0, -145.0,  45.0,  20.0, 0.0],
        [ 25.0,  55.0, -145.0, -45.0, -20.0, 0.0],
        [  0.0,  45.0, -135.0,   0.0,   0.0, 0.0],
    ]

    try:
        for tour in range(2):
            print("Tour de danse", tour + 1, "/ 2")

            for i, pose in enumerate(poses, start=1):
                print("  Move", i, "->", pose)

                # 100 = vitesse rapide.
                mc.send_angles(pose, int(vitesse), 2)

                # Temps tres court pour donner un effet dynamique.
                time.sleep(0.25)

        print("DANSE TERMINEE !!!")
        return True

    except Exception as e:
        print("Erreur pendant la danse de victoire :", e)
        return False


def main():
    mc = MyCobot280('/dev/ttyTHS1', 1000000)
    # -----------------------------------------------------
    # LASER
    # -----------------------------------------------------
    PIN = 33

    print("Configuration G33 en sortie")
    mc.set_pin_mode(PIN, 0)
    time.sleep(0.1)
    print("G33 = 0 -> LASER ETEINT")
    time.sleep(1)


    Mise_en_Position_Initiale(100, True)
    time.sleep(2)

    VAL_J4 = float(input("Valeur de J4 voulue : "))
    VAL_J5 = float(input("Valeur de J5 voulue : "))

    ANGLES_POSITION_VISEES= [
    0.0,      # J1 - rotation de la base
    50.0,     # J2 - epaule
    -145.0,   # J3 - coude
    VAL_J4,     # J4 - poignet
    VAL_J5,      # J5 - inclinaison de l'outil
    0.0       # J6 - rotation finale de l'outil
]
    
    ANGLES=Verification_Angle_SECU(ANGLES_POSITION_VISEES)
    
    print()
    print("Position actuelle :", lireAngles(mc))
    print("Position visee    :", ANGLES_POSITION_VISEES)

    
    # -----------------------------------------------------
    # Mouvement
    # -----------------------------------------------------
    print()
    print("-----------------------------------------")
    print(ANGLES)
    succes = aller_en_position_visee(mc,ANGLES,50)
    print("-----------------------------------------")

    
    mc.set_pin_mode(PIN, 1)
    time.sleep(0.1)
    mc.set_digital_output(PIN, 0)
    print("G33 = 1 -> LASER ALLUME")
    time.sleep(5)

    print("G33 = 0 -> LASER ETEINT")
    mc.set_digital_output(PIN, 1)
    time.sleep(1)

    # Une fois le laser eteint, le robot fait sa danse de victoire.
    danse_victoire(mc, 100)
    time.sleep(1)

    print("Fin du laser, merci bonsoir")


    # -----------------------------------------------------
    # Remise en position initiale
    # -----------------------------------------------------

    Mise_en_Position_Initiale(100, False)

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