import matplotlib.pyplot as plt
from pymycobot import MyCobot280
import time
import numpy as np
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


##### OBJECTIF : trouver la vitesse de mouvement permettant d'être le plus précis pour J4


def lireJ4(mc):
    angles = mc.get_angles() #return a float list of all degree 
    J4 = angles[3]
    return J4

def erreur_init(J4_init,J4_init_lu):
    res = J4_init - J4_init_lu
    return(abs(res))

def erreur_finale(J4_final, J4_final_lu):
    res = J4_final - J4_final_lu
    return(abs(res))

def aller_en_position_visee(mc, angles=None, vitesse = int , timeout=TIMEOUT_INIT):
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

    

        # La vitesse doit imperativement etre un entier
        mc.sync_send_angles(angles, int(vitesse), timeout)

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

def main():

    liste_vitesses = [i*5 for i in range(1, 20)] # génère la liste des vitesses à tester de 0 à 100 de 5 en 5

    ANGLES_POSITION_INITIALE = [
        0.0,      # J1 - rotation de la base
        6,     # J2 - epaule
        -105.0,   # J3 - coude
        -10,     # J4 - poignet
        -2,      # J4 - inclinaison de l'outil
        0.0       # J6 - rotation finale de l'outil
    ]

    ANGLES_POSITION_FINALE = [
        0.0,      # J1 - rotation de la base
        6.0,     # J2 - epaule
        -105.0,   # J3 - coude
        100.0,     # J4 - poignet
        -2,      # J4 - inclinaison de l'outil
        0.0       # J6 - rotation finale de l'outil
    ]
    
    mc=connecterRobot()
    liste_erreur_J4_initiale = []
    liste_erreur_J4_finale = []
    for vitesse in liste_vitesses :

        aller_en_position_visee(mc,ANGLES_POSITION_INITIALE, 50,timeout=TIMEOUT_INIT)
        J4_init_lu = lireJ4(mc)
        print(f"VITESSE ENVOYEE : {vitesse}--------------------------------------------")
        print(J4_init_lu)
        liste_erreur_J4_initiale.append(erreur_init(-10,J4_init_lu))
        print(f"l'erreur de position initiale est : {erreur_init(-10,J4_init_lu)}")
        print()
        aller_en_position_visee(mc,ANGLES_POSITION_FINALE, vitesse,timeout=TIMEOUT_INIT)
        J4_final_lu = lireJ4(mc)
        liste_erreur_J4_finale.append(erreur_finale(100,J4_final_lu))
        print(f"l'erreur de position finale est : {erreur_finale(100,J4_final_lu)}")
        print()
    
    print("erreur initiale",liste_erreur_J4_initiale)
    print("erreur finale",liste_erreur_J4_finale) 

    plt.figure()
    plt.plot(liste_vitesses, liste_erreur_J4_finale)
    plt.title("Erreur de J4")
    plt.xlabel("vitesse")
    plt.ylabel("Erreur angulaire")
    plt.grid()
    plt.show()
    plt.close()

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
    



    

