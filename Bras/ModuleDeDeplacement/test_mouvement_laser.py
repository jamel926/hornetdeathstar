from pymycobot import MyCobot280
import time

from Bras.ModuleDeDeplacement.Version_Finale.PU_PositionVisee import (
    Mise_en_Position_Initiale,
    Verification_Angle_SECU,
    aller_en_position_visee
)

from Bras.ModuleDeDeplacement.PU_PositionInitiale import (
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

def main():
    Mise_en_Position_Initiale()
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
    mc = MyCobot280('/dev/ttyTHS1', 1000000)
    print()
    print("Position actuelle :", lireAngles(mc))
    print("Position visee    :", ANGLES_POSITION_VISEES)

    print()
    confirmation = input("Lancer la mise en position visée ? (o/n) : ")

    if confirmation.lower() != "o":
        print("Mise en position visee annulee par l'operateur.")
        return 1
    
    # -----------------------------------------------------
    # Mouvement
    # -----------------------------------------------------
    print()
    print("-----------------------------------------")
    print(ANGLES)
    succes = aller_en_position_visee(mc,ANGLES)
    print("-----------------------------------------")

    # -----------------------------------------------------
    # LASER
    # -----------------------------------------------------
    PIN = 33

    print("Configuration G33 en sortie")
    mc.set_pin_mode(PIN, 1)
    time.sleep(0.1)
    print("G33 = 1 -> LASER ETEINT")
    time.sleep(1)
    
    mc.set_digital_output(PIN, 0)
    print("G33 = 0 -> LASER ALLUME")
    time.sleep(5)

    print("G33 = 1 -> LASER ETEINT")
    mc.set_digital_output(PIN, 1)
    time.sleep(1)

    print("Fin du laser, merci bonsoir")

    Mise_en_Position_Initiale()

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