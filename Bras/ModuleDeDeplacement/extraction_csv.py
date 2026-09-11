import pandas as pd
from PU_PositionVisee2 import aller_en_position_visee
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

# Chargement du fichier
df = pd.read_csv("/home/er/Documents/projet_imersif/hornetdeathstar/Bras/ModuleDeDeplacement/resultats/calibration_grille.csv")

print("Choisir la calibration à tester :")
print("1 - Calib 1")
print("2 - Calib 2")

choix = input("Votre choix : ")

if choix == "1":
    calibration = "Calib 1"
elif choix == "2":
    calibration = "Calib 2"
else:
    print("Choix invalide")
    exit()

# Filtrage
df = df[
    (df["serie"] == calibration) &
    (df["usage"] == "ajustement")
]

print(f"\n{len(df)} points trouvés pour {calibration}")

resultats = []

mc = connecterRobot()
TIMEOUT_INIT = 15  

for _, row in df.iterrows():

    cible_u = row["u_cm"]
    cible_v = row["v_cm"]

    j4 = row["j4_consigne"]
    j5 = row["j5_consigne"]

    print("\n--------------------------------")
    print(f"Cible : ({cible_u}, {cible_v})")
    print(f"Angles : J4={j4:.2f}  J5={j5:.2f}")

    angles = [
    0.0,      # J1 - rotation de la base
    50.0,     # J2 - epaule
    -145.0,   # J3 - coude
    j4,     # J4 - poignet
    j5,      # J5 - inclinaison de l'outil
    0.0       # J6 - rotation finale de l'outil
    ]

    aller_en_position_visee(mc, angles,50, TIMEOUT_INIT)

    input("Appuyez sur Entrée lorsque le robot est en position...")

    u_observe = float(input("Position observée u : "))
    v_observe = float(input("Position observée v : "))

    erreur_u = u_observe - cible_u
    erreur_v = v_observe - cible_v

    ok = (erreur_u == 0 and erreur_v == 0)

    resultats.append({
        "u_cible": cible_u,
        "v_cible": cible_v,
        "u_observe": u_observe,
        "v_observe": v_observe,
        "erreur_u": erreur_u,
        "erreur_v": erreur_v,
        "OK": ok
    })

resultats_df = pd.DataFrame(resultats)

nom_fichier = (
    f"validation_{calibration.replace(' ', '_')}.csv"
)

resultats_df.to_csv(nom_fichier, index=False)

print("\nRésumé")
print(f"Taux de réussite : {100*resultats_df['OK'].mean():.1f}%")
print(f"Fichier enregistré : {nom_fichier}")