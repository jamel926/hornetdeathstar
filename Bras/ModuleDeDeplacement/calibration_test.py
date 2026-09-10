"""
=========================================================
TEST DE LA CALIBRATION - POINTER UN POINT DEMANDE
=========================================================

Objectif :
---------
Verifier la calibration produite par calibration_grille.py :
on donne des coordonnees sur le quadrillage, le robot
pointe dessus.

Le script sert aussi d'outil de MESURE : apres chaque tir,
on peut saisir l'endroit ou le spot est reellement tombe.
L'ecart est enregistre, et c'est ce chiffre qui dit ce que
vaut la calibration.

Pourquoi la base est verifiee au demarrage :
-------------------------------------------
Le modele decrit la position du spot pour une pose de base
donnee. Si J1, J2 ou J3 ont bouge depuis la calibration,
tout le modele est faux. Le script refuse alors de
continuer plutot que de viser a cote sans le dire.

Utilisation :
------------
    python3 calibration_test.py

ATTENTION :
----------
Pointeur rouge classe 2 uniquement. Le laser de puissance
reste DEBRANCHE.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
from datetime import datetime

"""
Les fonctions du modele et la couche robot viennent du
script de calibration, qui est dans le meme dossier. Son
code executable est sous if __name__ == "__main__" : cet
import n'ouvre donc aucun port serie et ne declenche
aucun mouvement.
"""

from calibration_grille import (
    COTE_CARREAU,
    DOSSIER_RESULTATS,
    J4_MIN, J4_MAX, J5_MIN, J5_MAX,
    MARGE_ENVELOPPE,
    TOLERANCE_BASE,
    allumerPointeur,
    anglesPourPoint,
    approcher,
    chargerModele,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    lireTexte,
    predireImpact,
)


# =========================================================
# 2. PARAMETRES
# =========================================================

FICHIER_VALIDATION = os.path.join(DOSSIER_RESULTATS, "validation_pointage.csv")

ENTETE_VALIDATION = [
    "horodatage", "u_demande_carreaux", "v_demande_carreaux",
    "J4", "J5", "u_reel_carreaux", "v_reel_carreaux",
    "ecart_carreaux",
]


# =========================================================
# 3. VERIFICATIONS
# =========================================================

def verifierBase(mc, modele, tolerance=TOLERANCE_BASE):
    """Compare la base courante a celle de la calibration et retourne True si elle n'a pas bouge."""

    reference = modele.get("base")

    if not reference:
        print("ATTENTION : le modele ne contient pas la pose de base.")
        print("Impossible de verifier que le bras n'a pas bouge depuis la calibration.")
        return True

    mesure = lireAngles(mc)

    if mesure is None:
        print("Lecture des angles impossible.")
        return False

    ecarts = [abs(mesure[i] - reference[i]) for i in range(3)]

    print()
    print("Base de calibration :", [round(a, 2) for a in reference])
    print("Base courante       :", [round(mesure[i], 2) for i in range(3)])
    print("Ecarts              :", [round(e, 2) for e in ecarts])

    if max(ecarts) > tolerance:
        print()
        print("REFUS : la base a bouge de %.2f deg depuis la calibration." % max(ecarts))
        print("Le modele ne decrit plus la geometrie actuelle.")
        print("Remettre le bras dans sa pose de calibration, ou refaire une campagne.")
        return False

    print("Base conforme.")
    return True


# =========================================================
# 4. ENREGISTREMENT DES TIRS
# =========================================================

def enregistrerTir(demande, angles, reel):
    """Ajoute une ligne au fichier de validation et retourne True si l'ecriture a reussi."""

    if reel is None:
        ecart = ""
        u_reel = ""
        v_reel = ""
    else:
        ecart = round(((reel[0] - demande[0]) ** 2 + (reel[1] - demande[1]) ** 2) ** 0.5, 3)
        u_reel = round(reel[0], 3)
        v_reel = round(reel[1], 3)

    try:
        os.makedirs(DOSSIER_RESULTATS, exist_ok=True)

        nouveau = not os.path.exists(FICHIER_VALIDATION)

        with open(FICHIER_VALIDATION, "a", newline="") as fichier:

            redacteur = csv.writer(fichier)

            if nouveau:
                redacteur.writerow(ENTETE_VALIDATION)

            redacteur.writerow([
                datetime.now().isoformat(timespec="seconds"),
                round(demande[0], 3), round(demande[1], 3),
                round(angles[0], 3), round(angles[1], 3),
                u_reel, v_reel, ecart,
            ])

        return True

    except Exception as e:
        print("Impossible d'enregistrer le tir :", e)
        return False


# =========================================================
# 5. DIALOGUE
# =========================================================

def lireCible():
    """Lit le point a viser en carreaux et le retourne, ou None pour terminer."""

    """
    Tout se compte en carreaux, comme pendant la
    calibration. Accepter deux unites dans une meme chaine
    de mesure finit par produire un facteur d'echelle que
    rien ne signale -- ici il vaudrait 3.
    """

    print()
    print("Point a viser : deux nombres en CARREAUX (ex : 4 -2).")
    print("Taper fin pour terminer.")

    while True:

        reponse = lireTexte("cible > ")

        if reponse.lower() in ("fin", "f", "q"):
            return None

        morceaux = reponse.replace(",", " ").split()

        if len(morceaux) != 2:
            print("  Il faut deux nombres separes par un espace.")
            continue

        try:
            return (float(morceaux[0]), float(morceaux[1]))

        except ValueError:
            print("  Saisie illisible.")


def lireImpactReel(demande):
    """Demande ou le spot est reellement tombe, en carreaux, et le retourne ou None."""

    print("  Ou le spot est-il tombe ? deux nombres en carreaux,")
    print("  ou Entree si le releve n'est pas fait.")

    reponse = lireTexte("  impact reel > ")

    if not reponse:
        return None

    morceaux = reponse.replace(",", " ").split()

    if len(morceaux) != 2:
        print("  Saisie ignoree.")
        return None

    try:
        reel = (float(morceaux[0]), float(morceaux[1]))

    except ValueError:
        print("  Saisie ignoree.")
        return None

    ecart = ((reel[0] - demande[0]) ** 2 + (reel[1] - demande[1]) ** 2) ** 0.5

    print("  Ecart de pointage : %.2f carreaux (%.1f cm)" % (ecart, ecart * COTE_CARREAU))

    return reel


def afficherRefus(modele, u, v):
    """Explique pourquoi un point est refuse par le modele."""

    if not dansEnveloppe(modele, u, v, MARGE_ENVELOPPE):
        e = modele["enveloppe"]
        print("  REFUS : point hors de la zone calibree.")
        print("  Zone relevee : u de %.1f a %.1f carreaux, v de %.1f a %.1f carreaux (marge %.1f carreaux)."
              % (e["u_min"], e["u_max"], e["v_min"], e["v_max"], MARGE_ENVELOPPE))
        print("  Le modele n'extrapole pas : il faut etendre la calibration.")
        return

    print("  REFUS : les angles calcules sortent des limites du bras")
    print("  (J4 de %.0f a %.0f, J5 de %.0f a %.0f degres)." % (J4_MIN, J4_MAX, J5_MIN, J5_MAX))


# =========================================================
# 6. PROGRAMME
# =========================================================

def main():
    """Charge le modele, pointe les points demandes et retourne le code de sortie."""

    print("=========================================")
    print("TEST DE LA CALIBRATION")
    print("=========================================")
    print()
    print("SECURITE : pointeur rouge classe 2, laser de puissance DEBRANCHE.")

    modele = chargerModele()

    if modele is None:
        return 1

    print()
    print("Modele du", modele.get("date", "?"), "-", modele.get("nb_points", "?"), "points")

    residus = modele.get("residus")
    if residus:
        print("Erreur annoncee a l'ajustement : rms %.2f carreaux, max %.2f carreaux"
              % (residus["rms_carreaux"], residus["max_carreaux"]))

    residus_validation = modele.get("residus_validation")
    if residus_validation:
        print("Erreur en validation croisee   : rms %.2f carreaux, max %.2f carreaux"
              % (residus_validation["rms_carreaux"], residus_validation["max_carreaux"]))
    else:
        print("Pas de validation croisee dans ce modele : l'erreur reelle n'est pas verifiee.")

    mc = connecterRobot()

    if mc is None:
        return 1

    if not verifierBase(mc, modele):
        return 1

    angles = lireAngles(mc)
    base = angles[0:3]
    j6 = angles[5]

    try:
        if not allumerPointeur(mc):
            print("Le pointeur ne s'allume pas.")

        while True:

            demande = lireCible()

            if demande is None:
                break

            u, v = demande

            resultat = anglesPourPoint(modele, u, v)

            if resultat is None:
                afficherRefus(modele, u, v)
                continue

            j4, j5 = resultat

            print("  Angles calcules : J4 = %+.2f   J5 = %+.2f" % (j4, j5))

            arrive, mesure = approcher(mc, base, j4, j5, j6)

            if mesure is None:
                print("  Mouvement echoue.")
                continue

            if not arrive:
                print("  ATTENTION : la tete n'a pas atteint la consigne.")

            # -------------------------------------------------
            # Ecart de reproduction
            # -------------------------------------------------

            """
            Les angles reellement atteints ne sont pas
            exactement ceux commandes. On affiche ou le
            modele dit que le spot est tombe compte tenu de
            cet ecart : c'est la part mecanique de l'erreur,
            distincte de l'erreur du modele.
            """

            prediction = predireImpact(modele, mesure[3], mesure[4])

            if prediction is not None:
                derive = ((prediction[0] - u) ** 2 + (prediction[1] - v) ** 2) ** 0.5
                if derive > 0.1:
                    print("  Angles atteints : J4 = %+.2f  J5 = %+.2f  -> %.2f carreaux de derive mecanique"
                          % (mesure[3], mesure[4], derive))

            reel = lireImpactReel(demande)

            enregistrerTir(demande, (mesure[3], mesure[4]), reel)

    finally:

        eteindrePointeur(mc)
        print()
        print("Pointeur eteint. Moteurs laisses sous tension.")
        print("Tirs enregistres dans :", FICHIER_VALIDATION)

    return 0


# =========================================================
# 7. EXECUTION AUTONOME
# =========================================================

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
