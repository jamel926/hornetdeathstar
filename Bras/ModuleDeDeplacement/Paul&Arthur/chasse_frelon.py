"""
=========================================================
CHASSE AU FRELON - VISEE, TIR, ET DANSE DE VICTOIRE
=========================================================

Objectif :
---------
Pointer le laser sur la position donnee (celle du frelon),
l'allumer pendant une duree fixe, l'eteindre, puis faire
executer une petite sequence de mouvements ("danse") au bras
pour signaler que le frelon est mort.

Ce script reutilise telle quelle la calibration geometrique
et la correction apprise deja en place dans calibration_test.py
(choix de la calibration, verification de la base, correction
apprise via apprentissage_correction) : il ne fait qu'ajouter
le cycle allumage / duree fixe / extinction / danse APRES le
pointage.

ATTENTION SECURITE :
--------------------
Ce script pilote la MEME broche (33) que le pointeur rouge
classe 2 utilise dans tous les autres scripts du depot
(allumerPointeur / eteindrePointeur de calibration_grille.py).

Tant que le laser de puissance reel n'est pas cable sur une
broche et un circuit de securite dedies (et verifie comme
tel), NE PAS considerer ce script pret pour un laser de
puissance : il n'allume ici que ce que allumerPointeur()
allume deja ailleurs dans le depot.

DUREE_TIR est une valeur de depart a ajuster une fois que
vous connaissez le temps reel necessaire pour immobiliser un
frelon. SEQUENCE_DANSE est volontairement une petite oscillation
de faible amplitude autour du point de tir : a personnaliser
librement, elle passe par les memes verifications de bornes
que le reste du programme (viserAngles refuse silencieusement
tout mouvement hors limites).

Utilisation :
------------
    python3 chasse_frelon.py
"""

import time

from calibration_grille import (
    TOLERANCE_POINTAGE,
    allumerPointeur,
    anglesPourPoint,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    lireTexte,
    viserAngles,
)

from PU_PositionInitiale import allerPositionInitiale

from calibration_test import (
    choisirCalibration,
    construireModeleSerie,
    verifierBase,
)

from apprentissage_correction import chargerCorrection, cibleCorrigee


# =========================================================
# PARAMETRES A AJUSTER
# =========================================================

# Duree pendant laquelle le laser reste allume sur la cible
# avant d'etre considere comme suffisant. A AJUSTER selon le
# temps reel d'efficacite mesure sur le terrain.
DUREE_TIR = 3.0   # secondes

# Sequence de danse : liste de deplacements (dJ4, dJ5) en
# degres, appliques successivement AUTOUR de la position de
# tir (donc en absolu : angle_tir + delta). Amplitude petite
# a dessein pour rester loin des butees mecaniques ; le
# dernier point revient a (0, 0) pour terminer sur la cible.
SEQUENCE_DANSE = [
    (10, 0),
    (-10, 10),
    (0, -10),
    (-10, 0),
    (10, -10),
    (0, 10),
    (0, 0),
]

PAUSE_ENTRE_PAS_DANSE = 0.15   # secondes


# =========================================================
# DANSE DE VICTOIRE
# =========================================================

def danserVictoire(mc, base, j4_tir, j5_tir, j6):
    """Fait osciller la tete autour de la position de tir, puis revient dessus."""

    print()
    print("Le frelon est mort. Petite danse de victoire...")

    for dj4, dj5 in SEQUENCE_DANSE:

        cible_j4 = j4_tir + dj4
        cible_j5 = j5_tir + dj5

        # viserAngles() refuse tout seul un mouvement hors des
        # bornes constructeur (J4_MIN/MAX, J5_MIN/MAX) : pas de
        # verification supplementaire necessaire ici.
        arrive, mesure = viserAngles(mc, base, cible_j4, cible_j5, j6)

        if mesure is None:
            print("  Mouvement de danse refuse, arret de la sequence.")
            return

        time.sleep(PAUSE_ENTRE_PAS_DANSE)

    print("Danse terminee.")


# =========================================================
# SAISIE DE LA POSITION DU FRELON
# =========================================================

def lireCibleFrelon():
    """Lit la position du frelon en carreaux et la retourne, ou None pour terminer."""

    print()
    print("Position du frelon : deux nombres en CARREAUX (ex : 4 -2).")
    print("Taper fin pour terminer.")

    while True:

        reponse = lireTexte("frelon > ")

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


# =========================================================
# PROGRAMME PRINCIPAL
# =========================================================

def main():
    print("=========================================")
    print("CHASSE AU FRELON")
    print("=========================================")
    print()
    print("SECURITE : verifier que c'est bien le pointeur rouge classe 2")
    print("qui est branche sur la broche du laser avant de continuer.")

    # -----------------------------------------------------
    # Calibration, comme dans calibration_test.py
    # -----------------------------------------------------

    serie = choisirCalibration()

    if serie is None:
        print("Aucune calibration selectionnee.")
        return 0

    print()
    print("Calibration selectionnee :", serie)

    modele = construireModeleSerie(serie)

    if modele is None:
        return 1

    mc = connecterRobot()

    if mc is None:
        return 1

    allerPositionInitiale(mc)

    if not verifierBase(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    # -----------------------------------------------------
    # Correction apprise, comme dans calibration_test.py
    # -----------------------------------------------------

    correction = chargerCorrection()

    if correction is None:
        print()
        print("Pas de correction apprise : le modele est utilise tel quel.")
    else:
        print()
        print(
            "Correction apprise '%s' appliquee (%s, %d points)."
            % (
                correction.get("type", "?"),
                correction.get("serie", "serie inconnue"),
                correction.get("nb_points", 0),
            )
        )

    try:
        while True:

            demande = lireCibleFrelon()

            if demande is None:
                break

            u, v = demande

            # -------------------------------------------------
            # Verification de l'enveloppe sur la cible ORIGINALE.
            # -------------------------------------------------

            if not dansEnveloppe(modele, u, v):
                e = modele["enveloppe"]
                print("  Point hors de la zone de la calibration '%s'." % serie)
                print(
                    "  Zone autorisee : u %.1f..%.1f, v %.1f..%.1f carreaux."
                    % (e["u_min"], e["u_max"], e["v_min"], e["v_max"])
                )
                continue

            # La correction deplace la cible donnee au modele ;
            # le modele lui-meme n'est pas touche.
            vise_u, vise_v = cibleCorrigee(correction, u, v)

            if (vise_u, vise_v) != (u, v):
                print("  Cible corrigee : (%.2f, %.2f) carreaux" % (vise_u, vise_v))

            resultat = anglesPourPoint(modele, vise_u, vise_v)

            if resultat is None:
                print("  Impossible de calculer J4/J5 pour ce point avec cette calibration.")
                continue

            j4, j5 = resultat

            print("  Angles calcules : J4 = %+.2f   J5 = %+.2f" % (j4, j5))

            vise, mesure = viserAngles(mc, base, j4, j5, j6)

            if mesure is None:
                print("  Mouvement echoue.")
                continue

            if not vise:
                print(
                    "  ATTENTION : la tete se repose a %+.2f / %+.2f deg de l'angle demande"
                    % (mesure[3] - j4, mesure[4] - j5)
                )
                print("  (tolerance %.2f deg)." % TOLERANCE_POINTAGE)

            # -------------------------------------------------
            # Tir : allumage, duree fixe, extinction
            # -------------------------------------------------

            if not allumerPointeur(mc):
                print("  Le pointeur ne s'allume pas, tir annule.")
                continue

            print("  Laser allume, tir en cours (%.1f s)..." % DUREE_TIR)
            time.sleep(DUREE_TIR)

            eteindrePointeur(mc)
            print("  Laser eteint : frelon considere mort.")

            # -------------------------------------------------
            # Danse de victoire, puis retour sur la cible
            # -------------------------------------------------

            danserVictoire(mc, base, mesure[3], mesure[4], j6)

            viserAngles(mc, base, j4, j5, j6)

    finally:

        eteindrePointeur(mc)
        print()
        print("Pointeur eteint. Moteurs laisses sous tension.")
        print("Calibration utilisee :", serie)

    return 0


# =========================================================
# EXECUTION AUTONOME
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
