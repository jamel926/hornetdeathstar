"""
=========================================================
ROBOT MYCOBOT 280 - DERIVE DE LA BASE SOUS L'USAGE
=========================================================

Objectif :
---------
Repondre a la seule question qui decide si un bras dont un
axe de base est tenu par du frottement est utilisable :

    est-ce que la base reste ou on l'a mise, une fois que
    la tete a bouge des dizaines de fois ?

Un axe qui tient a l'arret ne prouve rien. C'est sous les
a-coups repetes de J4 et J5 qu'un reducteur qui freine
finit par ceder, et il cede lentement : quelques dixiemes
de degre par dizaine de cycles, invisibles a l'oeil.

Ce que ca coute si on ne le mesure pas :
---------------------------------------
Toute la calibration de pointage est ancree sur la pose de
base. Si elle derive d'un degre entre le moment ou on
calibre et le moment ou on tire, l'erreur se retrouve
integralement dans la position du point d'impact, sans que
le systeme ait le moindre moyen de s'en apercevoir.

Prerequis :
----------
Une reference de base enregistree avec PU_Tete (option 1).

Utilisation :
------------
    python3 test_derive_base.py

Le test tourne seul, sans operateur. Laisser le bras
travailler et revenir a la fin.

ATTENTION :
----------
Laser debranche, espace degage : la tete va balayer en
continu pendant plusieurs minutes.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import csv
import os
import time
from datetime import datetime

from PU_PositionInitiale import (
    connecterRobot,
    lireAngles,
    verifierAngles,
    NOMS_ARTICULATIONS
)

from PU_Tete import (
    bougerTete,
    lireBase,
    chargerReference,
    construirePose,
    INDICES_BASE,
    TOLERANCE_BASE
)

from PU_Journal import (
    ouvrirJournal,
    ajouterMesure,
    demanderObservation,
    fermerJournal,
    annoncerJournal
)


# =========================================================
# 2. PARAMETRES
# =========================================================

NB_CYCLES_DEFAUT = 60

# Amplitude du balayage de la tete autour de sa reference.
AMPLITUDE_J4 = 30.0     # degres
AMPLITUDE_J5 = 30.0     # degres

# Frequence d'affichage de l'avancement.
PERIODE_AFFICHAGE = 10  # cycles


# =========================================================
# 3. POSES DE TETE PARCOURUES
# =========================================================

def construirePosesTete(tete_reference):
    """Retourne les poses de tete parcourues en boucle, autour de la reference."""

    """
    Les quatre coins puis le centre : c'est le parcours qui
    fait le plus travailler le poignet dans les deux axes,
    donc celui qui sollicite le plus la base. Un balayage
    fin serait plus joli et moins severe.
    """

    j4 = tete_reference[0]
    j5 = tete_reference[1]
    j6 = tete_reference[2]

    return [
        [j4 - AMPLITUDE_J4, j5 - AMPLITUDE_J5, j6],
        [j4 + AMPLITUDE_J4, j5 + AMPLITUDE_J5, j6],
        [j4 - AMPLITUDE_J4, j5 + AMPLITUDE_J5, j6],
        [j4 + AMPLITUDE_J4, j5 - AMPLITUDE_J5, j6],
        [j4, j5, j6],
    ]


def filtrerPosesValides(poses, base_reference):
    """Ecarte les poses de tete qui sortiraient des limites articulaires."""

    valides = []

    for tete in poses:

        if verifierAngles(construirePose(base_reference, tete)):
            valides.append(tete)
        else:
            print("Pose de tete ecartee (hors limites) :", [round(a, 1) for a in tete])

    return valides


# =========================================================
# 4. FICHIER DE RESULTATS
# =========================================================

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")
FICHIER_RESULTATS = os.path.join(DOSSIER_RESULTATS, "derive_base.csv")

ENTETE_CSV = ["horodatage", "cycle", "J1", "J2", "J3",
              "derive_J1", "derive_J2", "derive_J3"]


def enregistrerCycle(cycle, base, derives):
    """Ajoute une ligne au fichier de resultats de la derive."""

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        fichier_neuf = not os.path.isfile(FICHIER_RESULTATS)

        with open(FICHIER_RESULTATS, "a", newline="") as f:

            redacteur = csv.writer(f)

            if fichier_neuf:
                redacteur.writerow(ENTETE_CSV)

            redacteur.writerow(
                [datetime.now().isoformat(timespec="seconds"), cycle]
                + [round(a, 3) for a in base]
                + [round(d, 3) for d in derives]
            )

    except Exception as e:
        print("Impossible d'ecrire le fichier de resultats :", e)


# =========================================================
# 5. ANALYSE
# =========================================================

def derivesDepuisReference(base, base_reference):
    """Retourne l'ecart signe de chaque axe de base a sa reference."""

    derives = []

    for i in range(3):
        derives.append(base[i] - base_reference[i])

    return derives


def analyserDerive(historique):
    """Retourne (derive finale, derive maximale, derive du dernier tiers) par axe."""

    """
    Le dernier tiers est compare au premier : une base qui
    s'est tassee une fois au demarrage puis ne bouge plus
    n'a pas le meme avenir qu'une base qui glisse encore a
    la fin du test. La premiere se replace, la seconde
    interdit la phase 1 sur ce bras.
    """

    finale = list(historique[-1])
    maximale = []
    tiers = []

    tiers_debut = len(historique) // 3
    tiers_fin = 2 * len(historique) // 3

    for i in range(3):

        valeurs = [derives[i] for derives in historique]
        maximale.append(max(valeurs, key=abs))

        debut = historique[tiers_debut][i]
        fin = historique[-1][i]
        tiers.append(fin - debut)

    return finale, maximale, tiers


def afficherRapport(historique, nb_cycles, tolerance=TOLERANCE_BASE):
    """Affiche le tableau de derive et le verdict sur l'utilisabilite de la base."""

    finale, maximale, tiers = analyserDerive(historique)

    print()
    print("=========================================")
    print("   DERIVE DE LA BASE SUR", nb_cycles, "CYCLES")
    print("=========================================")
    print()
    print("       finale    maximale   2 derniers tiers")

    for i in range(3):
        print(
            "  " + NOMS_ARTICULATIONS[INDICES_BASE[i]]
            + "  " + str(round(finale[i], 3)).rjust(8)
            + "  " + str(round(maximale[i], 3)).rjust(10)
            + "  " + str(round(tiers[i], 3)).rjust(15)
        )

    derive_max = max(abs(d) for d in finale)
    derive_tardive = max(abs(d) for d in tiers)

    print()
    print("Derive finale maximale :", round(derive_max, 3), "degres")
    print("Dont sur les deux derniers tiers :", round(derive_tardive, 3), "degres")

    print()
    print("=========================================")

    if derive_max <= tolerance and derive_tardive <= tolerance / 2:
        verdict = "base stable"
        print("BASE STABLE : elle tient sous l'usage.")
        print("La phase 1 est jouable sur ce bras.")

    elif derive_tardive <= tolerance / 2:
        verdict = "tassement initial"
        print("TASSEMENT INITIAL : la base a bouge puis s'est stabilisee.")
        print("Replacer la base, reenregistrer la reference, et refaire")
        print("ce test : s'il ressort stable, le bras est utilisable.")

    else:
        verdict = "glissement continu"
        print("GLISSEMENT CONTINU : la base bouge encore a la fin du test.")
        print("Aucune calibration de pointage ne tiendra sur ce bras.")
        print("Le garder comme banc d'essai logiciel, pas comme demonstrateur.")

    print("=========================================")

    return {
        "nb_cycles": nb_cycles,
        "derive_finale": [round(d, 3) for d in finale],
        "derive_maximale": [round(d, 3) for d in maximale],
        "derive_deux_derniers_tiers": [round(d, 3) for d in tiers],
        "tolerance": tolerance,
        "verdict": verdict,
    }


# =========================================================
# 6. EXECUTION AUTONOME
# =========================================================

def demanderEntier(message, defaut):
    """Demande un entier a l'operateur, retourne la valeur par defaut si la saisie est vide."""

    saisie = input(message).strip()

    if saisie == "":
        return defaut

    try:
        return int(saisie)

    except ValueError:
        print("Saisie invalide, valeur par defaut retenue :", defaut)
        return defaut


def main():
    """Fait travailler la tete en boucle et mesure la derive de la base."""

    print()
    print("=========================================")
    print("   DERIVE DE LA BASE SOUS L'USAGE")
    print("=========================================")
    print()
    print("ATTENTION : la tete va balayer en continu.")
    print("Laser debranche, espace degage.")
    print()

    # -----------------------------------------------------
    # Reference
    # -----------------------------------------------------

    reference = chargerReference()

    if reference is None:
        print("Aucune reference de base enregistree.")
        print("Lancer d'abord PU_Tete.py et choisir l'option 1.")
        return 1

    print("Reference du robot :", reference["robot"],
          "- enregistree le", reference["date"])
    print("Base de reference  :", reference["base"])
    print()
    print("VERIFIER que c'est bien ce robot qui est devant vous.")

    nb_cycles = demanderEntier(
        "Nombre de cycles [" + str(NB_CYCLES_DEFAUT) + "] : ",
        NB_CYCLES_DEFAUT
    )

    # -----------------------------------------------------
    # Connexion
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    journal = ouvrirJournal("derive_base", mc, {
        "nb_cycles": nb_cycles,
        "reference": reference,
        "AMPLITUDE_J4": AMPLITUDE_J4,
        "AMPLITUDE_J5": AMPLITUDE_J5,
        "TOLERANCE_BASE": TOLERANCE_BASE,
    })

    # -----------------------------------------------------
    # Poses parcourues
    # -----------------------------------------------------

    poses = filtrerPosesValides(
        construirePosesTete(reference["tete"]),
        reference["base"]
    )

    if len(poses) < 2:
        print("Trop peu de poses de tete valides, test impossible.")
        print("Reduire AMPLITUDE_J4 et AMPLITUDE_J5.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "poses de tete invalides"}))
        return 1

    print()
    print(len(poses), "poses de tete parcourues en boucle.")
    print()
    confirmation = input("Lancer le test ? (o/n) : ").strip().lower()

    if confirmation != "o":
        print("Test annule par l'operateur.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "annule par l'operateur"}))
        return 1

    # -----------------------------------------------------
    # Boucle de cycles
    # -----------------------------------------------------

    historique = []

    print()

    for cycle in range(1, nb_cycles + 1):

        tete = poses[(cycle - 1) % len(poses)]

        bougerTete(mc, tete)

        base = lireBase(mc)

        if base is None:
            print("Cycle", cycle, ": lecture impossible, cycle ignore.")
            continue

        derives = derivesDepuisReference(base, reference["base"])
        historique.append(derives)

        enregistrerCycle(cycle, base, derives)

        ajouterMesure(
            journal, "cycle", "cycle " + str(cycle),
            None, lireAngles(mc), None,
            extra={"derive_base": [round(d, 3) for d in derives]}
        )

        if cycle % PERIODE_AFFICHAGE == 0:
            print("Cycle", cycle, "- derive base :",
                  [round(d, 3) for d in derives], "degres")

    # -----------------------------------------------------
    # Rapport
    # -----------------------------------------------------

    if len(historique) < 3:
        print()
        print("Trop peu de cycles exploitables pour conclure.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "moins de trois cycles"}))
        return 1

    resume = afficherRapport(historique, len(historique))

    print()
    print("Resultats ecrits dans :", FICHIER_RESULTATS)

    print()
    print("Qu'a fait le bras ? (claquement, glissement visible, echauffement...)")
    demanderObservation(journal)

    annoncerJournal(fermerJournal(journal, resume))

    print()
    print("Les moteurs restent alimentes.")

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
