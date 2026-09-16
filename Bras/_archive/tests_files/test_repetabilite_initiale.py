"""
=========================================================
ROBOT MYCOBOT 280 - REPETABILITE DE LA POSITION INITIALE
=========================================================

Objectif :
---------
Repondre a deux questions differentes, souvent confondues :

    1. Le bras arrive-t-il ou on lui demande d'aller ?
       C'est la JUSTESSE (le biais consigne / mesure).

    2. Revient-il toujours au meme endroit ?
       C'est la REPETABILITE (la dispersion entre essais).

Un biais constant se rattrape par un decalage dans le
code. Une dispersion, non : elle se retrouve telle quelle
dans l'erreur de pointage du laser. Les deux chiffres sont
donc rapportes separement.

C'est l'etape E1 de la feuille de route : ecart maximal
sous 2 degres sur trois essais, et sous 2 degres apres une
coupure d'alimentation.

Utilisation :
------------
    python3 test_repetabilite_initiale.py

Chaque essai est ecrit dans resultats/repetabilite_initiale.csv.
Relancer le script apres une coupure d'alimentation avec une
autre etiquette de serie : la comparaison entre series est
affichee automatiquement a la fin.

ATTENTION :
----------
Laser debranche, 40 cm degages autour du bras, un seul
programme a la fois sur le port serie.
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
    allerPositionInitiale,
    ecartMaximal,
    ANGLES_POSITION_INITIALE,
    NOMS_ARTICULATIONS
)

from PU_Visee import lireCoords, NOMS_COORDONNEES

from PU_BrasMou import relacherBras, verrouillerBras

from PU_Journal import (
    ouvrirJournal,
    ajouterMesure,
    demanderObservation,
    fermerJournal,
    annoncerJournal
)


# =========================================================
# 2. PARAMETRES DU TEST
# =========================================================

NB_ESSAIS_DEFAUT = 3

# Critere de la feuille de route, etape E1.
TOLERANCE_REPETABILITE = 2.0    # degres

# Bruit de lecture accepte au repos, etape E0.
TOLERANCE_BRUIT_LECTURE = 0.5   # degres

# Nombre de lectures moyennees pour une mesure de pose.
NB_LECTURES = 5
PERIODE_LECTURE = 0.2           # secondes

# Temps laisse aux servos apres la fin du mouvement, avant
# de mesurer. Une mesure prise trop tot lit un bras encore
# en train de se stabiliser.
DELAI_STABILISATION = 2.0       # secondes


# =========================================================
# 3. POSE DE DETOUR
# =========================================================

"""
Pose intermediaire du mode automatique.

Le bras doit repartir de loin entre deux essais, sinon on
mesure la repetabilite d'un bras qui n'a pas bouge. Le
detour est defini comme un decalage de la pose initiale
plutot que comme une pose absolue : il reste valable si
ANGLES_POSITION_INITIALE change.
"""

DECALAGE_DETOUR = [-25.0, -25.0, 25.0, -30.0, 25.0, 30.0]

VITESSE_DETOUR = 20             # int, 1 a 100
TIMEOUT_DETOUR = 15             # secondes


def calculerPoseDetour(angles_initiaux=None, decalage=None):
    """Retourne la pose de detour, ou None si elle sort des limites articulaires."""

    if angles_initiaux is None:
        angles_initiaux = ANGLES_POSITION_INITIALE

    if decalage is None:
        decalage = DECALAGE_DETOUR

    pose = []

    for i in range(6):
        pose.append(float(angles_initiaux[i] + decalage[i]))

    """
    On refuse une pose de detour hors limites, on ne la
    rabote pas : un detour silencieusement deplace ne
    perturbe pas le bras comme prevu et fausse la mesure.
    """

    if not verifierAngles(pose):
        return None

    return pose


# =========================================================
# 4. FICHIER DE RESULTATS
# =========================================================

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")
FICHIER_RESULTATS = os.path.join(DOSSIER_RESULTATS, "repetabilite_initiale.csv")

ENTETE_CSV = (
    ["horodatage", "serie", "essai"]
    + NOMS_ARTICULATIONS
    + NOMS_COORDONNEES
    + ["ecart_max_consigne_deg", "bruit_lecture_deg"]
)


def enregistrerEssai(serie, numero, angles, coords, ecart, bruit):
    """Ajoute une ligne au fichier de resultats, en creant l'entete au premier essai."""

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        fichier_neuf = not os.path.isfile(FICHIER_RESULTATS)

        with open(FICHIER_RESULTATS, "a", newline="") as f:

            redacteur = csv.writer(f)

            if fichier_neuf:
                redacteur.writerow(ENTETE_CSV)

            if coords is None:
                coords = ["", "", "", "", "", ""]

            ligne = (
                [datetime.now().isoformat(timespec="seconds"), serie, numero]
                + [round(a, 2) for a in angles]
                + [c if c == "" else round(c, 2) for c in coords]
                + [round(ecart, 2), round(bruit, 2)]
            )

            redacteur.writerow(ligne)

    except Exception as e:
        print("Impossible d'ecrire le fichier de resultats :", e)
        return False

    return True


# =========================================================
# 5. STATISTIQUES
# =========================================================

def moyenneParArticulation(poses):
    """Retourne la moyenne de chaque articulation sur une liste de poses."""

    moyennes = []

    for i in range(6):

        somme = 0.0
        for pose in poses:
            somme += pose[i]

        moyennes.append(somme / len(poses))

    return moyennes


def etendueParArticulation(poses):
    """Retourne l'etendue (max moins min) de chaque articulation sur une liste de poses."""

    etendues = []

    for i in range(6):

        valeurs = []
        for pose in poses:
            valeurs.append(pose[i])

        etendues.append(max(valeurs) - min(valeurs))

    return etendues


def biaisParArticulation(poses, cible):
    """Retourne l'ecart signe entre la moyenne mesuree et la consigne, articulation par articulation."""

    moyennes = moyenneParArticulation(poses)

    biais = []

    for i in range(6):
        biais.append(moyennes[i] - cible[i])

    return biais


def indiceMaximum(valeurs):
    """Retourne l'indice de la plus grande valeur absolue de la liste."""

    indice = 0

    for i in range(len(valeurs)):
        if abs(valeurs[i]) > abs(valeurs[indice]):
            indice = i

    return indice


# =========================================================
# 6. MESURE D'UNE POSE
# =========================================================

def mesurerPoseDetaillee(mc, nb_lectures=NB_LECTURES, periode=PERIODE_LECTURE):
    """Moyenne plusieurs lectures et retourne (pose moyenne, bruit maximal, lectures brutes)."""

    lectures = []

    for _ in range(nb_lectures):

        angles = lireAngles(mc)

        if angles is not None:
            lectures.append(angles)

        time.sleep(periode)

    if len(lectures) == 0:
        return None, None, []

    moyenne = moyenneParArticulation(lectures)

    """
    L'etendue des lectures d'un bras immobile est le bruit
    du capteur. C'est le plancher de tout ce qui sera
    mesure ensuite : inutile de chercher une dispersion
    plus fine que lui.
    """

    bruit = max(etendueParArticulation(lectures))

    return moyenne, bruit, lectures


def mesurerPose(mc, nb_lectures=NB_LECTURES, periode=PERIODE_LECTURE):
    """Moyenne plusieurs lectures et retourne (pose moyenne, bruit maximal de lecture)."""

    moyenne, bruit, _ = mesurerPoseDetaillee(mc, nb_lectures, periode)

    return moyenne, bruit


# =========================================================
# 7. ETAPE E0 - COMMUNICATION
# =========================================================

def verifierCommunication(mc, journal=None):
    """Verifie que le robot renvoie une pose stable au repos (etape E0)."""

    print()
    print("-----------------------------------------")
    print("E0 - Communication et bruit de lecture")
    print("-----------------------------------------")

    pose, bruit, lectures = mesurerPoseDetaillee(mc)

    if pose is None:
        print("Aucune lecture exploitable : port, baudrate ou carte Atom.")
        return False

    coords = lireCoords(mc)

    ajouterMesure(
        journal, "communication", "lecture au repos",
        None, pose, lectures,
        extra={"bruit_lecture": round(bruit, 3), "coords": coords}
    )

    print("Angles  :", [round(a, 2) for a in pose])

    if coords is None:
        print("Coords  : lecture impossible dans cette pose.")
    else:
        print("Coords  :", [round(c, 2) for c in coords])

    print("Bruit de lecture :", round(bruit, 2), "degres")

    if bruit > TOLERANCE_BRUIT_LECTURE:
        print("ATTENTION : bruit superieur a", TOLERANCE_BRUIT_LECTURE, "degres.")
        print("Toute dispersion mesuree ensuite sera noyee dedans.")
        return False

    print("Communication correcte.")
    return True


# =========================================================
# 8. PERTURBATION ENTRE DEUX ESSAIS
# =========================================================

def perturberAutomatique(mc):
    """Envoie le bras sur la pose de detour pour qu'il reparte de loin."""

    pose = calculerPoseDetour()

    if pose is None:
        print("La pose de detour sort des limites articulaires, perturbation annulee.")
        print("Corriger DECALAGE_DETOUR dans ce fichier.")
        return False

    print("Detour par :", [round(a, 1) for a in pose])

    try:
        mc.sync_send_angles(pose, VITESSE_DETOUR, TIMEOUT_DETOUR)

    except Exception as e:
        print("Erreur pendant le detour :", e)
        return False

    time.sleep(1)
    return True


def perturberManuel(mc):
    """Relache le bras, laisse l'operateur le deplacer a la main, puis reverrouille."""

    print()
    print("Le bras va devenir mou et tomber s'il n'est pas tenu.")
    confirmation = input("Le bras est-il tenu ? (o/n) : ").strip().lower()

    if confirmation != "o":
        print("Perturbation annulee.")
        return False

    if not relacherBras(mc):
        return False

    print()
    print("Deplacer le bras franchement, loin de la pose initiale.")
    input("Appuyer sur entree quand le bras est en place et tenu : ")

    return verrouillerBras(mc)


# =========================================================
# 9. DEROULEMENT D'UN ESSAI
# =========================================================

def executerEssai(mc, numero, serie, perturber, journal=None):
    """Perturbe le bras, le renvoie en position initiale et retourne la pose mesuree."""

    print()
    print("=========================================")
    print("Essai", numero)
    print("=========================================")

    # -----------------------------------------------------
    # Perturbation
    # -----------------------------------------------------

    """
    Sans perturbation prealable on mesure la repetabilite
    d'un bras qui n'a pas bouge, c'est-a-dire rien. Le
    premier essai fait exception : le bras arrive dans une
    pose quelconque, elle joue ce role.
    """

    if numero > 1:
        if not perturber(mc):
            print("Perturbation echouee, essai abandonne.")
            return None

    # -----------------------------------------------------
    # Retour en position initiale
    # -----------------------------------------------------

    print()

    if not allerPositionInitiale(mc):
        print()
        print("Le retour en position initiale a echoue.")
        print("L'essai est tout de meme mesure : l'ecart est l'information utile.")

    # -----------------------------------------------------
    # Mesure
    # -----------------------------------------------------

    time.sleep(DELAI_STABILISATION)

    pose, bruit, lectures = mesurerPoseDetaillee(mc)

    if pose is None:
        print("Mesure impossible, essai abandonne.")
        return None

    coords = lireCoords(mc)
    ecart = ecartMaximal(pose, ANGLES_POSITION_INITIALE)

    print()
    print("Pose mesuree     :", [round(a, 2) for a in pose])
    print("Ecart a la consigne :", round(ecart, 2), "degres")
    print("Bruit de lecture    :", round(bruit, 2), "degres")

    enregistrerEssai(serie, numero, pose, coords, ecart, bruit)

    ajouterMesure(
        journal, "essai", "essai " + str(numero),
        ANGLES_POSITION_INITIALE, pose, lectures,
        extra={
            "serie": serie,
            "bruit_lecture": round(bruit, 3),
            "coords": coords,
            "ecart_max_consigne": round(ecart, 3),
        }
    )

    return pose


# =========================================================
# 10. RAPPORT
# =========================================================

def afficherRapport(poses, cible=None):
    """Affiche le tableau biais / dispersion et le verdict de l'etape E1."""

    if cible is None:
        cible = ANGLES_POSITION_INITIALE

    biais = biaisParArticulation(poses, cible)
    etendues = etendueParArticulation(poses)
    moyennes = moyenneParArticulation(poses)

    print()
    print("=========================================")
    print("   RAPPORT SUR", len(poses), "ESSAIS")
    print("=========================================")
    print()
    print("        consigne   mesure    biais   dispersion")

    for i in range(6):
        print(
            "  " + NOMS_ARTICULATIONS[i]
            + "  " + str(round(cible[i], 1)).rjust(8)
            + "  " + str(round(moyennes[i], 2)).rjust(8)
            + "  " + str(round(biais[i], 2)).rjust(7)
            + "  " + str(round(etendues[i], 2)).rjust(11)
        )

    # -----------------------------------------------------
    # Repetabilite
    # -----------------------------------------------------

    """
    La dispersion est le chiffre qui compte pour la suite :
    elle se retrouve integralement dans l'erreur de
    pointage du laser et rien ne la rattrape apres coup.
    """

    indice_dispersion = indiceMaximum(etendues)
    dispersion_max = etendues[indice_dispersion]

    print()
    print("REPETABILITE : dispersion maximale",
          round(dispersion_max, 2), "degres sur",
          NOMS_ARTICULATIONS[indice_dispersion])

    if dispersion_max <= TOLERANCE_REPETABILITE:
        print("  -> conforme au critere E1 (", TOLERANCE_REPETABILITE, "degres )")
        repetable = True
    else:
        print("  -> HORS CRITERE E1 (", TOLERANCE_REPETABILITE, "degres )")
        repetable = False

    # -----------------------------------------------------
    # Justesse
    # -----------------------------------------------------

    indice_biais = indiceMaximum(biais)
    biais_max = biais[indice_biais]

    print()
    print("JUSTESSE : biais maximal",
          round(biais_max, 2), "degres sur",
          NOMS_ARTICULATIONS[indice_biais])

    if abs(biais_max) <= TOLERANCE_REPETABILITE:
        print("  -> le bras arrive ou on lui demande.")
        juste = True
    else:
        print("  -> le bras s'arrete systematiquement a cote de la consigne.")
        print("     Un biais repetable n'interdit pas la suite : il se corrige")
        print("     dans le code. Mais il faut d'abord en connaitre la cause,")
        print("     sinon il changera de valeur sans prevenir.")
        juste = False

    print()
    print("=========================================")

    if repetable and juste:
        print("E1 VALIDE sur cette serie.")
    elif repetable:
        print("Bras repetable mais decale : diagnostiquer le biais.")
    else:
        print("E1 NON VALIDE : le bras ne revient pas au meme endroit.")

    print("=========================================")

    return {
        "nb_essais": len(poses),
        "consigne": list(cible),
        "moyenne_mesuree": [round(a, 3) for a in moyennes],
        "biais_par_articulation": [round(b, 3) for b in biais],
        "dispersion_par_articulation": [round(e, 3) for e in etendues],
        "dispersion_max": round(dispersion_max, 3),
        "articulation_dispersion_max": NOMS_ARTICULATIONS[indice_dispersion],
        "biais_max": round(biais_max, 3),
        "articulation_biais_max": NOMS_ARTICULATIONS[indice_biais],
        "repetable": repetable,
        "juste": juste,
        "e1_valide": repetable and juste,
    }


# =========================================================
# 11. COMPARAISON ENTRE SERIES
# =========================================================

def comparerSeries():
    """Compare les poses moyennes des series deja enregistrees, coupure d'alimentation comprise."""

    if not os.path.isfile(FICHIER_RESULTATS):
        return

    try:
        with open(FICHIER_RESULTATS, "r", newline="") as f:
            lignes = list(csv.DictReader(f))

    except Exception as e:
        print("Impossible de relire le fichier de resultats :", e)
        return

    # -----------------------------------------------------
    # Regroupement par serie
    # -----------------------------------------------------

    series = {}

    for ligne in lignes:

        nom = ligne["serie"]

        if nom not in series:
            series[nom] = []

        try:
            pose = [float(ligne[nom_articulation]) for nom_articulation in NOMS_ARTICULATIONS]
        except (ValueError, KeyError):
            continue

        series[nom].append(pose)

    if len(series) < 2:
        return

    # -----------------------------------------------------
    # Pose moyenne de chaque serie
    # -----------------------------------------------------

    print()
    print("=========================================")
    print("   COMPARAISON DES SERIES ENREGISTREES")
    print("=========================================")
    print()

    moyennes_series = {}

    for nom in series:

        moyenne = moyenneParArticulation(series[nom])
        moyennes_series[nom] = moyenne

        print(nom, "(", len(series[nom]), "essais ) :",
              [round(a, 2) for a in moyenne])

    # -----------------------------------------------------
    # Ecart entre series
    # -----------------------------------------------------

    """
    C'est le test de la coupure d'alimentation : si la pose
    moyenne change d'une serie a l'autre, le bras ne
    retrouve pas sa reference apres extinction et toute
    calibration ulterieure meurt a chaque redemarrage.
    """

    poses_moyennes = list(moyennes_series.values())
    etendues = etendueParArticulation(poses_moyennes)

    indice = indiceMaximum(etendues)

    print()
    print("Ecart maximal entre series :",
          round(etendues[indice], 2), "degres sur",
          NOMS_ARTICULATIONS[indice])

    if etendues[indice] <= TOLERANCE_REPETABILITE:
        print("  -> la pose de reference survit d'une serie a l'autre.")
    else:
        print("  -> la pose de reference NE survit PAS d'une serie a l'autre.")
        print("     Une calibration faite dans une serie sera fausse dans la suivante.")


# =========================================================
# 12. EXECUTION AUTONOME
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
    """Deroule la serie d'essais de repetabilite et affiche le rapport."""

    print()
    print("=========================================")
    print("   REPETABILITE DE LA POSITION INITIALE")
    print("=========================================")
    print()
    print("ATTENTION : laser debranche, 40 cm degages.")
    print()
    print("Position initiale visee :", ANGLES_POSITION_INITIALE)
    print()

    # -----------------------------------------------------
    # Parametres de la serie
    # -----------------------------------------------------

    serie = input("Etiquette de la serie (ex : avant_coupure) : ").strip()

    if serie == "":
        serie = "serie_" + datetime.now().strftime("%Y%m%d_%H%M")

    nb_essais = demanderEntier(
        "Nombre d'essais [" + str(NB_ESSAIS_DEFAUT) + "] : ",
        NB_ESSAIS_DEFAUT
    )

    print()
    print("Perturbation entre deux essais :")
    print("  1 - a la main (protocole E1 : bras relache, deplace, reverrouille)")
    print("  2 - detour automatique par une pose connue")

    choix = input("Choix [1] : ").strip()

    if choix == "2":
        perturber = perturberAutomatique
        print("Mode detour automatique.")
    else:
        perturber = perturberManuel
        print("Mode manuel.")

    # -----------------------------------------------------
    # Connexion
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    # -----------------------------------------------------
    # Journal de session
    # -----------------------------------------------------

    """
    Les reglages du script sont recopies dans le journal :
    une mesure surprenante ne se diagnostique pas sans
    savoir a quelle vitesse et avec quel timeout elle a ete
    prise.
    """

    journal = ouvrirJournal("repetabilite_initiale", mc, {
        "serie": serie,
        "nb_essais": nb_essais,
        "perturbation": "manuelle" if perturber is perturberManuel else "detour automatique",
        "TOLERANCE_REPETABILITE": TOLERANCE_REPETABILITE,
        "TOLERANCE_BRUIT_LECTURE": TOLERANCE_BRUIT_LECTURE,
        "NB_LECTURES": NB_LECTURES,
        "DELAI_STABILISATION": DELAI_STABILISATION,
        "VITESSE_DETOUR": VITESSE_DETOUR,
        "DECALAGE_DETOUR": DECALAGE_DETOUR,
    })

    # -----------------------------------------------------
    # E0 avant tout
    # -----------------------------------------------------

    """
    Une dispersion mesuree sur une liaison qui bafouille ne
    veut rien dire. On verifie la lecture avant de bouger
    quoi que ce soit.
    """

    if not verifierCommunication(mc, journal):
        print()
        print("Communication insuffisante, essais annules.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "communication insuffisante"}))
        return 1

    print()
    confirmation = input("Lancer la serie d'essais ? (o/n) : ").strip().lower()

    if confirmation != "o":
        print("Serie annulee par l'operateur.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "annule par l'operateur"}))
        return 1

    # -----------------------------------------------------
    # Essais
    # -----------------------------------------------------

    poses = []

    for numero in range(1, nb_essais + 1):

        pose = executerEssai(mc, numero, serie, perturber, journal)

        if pose is not None:
            poses.append(pose)

    # -----------------------------------------------------
    # Rapport
    # -----------------------------------------------------

    if len(poses) < 2:
        print()
        print("Moins de deux essais exploitables : aucune dispersion calculable.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "moins de deux essais exploitables"}))
        return 1

    resume = afficherRapport(poses)

    print()
    print("Resultats ecrits dans :", FICHIER_RESULTATS)

    comparerSeries()

    # -----------------------------------------------------
    # Journal
    # -----------------------------------------------------

    """
    L'observation de l'operateur vaut souvent la mesure :
    un bras qui vibre, qui force ou qui claque ne laisse
    aucune trace dans les angles lus.
    """

    print()
    print("Qu'a fait le bras pendant la serie ? (bruit, a-coup, blocage...)")
    demanderObservation(journal)

    annoncerJournal(fermerJournal(journal, resume))

    """
    Les moteurs restent alimentes : un power_off relacherait
    les servos et le bras retomberait, ce qui detruirait la
    pose qu'on vient de mesurer.
    """

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
