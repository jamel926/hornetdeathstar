"""
=========================================================
ROBOT MYCOBOT 280 - DIAGNOSTIC DE L'ECART SUR J3
=========================================================

Constat de depart :
------------------
J3 s'arrete a une vingtaine de degres de la consigne. Un
ecart de cette taille ne se corrige pas au jugé : selon sa
cause, la correction est un decalage dans le code, un
changement de pose de reference, ou une reparation.

Ce script ne corrige rien. Il distingue les causes.

Les cinq causes possibles :
--------------------------

    A. BUTEE OU COLLISION
       Le bras touche quelque chose, ou le firmware refuse
       d'aller plus loin. La mesure sature : au-dela d'une
       certaine consigne, elle ne bouge plus.

    B. DECALAGE D'ORIGINE (encodeur mal calibre)
       L'ecart est le meme partout, quelle que soit la
       consigne. Le bras est bien la ou on croit qu'il est,
       c'est le zero du capteur qui a bouge.

    C. AFFAISSEMENT SOUS LA CHARGE
       L'articulation ne tient pas le couple. L'ecart
       grandit quand l'avant-bras est plus tendu, donc
       quand J2 change, et se referme un peu si on renvoie
       la meme consigne.

    D. JEU MECANIQUE
       L'ecart depend du sens d'arrivee : atteindre -130 en
       venant de -100 ne donne pas la meme mesure qu'en
       venant de -150.

    E. MOUVEMENT INACHEVE
       sync_send_angles a rendu la main avant l'arrivee. Il
       retourne 1 meme sur timeout. L'ecart se referme si
       on attend, ou si on renvoie la consigne.

Utilisation :
------------
    python3 test_diagnostic_J3.py

Les mesures sont ecrites dans resultats/diagnostic_J3.csv.

=========================================================
ATTENTION
=========================================================

Ce script fait balayer J3 sur une large plage. Degager
50 cm autour du bras, laser debranche, et garder la main
pres de l'arret d'urgence pour les premiers passages.

Le mode pas a pas s'arrete avant chaque mouvement : le
choisir tant qu'on ne sait pas ou le bras va passer.
=========================================================
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
    ecartMaximal,
    ANGLES_POSITION_INITIALE,
    NOMS_ARTICULATIONS
)

from PU_BrasMou import relacherBras, verrouillerBras, formaterAngles

from test_repetabilite_initiale import mesurerPoseDetaillee, etendueParArticulation

from PU_Journal import (
    ouvrirJournal,
    ajouterMesure,
    demanderObservation,
    fermerJournal,
    annoncerJournal
)


# =========================================================
# 2. PARAMETRES DU DIAGNOSTIC
# =========================================================

# Indice de J3 dans les listes d'angles.
J3 = 2

VITESSE_DIAGNOSTIC = 20     # int, 1 a 100
TIMEOUT_DIAGNOSTIC = 20     # secondes

# Temps laisse au bras apres la fin annoncee du mouvement.
DELAI_STABILISATION = 2.0   # secondes

# Ecart a partir duquel on considere que la consigne n'est
# pas suivie. C'est la tolerance de l'etape E1.
TOLERANCE = 2.0             # degres


# =========================================================
# 3. POSE D'ESSAI
# =========================================================

"""
Pose dans laquelle J3 est balaye.

Les cinq autres axes sont mis a zero : le bras est deplie
et vertical, J3 balaye donc dans un plan degage, loin de la
table et de la base. On ne balaye PAS depuis la pose
initiale, ou le bras est replie vers l'avant : c'est
justement la que la collision est possible, et une
collision ferait passer la cause A pour une cause B.
"""

POSE_ESSAI = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Consignes de J3 balayees, de la plus sure a la plus
# engagee. L'ordre compte : on s'approche progressivement
# de la valeur qui pose probleme.
CONSIGNES_J3 = [-60.0, -80.0, -100.0, -120.0, -130.0, -140.0, -144.0, -150.0]

# Valeur de J3 utilisee pour les tests d'hysteresis et de
# charge : au milieu de la plage suspecte.
CONSIGNE_TEMOIN = -130.0

# Valeurs de J2 comparees dans le test de charge. Plus
# l'epaule est basse, plus l'avant-bras pese sur le coude.
VALEURS_J2_CHARGE = [0.0, 48.0]


# =========================================================
# 4. FICHIER DE RESULTATS
# =========================================================

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")
FICHIER_RESULTATS = os.path.join(DOSSIER_RESULTATS, "diagnostic_J3.csv")

ENTETE_CSV = [
    "horodatage", "test", "detail",
    "consigne_J3", "mesure_J3", "ecart_J3",
    "consigne_J2", "mesure_J2"
]


def enregistrerMesure(test, detail, consigne, mesure):
    """Ajoute une ligne au fichier de resultats du diagnostic."""

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        fichier_neuf = not os.path.isfile(FICHIER_RESULTATS)

        with open(FICHIER_RESULTATS, "a", newline="") as f:

            redacteur = csv.writer(f)

            if fichier_neuf:
                redacteur.writerow(ENTETE_CSV)

            redacteur.writerow([
                datetime.now().isoformat(timespec="seconds"),
                test,
                detail,
                round(consigne[J3], 2),
                round(mesure[J3], 2),
                round(mesure[J3] - consigne[J3], 2),
                round(consigne[1], 2),
                round(mesure[1], 2)
            ])

    except Exception as e:
        print("Impossible d'ecrire le fichier de resultats :", e)


# =========================================================
# 5. DEPLACEMENT MESURE
# =========================================================

def envoyerPose(mc, consigne, pas_a_pas=False, journal=None,
                categorie="mouvement", detail=""):
    """Envoie une pose, attend la stabilisation et retourne la pose reellement mesuree."""

    if not verifierAngles(consigne):
        print("Consigne hors limites, mouvement annule.")
        return None

    print()
    print("Consigne :", formaterAngles(consigne))

    if pas_a_pas:
        reponse = input("Lancer ce mouvement ? (o/n) : ").strip().lower()
        if reponse != "o":
            print("Mouvement passe.")
            return None

    try:

        if mc.is_moving() == 1:
            mc.stop()
            time.sleep(0.5)

        mc.sync_send_angles(
            [float(a) for a in consigne],
            int(VITESSE_DIAGNOSTIC),
            TIMEOUT_DIAGNOSTIC
        )

    except Exception as e:
        print("Erreur pendant le mouvement :", e)
        return None

    time.sleep(DELAI_STABILISATION)

    mesure, bruit, lectures = mesurerPoseDetaillee(mc)

    if mesure is None:
        print("Mesure impossible.")
        return None

    ajouterMesure(
        journal, categorie, detail, consigne, mesure, lectures,
        extra={"bruit_lecture": round(bruit, 3)}
    )

    ecart_j3 = mesure[J3] - consigne[J3]

    print("Mesure   :", formaterAngles(mesure))
    print("Ecart J3 :", round(ecart_j3, 2), "degres",
          "   ecart max tous axes :", round(ecartMaximal(mesure, consigne), 2))

    return mesure


def poseAvecJ3(valeur_j3, valeur_j2=None):
    """Retourne la pose d'essai avec J3 (et eventuellement J2) remplaces."""

    pose = list(POSE_ESSAI)
    pose[J3] = float(valeur_j3)

    if valeur_j2 is not None:
        pose[1] = float(valeur_j2)

    return pose


# =========================================================
# 6. TEST 1 - BALAYAGE DE J3
# =========================================================

def testBalayage(mc, pas_a_pas, journal=None):
    """Balaye J3 sur toute la plage suspecte et retourne la liste (consigne, mesure)."""

    print()
    print("=========================================")
    print(" TEST 1 - BALAYAGE DE J3")
    print("=========================================")
    print()
    print("Distingue une butee (la mesure sature) d'un")
    print("decalage d'origine (l'ecart est constant) et")
    print("d'un affaissement (l'ecart grandit).")

    resultats = []

    for consigne_j3 in CONSIGNES_J3:

        consigne = poseAvecJ3(consigne_j3)
        mesure = envoyerPose(mc, consigne, pas_a_pas, journal,
                             "balayage", "J3 = " + str(consigne_j3))

        if mesure is None:
            continue

        enregistrerMesure("balayage", "", consigne, mesure)
        resultats.append((consigne_j3, mesure[J3]))

        """
        Si le bras cesse d'avancer alors que la consigne
        continue de descendre, insister ne sert qu'a forcer
        sur une butee. On previent, l'operateur decide.
        """

        if len(resultats) >= 2:

            progression = abs(resultats[-1][1] - resultats[-2][1])
            demande = abs(resultats[-1][0] - resultats[-2][0])

            if demande > 1.0 and progression < 0.2 * demande:
                print()
                print("ATTENTION : la mesure ne suit plus la consigne.")
                print("Le bras est probablement en butee ou contre un obstacle.")
                reponse = input("Continuer le balayage ? (o/n) : ").strip().lower()
                if reponse != "o":
                    break

    return resultats


# =========================================================
# 7. TEST 2 - SENS D'APPROCHE
# =========================================================

def testHysteresis(mc, pas_a_pas, journal=None):
    """Atteint la meme consigne par le haut puis par le bas et retourne les deux mesures."""

    print()
    print("=========================================")
    print(" TEST 2 - SENS D'APPROCHE")
    print("=========================================")
    print()
    print("Meme consigne atteinte dans les deux sens. Une")
    print("difference entre les deux est du jeu mecanique.")

    mesures = {}

    # -----------------------------------------------------
    # Approche par le haut
    # -----------------------------------------------------

    envoyerPose(mc, poseAvecJ3(-60.0), pas_a_pas, journal,
                "hysteresis", "mise en place par le haut")
    mesure = envoyerPose(mc, poseAvecJ3(CONSIGNE_TEMOIN), pas_a_pas, journal,
                         "hysteresis", "arrivee depuis -60")

    if mesure is not None:
        enregistrerMesure("hysteresis", "depuis -60", poseAvecJ3(CONSIGNE_TEMOIN), mesure)
        mesures["haut"] = mesure[J3]

    # -----------------------------------------------------
    # Approche par le bas
    # -----------------------------------------------------

    envoyerPose(mc, poseAvecJ3(-150.0), pas_a_pas, journal,
                "hysteresis", "mise en place par le bas")
    mesure = envoyerPose(mc, poseAvecJ3(CONSIGNE_TEMOIN), pas_a_pas, journal,
                         "hysteresis", "arrivee depuis -150")

    if mesure is not None:
        enregistrerMesure("hysteresis", "depuis -150", poseAvecJ3(CONSIGNE_TEMOIN), mesure)
        mesures["bas"] = mesure[J3]

    return mesures


# =========================================================
# 8. TEST 3 - CONSIGNE RENVOYEE
# =========================================================

def testInsistance(mc, pas_a_pas, journal=None, nb_renvois=3):
    """Renvoie plusieurs fois la meme consigne et retourne la suite des mesures."""

    print()
    print("=========================================")
    print(" TEST 3 - CONSIGNE RENVOYEE")
    print("=========================================")
    print()
    print("La meme consigne, plusieurs fois de suite. Si")
    print("l'ecart se referme, le mouvement etait inacheve")
    print("ou l'articulation flechit sous la charge. S'il ne")
    print("bouge pas, c'est une butee ou un decalage.")

    consigne = poseAvecJ3(CONSIGNE_TEMOIN)
    mesures = []

    for i in range(nb_renvois):

        mesure = envoyerPose(mc, consigne, pas_a_pas, journal,
                             "insistance", "envoi " + str(i + 1))

        if mesure is None:
            continue

        enregistrerMesure("insistance", "envoi " + str(i + 1), consigne, mesure)
        mesures.append(mesure[J3])

    return mesures


# =========================================================
# 9. TEST 4 - INFLUENCE DE LA CHARGE
# =========================================================

def testCharge(mc, pas_a_pas, journal=None):
    """Mesure le meme J3 pour deux positions d'epaule et retourne les ecarts obtenus."""

    print()
    print("=========================================")
    print(" TEST 4 - INFLUENCE DE LA CHARGE")
    print("=========================================")
    print()
    print("Meme consigne de J3, deux positions de J2. Le")
    print("couple sur le coude change. Si l'ecart change")
    print("avec lui, l'articulation flechit sous la charge.")

    ecarts = {}

    for valeur_j2 in VALEURS_J2_CHARGE:

        consigne = poseAvecJ3(CONSIGNE_TEMOIN, valeur_j2)
        mesure = envoyerPose(mc, consigne, pas_a_pas, journal,
                             "charge", "J2 = " + str(valeur_j2))

        if mesure is None:
            continue

        enregistrerMesure("charge", "J2 = " + str(valeur_j2), consigne, mesure)
        ecarts[valeur_j2] = mesure[J3] - CONSIGNE_TEMOIN

    return ecarts


# =========================================================
# 10. TEST 5 - LECTURE SANS COUPLE
# =========================================================

def testSansCouple(mc, journal=None):
    """Compare la pose lue bras relache et bras verrouille sur cette meme pose."""

    print()
    print("=========================================")
    print(" TEST 5 - LECTURE BRAS RELACHE")
    print("=========================================")
    print()
    print("Le bras est relache, place a la main, et sa pose")
    print("est lue. On lui redemande ensuite cette pose")
    print("exacte. S'il s'en ecarte, il ne tient pas la")
    print("position : ce n'est ni la consigne ni le capteur.")
    print()
    print("Profiter du bras relache pour verifier l'angle du")
    print("coude au rapporteur : si la mesure du robot ne")
    print("correspond pas au bras reel, le capteur est")
    print("decale et tout le reste en decoule.")

    print()
    print("Le bras va devenir mou et tomber s'il n'est pas tenu.")
    confirmation = input("Le bras est-il tenu ? (o/n) : ").strip().lower()

    if confirmation != "o":
        print("Test annule.")
        return None

    if not relacherBras(mc):
        return None

    print()
    print("Placer le coude a une vingtaine de degres de la")
    print("butee, bras toujours tenu.")
    input("Appuyer sur entree quand c'est fait : ")

    pose_main, _, lectures_main = mesurerPoseDetaillee(mc)

    if pose_main is None:
        print("Lecture impossible.")
        return None

    ajouterMesure(journal, "sans_couple", "pose lue bras relache",
                  None, pose_main, lectures_main)

    print()
    print("Pose lue bras relache :", formaterAngles(pose_main))

    if not verrouillerBras(mc):
        return None

    time.sleep(DELAI_STABILISATION)

    pose_tenue, _, lectures_tenue = mesurerPoseDetaillee(mc)

    if pose_tenue is None:
        return None

    ecart = pose_tenue[J3] - pose_main[J3]

    print("Pose apres verrouillage :", formaterAngles(pose_tenue))
    print("Deplacement de J3 pendant le verrouillage :", round(ecart, 2), "degres")

    enregistrerMesure("sans_couple", "verrouillage sur pose lue", pose_main, pose_tenue)

    ajouterMesure(journal, "sans_couple", "pose tenue apres verrouillage",
                  pose_main, pose_tenue, lectures_tenue)

    return ecart


# =========================================================
# 11. VERDICT
# =========================================================

def analyserBalayage(resultats):
    """Retourne (ecart moyen, variation de l'ecart, saturation constatee) a partir du balayage."""

    ecarts = []

    for consigne, mesure in resultats:
        ecarts.append(mesure - consigne)

    ecart_moyen = sum(ecarts) / len(ecarts)
    variation = max(ecarts) - min(ecarts)

    """
    Saturation : les dernieres consignes descendent, la
    mesure non. On compare la course demandee a la course
    reellement parcourue sur la seconde moitie du balayage.
    """

    milieu = len(resultats) // 2

    course_demandee = abs(resultats[-1][0] - resultats[milieu][0])
    course_reelle = abs(resultats[-1][1] - resultats[milieu][1])

    sature = course_demandee > 1.0 and course_reelle < 0.5 * course_demandee

    return ecart_moyen, variation, sature


def afficherVerdict(balayage, hysteresis, insistance, charge, sans_couple):
    """Recoupe les tests effectues et designe la cause la plus probable."""

    print()
    print("=========================================")
    print("   VERDICT")
    print("=========================================")

    indices = []

    # -----------------------------------------------------
    # Balayage
    # -----------------------------------------------------

    if balayage is not None and len(balayage) >= 3:

        ecart_moyen, variation, sature = analyserBalayage(balayage)

        print()
        print("Balayage :", len(balayage), "points")
        print("  ecart moyen        :", round(ecart_moyen, 2), "degres")
        print("  variation de l'ecart :", round(variation, 2), "degres")
        print("  saturation          :", "oui" if sature else "non")

        if sature:
            print()
            print("  -> BUTEE OU COLLISION (cause A).")
            print("     Le bras cesse d'avancer alors que la consigne continue.")
            print("     Chercher l'obstacle, et verifier que la pose initiale")
            print("     n'est pas simplement inatteignable sous son propre poids.")
            indices.append("A")

        elif variation < TOLERANCE:
            print()
            print("  -> DECALAGE D'ORIGINE (cause B).")
            print("     L'ecart est le meme sur toute la plage :", round(ecart_moyen, 1), "degres.")
            print("     Le bras suit fidelement, c'est la reference du capteur")
            print("     qui est decalee. Verifier au rapporteur avant de conclure.")
            indices.append("B")

        else:
            print()
            print("  -> ecart variable sur la plage : ni butee franche, ni")
            print("     decalage constant. Regarder les tests de charge et de sens.")

    # -----------------------------------------------------
    # Insistance
    # -----------------------------------------------------

    if insistance is not None and len(insistance) >= 2:

        progres = abs(insistance[-1] - insistance[0])

        print()
        print("Consigne renvoyee :", [round(m, 2) for m in insistance])

        if progres > TOLERANCE:
            print("  -> MOUVEMENT INACHEVE ou AFFAISSEMENT (causes E ou C).")
            print("     L'ecart se referme quand on insiste :", round(progres, 2), "degres gagnes.")
            print("     Augmenter TIMEOUT et baisser la vitesse avant tout le reste.")
            indices.append("E")
        else:
            print("  -> insister ne change rien : le mouvement etait bien termine.")

    # -----------------------------------------------------
    # Charge
    # -----------------------------------------------------

    if charge is not None and len(charge) >= 2:

        valeurs = list(charge.values())
        difference = max(valeurs) - min(valeurs)

        print()
        print("Ecart selon J2 :")
        for valeur_j2 in charge:
            print("  J2 =", valeur_j2, "-> ecart J3 =", round(charge[valeur_j2], 2))

        if difference > TOLERANCE:
            print("  -> AFFAISSEMENT SOUS LA CHARGE (cause C).")
            print("     L'ecart depend de la position de l'epaule :", round(difference, 2), "degres.")
            print("     Le coude ne tient pas le couple. Une pose initiale plus")
            print("     repliee reduira la charge ; sinon la compensation devra")
            print("     dependre de la pose, pas etre une constante.")
            indices.append("C")
        else:
            print("  -> l'ecart ne depend pas de la charge.")

    # -----------------------------------------------------
    # Hysteresis
    # -----------------------------------------------------

    if hysteresis is not None and len(hysteresis) == 2:

        difference = abs(hysteresis["haut"] - hysteresis["bas"])

        print()
        print("Sens d'approche : par le haut", round(hysteresis["haut"], 2),
              " par le bas", round(hysteresis["bas"], 2))

        if difference > TOLERANCE:
            print("  -> JEU MECANIQUE (cause D) :", round(difference, 2), "degres.")
            print("     Toujours approcher la pose de reference dans le meme sens.")
            indices.append("D")
        else:
            print("  -> pas de jeu mesurable.")

    # -----------------------------------------------------
    # Lecture sans couple
    # -----------------------------------------------------

    if sans_couple is not None:

        print()
        print("Verrouillage sur la pose lue : deplacement de", round(sans_couple, 2), "degres")

        if abs(sans_couple) > TOLERANCE:
            print("  -> le bras ne tient pas une pose qu'il vient pourtant de lire.")
            print("     C'est le servo, pas la consigne ni le capteur (cause C).")
            indices.append("C")
        else:
            print("  -> le bras tient la pose lue.")

    # -----------------------------------------------------
    # Synthese
    # -----------------------------------------------------

    print()
    print("-----------------------------------------")

    if len(indices) == 0:
        print("Aucune cause franche. Relancer avec tous les tests,")
        print("ou l'ecart de 20 degres n'apparait pas dans cette pose")
        print("d'essai et est donc propre a la pose initiale repliee.")
    else:
        print("Causes retenues :", ", ".join(sorted(set(indices))))

    print("-----------------------------------------")

    return {
        "causes_retenues": sorted(set(indices)),
        "balayage": balayage,
        "hysteresis": hysteresis,
        "insistance": insistance,
        "charge": {str(cle): round(charge[cle], 3) for cle in charge} if charge else None,
        "sans_couple": round(sans_couple, 3) if sans_couple is not None else None,
    }


# =========================================================
# 12. EXECUTION AUTONOME
# =========================================================

def main():
    """Deroule les tests de diagnostic de J3 et affiche le verdict."""

    print()
    print("=========================================")
    print("   DIAGNOSTIC DE L'ECART SUR J3")
    print("=========================================")
    print()
    print("ATTENTION : 50 cm degages, laser debranche.")
    print("J3 va balayer de", CONSIGNES_J3[0], "a", CONSIGNES_J3[-1], "degres")
    print("depuis la pose", POSE_ESSAI, ".")
    print()
    print("Pose initiale du projet :", ANGLES_POSITION_INITIALE)
    print()

    reponse = input("Mode pas a pas (confirmation avant chaque mouvement) ? (o/n) [o] : ").strip().lower()
    pas_a_pas = (reponse != "n")

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    journal = ouvrirJournal("diagnostic_J3", mc, {
        "POSE_ESSAI": POSE_ESSAI,
        "CONSIGNES_J3": CONSIGNES_J3,
        "CONSIGNE_TEMOIN": CONSIGNE_TEMOIN,
        "VALEURS_J2_CHARGE": VALEURS_J2_CHARGE,
        "VITESSE_DIAGNOSTIC": VITESSE_DIAGNOSTIC,
        "TIMEOUT_DIAGNOSTIC": TIMEOUT_DIAGNOSTIC,
        "DELAI_STABILISATION": DELAI_STABILISATION,
        "TOLERANCE": TOLERANCE,
        "pas_a_pas": pas_a_pas,
    })

    print()
    print("Pose actuelle :", formaterAngles(lireAngles(mc)))
    print()
    confirmation = input("Lancer le diagnostic ? (o/n) : ").strip().lower()

    if confirmation != "o":
        print("Diagnostic annule par l'operateur.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "annule par l'operateur"}))
        return 1

    # -----------------------------------------------------
    # Mise en pose d'essai
    # -----------------------------------------------------

    """
    Le bras part de la pose d'essai, pas de la pose
    initiale : on veut mesurer J3 dans une configuration
    degagee avant de conclure quoi que ce soit sur la pose
    repliee du demonstrateur.
    """

    print()
    print("Mise en pose d'essai.")

    if envoyerPose(mc, POSE_ESSAI, pas_a_pas, journal,
                   "mise en place", "pose d'essai") is None:
        print("Impossible d'atteindre la pose d'essai, diagnostic interrompu.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "pose d'essai inatteignable"}))
        return 1

    # -----------------------------------------------------
    # Tests
    # -----------------------------------------------------

    balayage = testBalayage(mc, pas_a_pas, journal)
    insistance = testInsistance(mc, pas_a_pas, journal)
    charge = testCharge(mc, pas_a_pas, journal)
    hysteresis = testHysteresis(mc, pas_a_pas, journal)

    print()
    reponse = input("Faire aussi le test bras relache (test 5) ? (o/n) : ").strip().lower()

    if reponse == "o":
        sans_couple = testSansCouple(mc, journal)
    else:
        sans_couple = None

    # -----------------------------------------------------
    # Verdict
    # -----------------------------------------------------

    resume = afficherVerdict(balayage, hysteresis, insistance, charge, sans_couple)

    print()
    print("Mesures ecrites dans :", FICHIER_RESULTATS)

    """
    Ce que l'operateur a vu ne se lit dans aucune mesure :
    un bras qui force, qui claque ou qui touche la table
    designe la cause bien plus vite que le tableau.
    """

    print()
    print("Qu'a fait le bras ? (bruit, a-coup, contact avec la table, vibration...)")
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
        print("ATTENTION : si le bras etait relache, il l'est toujours.")
        code_retour = 1

    print()
    print("Programme termine.")

    raise SystemExit(code_retour)
