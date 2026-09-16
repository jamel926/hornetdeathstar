"""
=========================================================
ROBOT MYCOBOT 280 - COMMANDE DE LA TETE, BASE FIGEE
=========================================================

Objectif :
---------
Piloter J4, J5 et J6 sans jamais redemander a J1, J2 et J3
de bouger. C'est la phase 1 du projet : le laser part
toujours du meme endroit, seule l'orientation change.

Pourquoi un module a part :
--------------------------
send_angles() prend les SIX axes, on ne peut pas en omettre
un. Commander la tete revient donc a renvoyer aussi la
base, et la question est : avec quelles valeurs ?

    - avec les valeurs NOMINALES, on redemande a la base
      d'aller quelque part a chaque tir ;
    - avec les valeurs MESUREES, on lui demande de rester
      exactement ou elle est.

Ce module fait le second choix. Sur un bras sain, ca evite
d'user la base et de reintroduire son erreur de
positionnement a chaque mouvement. Sur un bras dont un axe
de base ne repond plus, c'est la seule facon de bouger la
tete sans exiger l'impossible du firmware.

Le banc d'essai a J3 mort :
--------------------------
Le servo ne repond plus mais le reducteur freine, donc
l'articulation tient la position ou on la place a la main.
Deux consequences qui justifient tout ce module :

    1. On ne commande JAMAIS la valeur nominale de J3, on
       renvoie la valeur lue. Sinon le firmware attend une
       arrivee qui n'aura pas lieu.

    2. L'arrivee ne se juge que sur J4, J5, J6. Un controle
       sur les six axes echouerait a chaque appel alors que
       la tete est correctement placee.

ATTENTION - la base est tenue par du frottement :
------------------------------------------------
Rien ne la maintient activement. Elle peut glisser sous les
vibrations de la tete, et le systeme n'en saurait rien : il
croirait viser juste. C'est pour ca que verifierBase()
existe, et c'est une condition d'inhibition du tir, pas un
confort de mise au point.

Utilisation :
------------
    from PU_PositionInitiale import connecterRobot
    from PU_Tete import enregistrerReference, verifierBase, bougerTete

    mc = connecterRobot()
    enregistrerReference(mc, "banc d'essai")   # une fois, bras place
    ...
    if verifierBase(mc)[0]:
        bougerTete(mc, [75.0, 10.0, 1.0])

    python3 PU_Tete.py                          # console interactive
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import json
import os
import time
from datetime import datetime

from PU_PositionInitiale import (
    connecterRobot,
    lireAngles,
    verifierAngles,
    ANGLES_POSITION_INITIALE,
    NOMS_ARTICULATIONS
)


# =========================================================
# 2. DECOUPAGE DU BRAS
# =========================================================

"""
J1, J2, J3 placent le poignet quelque part dans l'espace :
c'est la BASE, figee en phase 1.
J4, J5, J6 orientent le faisceau depuis cet endroit :
c'est la TETE, la seule partie commandee.
"""

INDICES_BASE = [0, 1, 2]
INDICES_TETE = [3, 4, 5]


# =========================================================
# 3. PARAMETRES
# =========================================================

VITESSE_TETE = 20        # int, 1 a 100 (impose par pymycobot)
TIMEOUT_TETE = 10        # secondes

# Ecart tolere sur les axes de tete pour declarer l'arrivee.
TOLERANCE_TETE = 2.0     # degres

"""
Ecart tolere sur la base avant de considerer qu'elle a
bouge. Volontairement plus serre que la tete : la base est
tenue par du frottement, et c'est elle qui porte toute la
calibration de pointage. Un degre de derive au coude
deplace le point d'impact de plusieurs centimetres.
"""
TOLERANCE_BASE = 1.0     # degres

# Periode de scrutation pendant l'attente d'arrivee.
PERIODE_SCRUTATION = 0.2  # secondes

"""
Decalage de la pose d'approche, en degres.

Toute pose de tete est atteinte en passant d'abord par
cette pose decalee, donc TOUJOURS dans le meme sens de
rotation. Le jeu des reducteurs est ainsi rattrape du meme
cote a chaque fois.

Ce n'est pas une precaution theorique : la campagne de
repetabilite du 08/09 donne 2,19 degres de dispersion sur
J1 avec une approche libre, contre 0,09 avec une approche
identique a chaque essai. Le meme jeu existe sur J5, qui
est un axe de pointage.

Le signe compte, la valeur peu : ne pas l'annuler.
"""
APPROCHE_TETE = [8.0, 8.0, 0.0]   # degres


# =========================================================
# 4. FICHIER DE REFERENCE
# =========================================================

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")
FICHIER_REFERENCE = os.path.join(DOSSIER_RESULTATS, "reference_base.json")


# =========================================================
# 5. LECTURE PAR PARTIE
# =========================================================

def lirePartie(mc, indices):
    """Retourne les angles des articulations demandees, ou None si la lecture echoue."""

    angles = lireAngles(mc)

    if angles is None:
        return None

    return [angles[i] for i in indices]


def lireBase(mc):
    """Retourne les angles J1, J2, J3 courants."""

    return lirePartie(mc, INDICES_BASE)


def lireTete(mc):
    """Retourne les angles J4, J5, J6 courants."""

    return lirePartie(mc, INDICES_TETE)


def construirePose(base, tete):
    """Assemble une consigne six axes a partir des trois angles de base et des trois de tete."""

    return [float(base[0]), float(base[1]), float(base[2]),
            float(tete[0]), float(tete[1]), float(tete[2])]


def ecartPartie(mesure, consigne, indices):
    """Retourne le plus grand ecart, en degres, sur les articulations demandees."""

    ecarts = []

    for i in indices:
        ecarts.append(abs(mesure[i] - consigne[i]))

    return max(ecarts)


# =========================================================
# 6. REFERENCE DE LA BASE
# =========================================================

def enregistrerReference(mc, etiquette_robot, commentaire=""):
    """Enregistre la pose courante comme reference de base et retourne le dictionnaire ecrit."""

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture impossible, reference non enregistree.")
        return None

    reference = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "robot": etiquette_robot,
        "commentaire": commentaire,
        "base": [round(angles[i], 3) for i in INDICES_BASE],
        "tete": [round(angles[i], 3) for i in INDICES_TETE],
        "pose_complete": [round(a, 3) for a in angles],
        "pose_nominale_projet": list(ANGLES_POSITION_INITIALE),
    }

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        with open(FICHIER_REFERENCE, "w") as f:
            json.dump(reference, f, indent=2, ensure_ascii=False)

    except Exception as e:
        print("Impossible d'ecrire la reference :", e)
        return None

    print()
    print("Reference enregistree pour le robot :", etiquette_robot)
    print("  base J1 J2 J3 :", reference["base"])
    print("  tete J4 J5 J6 :", reference["tete"])
    print("  fichier       :", FICHIER_REFERENCE)

    # -----------------------------------------------------
    # Ecart a la pose nominale du projet
    # -----------------------------------------------------

    """
    On affiche l'ecart a ANGLES_POSITION_INITIALE sans le
    corriger : sur un bras dont un axe est place a la main,
    la reference utile est la pose REELLE, pas celle que le
    projet avait prevue. Mais il faut savoir de combien on
    s'en ecarte, parce que la couverture de la zone en
    depend.
    """

    for i in INDICES_BASE:
        ecart = angles[i] - ANGLES_POSITION_INITIALE[i]
        if abs(ecart) > TOLERANCE_BASE:
            print("  ATTENTION :", NOMS_ARTICULATIONS[i], "s'ecarte de",
                  round(ecart, 2), "degres de la pose nominale du projet.")

    return reference


def chargerReference():
    """Relit la reference de base enregistree, ou None si aucune n'existe."""

    if not os.path.isfile(FICHIER_REFERENCE):
        return None

    try:
        with open(FICHIER_REFERENCE) as f:
            return json.load(f)

    except Exception as e:
        print("Reference illisible :", e)
        return None


def verifierBase(mc, tolerance=TOLERANCE_BASE, reference=None):
    """Compare la base courante a la reference et retourne (conforme, ecarts, mesure)."""

    if reference is None:
        reference = chargerReference()

    if reference is None:
        print("Aucune reference enregistree : impossible de verifier la base.")
        return False, None, None

    base = lireBase(mc)

    if base is None:
        print("Lecture impossible : la base est consideree comme non conforme.")
        return False, None, None

    ecarts = []

    for i in range(3):
        ecarts.append(base[i] - reference["base"][i])

    conforme = max(abs(e) for e in ecarts) <= tolerance

    return conforme, ecarts, base


def afficherEtatBase(mc, tolerance=TOLERANCE_BASE):
    """Affiche l'ecart de la base a sa reference et retourne True si elle est conforme."""

    conforme, ecarts, base = verifierBase(mc, tolerance)

    if ecarts is None:
        return False

    print()
    print("Base mesuree :", [round(a, 2) for a in base])
    print("Derive       :", [round(e, 2) for e in ecarts], "degres")

    if conforme:
        print("Base conforme a sa reference.")
    else:
        print("BASE HORS REFERENCE.")
        print("La calibration de pointage n'est plus valable, et le tir")
        print("doit rester inhibe tant que la base n'a pas ete replacee.")

    return conforme


# =========================================================
# 7. DEPLACEMENT DE LA TETE
# =========================================================

def bougerTete(mc, tete, vitesse=VITESSE_TETE, timeout=TIMEOUT_TETE,
               tolerance=TOLERANCE_TETE):
    """Commande J4, J5, J6 en laissant la base ou elle est et retourne (arrive, pose mesuree)."""

    # -----------------------------------------------------
    # Construction de la consigne
    # -----------------------------------------------------

    base = lireBase(mc)

    if base is None:
        print("Lecture impossible, mouvement annule.")
        return False, None

    consigne = construirePose(base, tete)

    if not verifierAngles(consigne):
        print("Consigne hors limites, mouvement annule.")
        return False, None

    # -----------------------------------------------------
    # Envoi
    # -----------------------------------------------------

    """
    send_angles et non sync_send_angles.

    sync_send_angles attend que les SIX axes soient arrives.
    Sur un bras dont un axe de base ne repond plus, cette
    attente peut ne jamais se terminer : on partirait en
    timeout complet a chaque mouvement de tete. On scrute
    donc nous-memes, et uniquement les axes qu'on commande.
    """

    try:

        if mc.is_moving() == 1:
            mc.stop()
            time.sleep(0.3)

        mc.send_angles(consigne, int(vitesse))

    except Exception as e:
        print("Erreur pendant l'envoi de la consigne :", e)
        return False, None

    # -----------------------------------------------------
    # Attente d'arrivee, sur la tete seulement
    # -----------------------------------------------------

    debut = time.time()
    mesure = None

    while time.time() - debut < timeout:

        time.sleep(PERIODE_SCRUTATION)

        mesure = lireAngles(mc)

        if mesure is None:
            continue

        if ecartPartie(mesure, consigne, INDICES_TETE) <= tolerance:
            break

    if mesure is None:
        print("Aucune lecture pendant l'attente d'arrivee.")
        return False, None

    ecart_tete = ecartPartie(mesure, consigne, INDICES_TETE)
    ecart_base = ecartPartie(mesure, consigne, INDICES_BASE)

    arrive = ecart_tete <= tolerance

    if not arrive:
        print("Tete non arrivee : ecart de", round(ecart_tete, 2),
              "degres apres", timeout, "secondes.")

    # -----------------------------------------------------
    # La base a-t-elle bouge pendant le mouvement ?
    # -----------------------------------------------------

    """
    C'est le mode de defaillance propre a une base tenue par
    du frottement : elle glisse pendant que la tete bouge.
    On le detecte ici, au plus pres du mouvement, plutot que
    de le decouvrir plus tard dans une erreur de pointage
    inexplicable.
    """

    if ecart_base > TOLERANCE_BASE:
        print("ATTENTION : la base a bouge de", round(ecart_base, 2),
              "degres pendant le mouvement de la tete.")

    return arrive, mesure


def bougerTeteAvecApproche(mc, tete, vitesse=VITESSE_TETE, timeout=TIMEOUT_TETE,
                           tolerance=TOLERANCE_TETE, decalage=None):
    """Atteint la pose de tete en passant par une pose d'approche, toujours dans le meme sens."""

    if decalage is None:
        decalage = APPROCHE_TETE

    approche = [tete[i] + decalage[i] for i in range(3)]

    base = lireBase(mc)

    if base is None:
        print("Lecture impossible, mouvement annule.")
        return False, None

    """
    Si la pose d'approche sort des limites, on rejoint la
    cible directement plutot que d'echouer : mieux vaut un
    point atteint avec un peu plus de jeu qu'un point non
    atteint. On le signale, parce que ce point sera plus
    disperse que les autres.
    """

    if verifierAngles(construirePose(base, approche)):
        bougerTete(mc, approche, vitesse, timeout, tolerance)
    else:
        print("Pose d'approche hors limites : arrivee directe, jeu non rattrape.")

    return bougerTete(mc, tete, vitesse, timeout, tolerance)


def retourTete(mc, reference=None, vitesse=VITESSE_TETE):
    """Ramene la tete aux angles de la reference enregistree."""

    if reference is None:
        reference = chargerReference()

    if reference is None:
        print("Aucune reference enregistree, retour impossible.")
        return False, None

    return bougerTete(mc, reference["tete"], vitesse)


# =========================================================
# 8. EXECUTION AUTONOME
# =========================================================

def lireNombre(message):
    """Demande un nombre a l'operateur jusqu'a ce que la saisie soit valide."""

    while True:

        saisie = input(message).strip()

        try:
            return float(saisie)

        except ValueError:
            print("Saisie invalide, entrer un nombre.")


def afficherMenu():
    """Affiche les actions disponibles."""

    print()
    print("-----------------------------------------")
    print(" 1 - Enregistrer la pose courante comme reference")
    print(" 2 - Etat de la base (derive depuis la reference)")
    print(" 3 - Bouger la tete (J4, J5, J6)")
    print(" 4 - Ramener la tete a la reference")
    print(" 5 - Afficher la pose complete")
    print(" q - Quitter")
    print("-----------------------------------------")


def main():
    """Console interactive de commande de la tete, base figee."""

    print()
    print("=========================================")
    print("   COMMANDE DE LA TETE - BASE FIGEE")
    print("=========================================")
    print()
    print("Seuls J4, J5 et J6 sont commandes. J1, J2 et J3")
    print("sont relus et renvoyes tels quels a chaque")
    print("mouvement : ils ne bougent pas.")
    print()

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    reference = chargerReference()

    if reference is None:
        print()
        print("Aucune reference de base enregistree.")
        print("Placer le bras dans sa pose de travail, puis choisir 1.")
    else:
        print()
        print("Reference chargee - robot :", reference["robot"],
              "- enregistree le", reference["date"])
        print("VERIFIER que c'est bien le robot devant vous :")
        print("une reference relevee sur l'autre machine n'a aucun sens ici.")

    while True:

        afficherMenu()
        choix = input("Choix : ").strip().lower()

        # -------------------------------------------------
        # Reference
        # -------------------------------------------------

        if choix == "1":

            etiquette = input("Etiquette du robot (ex : banc_essai, demo) : ").strip()

            if etiquette == "":
                print("Etiquette obligatoire : deux robots, deux references.")
                continue

            commentaire = input("Commentaire (entree pour passer) : ").strip()
            nouvelle = enregistrerReference(mc, etiquette, commentaire)

            if nouvelle is not None:
                reference = nouvelle

        elif choix == "2":
            afficherEtatBase(mc)

        # -------------------------------------------------
        # Mouvement de tete
        # -------------------------------------------------

        elif choix == "3":

            """
            La base est verifiee avant chaque mouvement, pas
            seulement au demarrage : c'est exactement la
            condition qui inhibera le tir plus tard.
            """

            if reference is not None and not afficherEtatBase(mc):
                confirmation = input("Bouger quand meme la tete ? (o/n) : ").strip().lower()
                if confirmation != "o":
                    continue

            tete_courante = lireTete(mc)
            print()
            print("Tete actuelle :", [round(a, 2) for a in tete_courante])

            j4 = lireNombre("J4 (deg) : ")
            j5 = lireNombre("J5 (deg) : ")
            j6 = lireNombre("J6 (deg) : ")

            arrive, mesure = bougerTete(mc, [j4, j5, j6])

            if mesure is not None:
                print("Pose mesuree :", [round(a, 2) for a in mesure])

            if arrive:
                print("Tete en position.")

        elif choix == "4":
            retourTete(mc, reference)

        elif choix == "5":
            print()
            print("Pose complete :", [round(a, 2) for a in lireAngles(mc)])

        elif choix == "q":
            print()
            print("Les moteurs restent alimentes.")
            return 0

        else:
            print("Choix inconnu.")


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
