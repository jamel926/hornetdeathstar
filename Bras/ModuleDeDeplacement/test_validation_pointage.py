"""
=========================================================
ROBOT MYCOBOT 280 - VALIDATION CROISEE DU POINTAGE
=========================================================

Objectif :
---------
Mesurer la vraie precision de visee, sur des points qui
n'ont PAS servi a construire le modele.

Pourquoi ce script existe :
--------------------------
Les residus affiches par PU_Pointage disent seulement a
quel point le modele colle aux points qui l'ont construit.
C'est toujours flatteur, et ca ne mesure rien d'autre que
lui-meme. Un modele a huit coefficients ajuste sur neuf
points peut avoir des residus quasi nuls et se tromper
lourdement partout ailleurs.

La seule mesure honnete est faite sur des points tires au
hasard dans la zone et ecartes de l'ajustement. C'est
l'etape E3d de la feuille de route.

Ce que le script separe :
------------------------
    BIAIS      - le modele tape systematiquement a cote,
                 toujours du meme cote. Se rattrape par un
                 decalage constant.

    DISPERSION - les impacts se repartissent autour de la
                 cible sans regularite. Ne se rattrape pas,
                 et se retrouve integralement dans l'erreur
                 de tir.

Les deux sont donnes separement parce qu'ils appellent des
decisions differentes.

Utilisation :
------------
    python3 test_validation_pointage.py

ATTENTION :
----------
Pointeur rouge classe 2 uniquement.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import csv
import os
import random
from datetime import datetime

from PU_PositionInitiale import connecterRobot

from PU_Tete import lireAngles, chargerReference, verifierBase

from PU_Pointage import (
    COTE_CARREAU,
    ENTETE_BALAYAGE,
    FICHIER_BALAYAGE,
    DOSSIER_RESULTATS,
    chargerModele,
    afficherModele,
    pointerSur,
    anglesPourPoint
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

NB_POINTS_DEFAUT = 3

"""
Fraction de l'enveloppe laissee libre sur les bords.

Les points de validation sont tires a l'interieur de la
zone balayee, pas sur son bord : le modele y est le moins
contraint, et valider dessus melangerait l'erreur du
modele avec un effet de bord.
"""
RETRAIT_BORD = 0.15


# =========================================================
# 3. TIRAGE DES POINTS
# =========================================================

def tirerPoints(modele, nombre):
    """Tire des points au hasard a l'interieur de la zone couverte par le modele."""

    e = modele["enveloppe"]

    largeur_u = e["u_max"] - e["u_min"]
    largeur_v = e["v_max"] - e["v_min"]

    points = []

    for _ in range(nombre):

        u = random.uniform(e["u_min"] + RETRAIT_BORD * largeur_u,
                           e["u_max"] - RETRAIT_BORD * largeur_u)

        v = random.uniform(e["v_min"] + RETRAIT_BORD * largeur_v,
                           e["v_max"] - RETRAIT_BORD * largeur_v)

        """
        Un point tire dans l'enveloppe des IMPACTS peut
        malgre tout demander des angles hors de la zone
        balayee, la correspondance n'etant pas un
        rectangle. On l'ecarte au tirage plutot que de
        laisser la visee echouer devant l'operateur.
        """

        if anglesPourPoint(modele, u, v) is not None:
            points.append((u, v))

    return points


# =========================================================
# 4. STATISTIQUES
# =========================================================

def moyenne(valeurs):
    """Retourne la moyenne d'une liste de nombres."""

    return sum(valeurs) / len(valeurs)


def ecartType(valeurs):
    """Retourne l'ecart type d'une liste de nombres, nul si elle en contient moins de deux."""

    if len(valeurs) < 2:
        return 0.0

    m = moyenne(valeurs)

    somme = 0.0
    for v in valeurs:
        somme += (v - m) ** 2

    return (somme / (len(valeurs) - 1)) ** 0.5


def analyserErreurs(erreurs):
    """Retourne biais, dispersion et erreur de distance a partir des ecarts (du, dv)."""

    du = [e[0] for e in erreurs]
    dv = [e[1] for e in erreurs]

    distances = [(e[0] ** 2 + e[1] ** 2) ** 0.5 for e in erreurs]

    biais_u = moyenne(du)
    biais_v = moyenne(dv)

    return {
        "nb_points": len(erreurs),
        "biais_u_cm": round(biais_u, 2),
        "biais_v_cm": round(biais_v, 2),
        "biais_norme_cm": round((biais_u ** 2 + biais_v ** 2) ** 0.5, 2),
        "dispersion_u_cm": round(ecartType(du), 2),
        "dispersion_v_cm": round(ecartType(dv), 2),
        "erreur_moyenne_cm": round(moyenne(distances), 2),
        "erreur_max_cm": round(max(distances), 2),
        "erreurs_cm": [[round(e[0], 2), round(e[1], 2)] for e in erreurs],
    }


def afficherRapport(resultat):
    """Affiche le rapport de validation en separant ce qui se corrige de ce qui ne se corrige pas."""

    print()
    print("=========================================")
    print("   VALIDATION SUR", resultat["nb_points"], "POINTS ECARTES")
    print("=========================================")
    print()
    print("Erreur moyenne :", resultat["erreur_moyenne_cm"], "cm")
    print("Erreur maximale:", resultat["erreur_max_cm"], "cm")

    print()
    print("BIAIS (rattrapable) :",
          resultat["biais_u_cm"], "cm en u,",
          resultat["biais_v_cm"], "cm en v")
    print("  norme :", resultat["biais_norme_cm"], "cm")
    print("  Un biais constant se corrige par un decalage applique a la")
    print("  consigne. Il ne remet pas le modele en cause.")

    print()
    print("DISPERSION (non rattrapable) :",
          resultat["dispersion_u_cm"], "cm en u,",
          resultat["dispersion_v_cm"], "cm en v")
    print("  Elle se retrouve telle quelle dans l'erreur de tir. C'est elle")
    print("  qui fixe la precision atteignable, pas l'erreur moyenne.")

    print()
    print("=========================================")

    """
    Aucun verdict automatique : le budget de precision de
    E3 n'est pas fixe, il attend l'erreur de position
    annoncee par l'equipe vision. Afficher un PASS ou un
    FAIL contre un seuil invente donnerait une fausse
    assurance.
    """

    print("Pas de verdict automatique : le budget de precision")
    print("n'est pas fixe. Il depend de l'erreur de la stereovision,")
    print("a demander a l'equipe camera.")
    print("=========================================")


# =========================================================
# 5. SAISIE
# =========================================================

def lireNombre(message):
    """Demande un nombre a l'operateur jusqu'a ce que la saisie soit valide."""

    while True:

        saisie = input(message).strip().replace(",", ".")

        try:
            return float(saisie)

        except ValueError:
            print("Saisie invalide, entrer un nombre.")


def demanderEntier(message, defaut):
    """Demande un entier, retourne la valeur par defaut si la saisie est vide."""

    saisie = input(message).strip()

    if saisie == "":
        return defaut

    try:
        return int(saisie)

    except ValueError:
        return defaut


def enregistrerPoint(serie, index, angles, mesure, impact, cible):
    """Ajoute le point de validation au fichier de balayage, marque comme tel."""

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        fichier_neuf = not os.path.isfile(FICHIER_BALAYAGE)

        with open(FICHIER_BALAYAGE, "a", newline="") as f:

            redacteur = csv.writer(f)

            if fichier_neuf:
                redacteur.writerow(ENTETE_BALAYAGE)

            redacteur.writerow(
                [datetime.now().isoformat(timespec="seconds"), serie, "validation", index,
                 round(angles[0], 2), round(angles[1], 2)]
                + [round(a, 2) for a in mesure]
                + [round(impact[0], 2), round(impact[1], 2),
                   "cible " + str(round(cible[0], 2)) + "/" + str(round(cible[1], 2))]
            )

    except Exception as e:
        print("Impossible d'ecrire le point de validation :", e)


# =========================================================
# 6. EXECUTION AUTONOME
# =========================================================

def main():
    """Vise des points ecartes de l'ajustement et mesure l'erreur reelle."""

    print()
    print("=========================================")
    print("   VALIDATION CROISEE DU POINTAGE")
    print("=========================================")
    print()
    print("ATTENTION : pointeur rouge classe 2 uniquement.")
    print()

    # -----------------------------------------------------
    # Modele
    # -----------------------------------------------------

    modele = chargerModele()

    if modele is None:
        print("Aucun modele de pointage enregistre.")
        print("Lancer d'abord test_balayage_tete.py puis PU_Pointage.py.")
        return 1

    afficherModele(modele)

    reference = chargerReference()

    if reference is not None and modele.get("robot") not in ("", None):
        if reference["robot"] != modele["robot"]:
            print()
            print("ATTENTION : le modele vient du robot", modele["robot"],
                  "et la reference de base du robot", reference["robot"], ".")
            reponse = input("Continuer quand meme ? (o/n) : ").strip().lower()
            if reponse != "o":
                return 1

    # -----------------------------------------------------
    # Points a viser
    # -----------------------------------------------------

    nb_points = demanderEntier(
        "Nombre de points de validation [" + str(NB_POINTS_DEFAUT) + "] : ",
        NB_POINTS_DEFAUT
    )

    points = tirerPoints(modele, nb_points)

    if len(points) == 0:
        print("Aucun point tirable dans la zone du modele.")
        return 1

    print()
    print(len(points), "points tires au hasard dans la zone couverte.")

    serie = input("Etiquette de la serie [validation] : ").strip() or "validation"

    # -----------------------------------------------------
    # Connexion
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    journal = ouvrirJournal("validation_pointage", mc, {
        "serie": serie,
        "modele": {"robot": modele.get("robot"), "date": modele.get("date"),
                   "nb_points": modele.get("nb_points"),
                   "residus": modele.get("residus")},
        "nb_points_validation": len(points),
        "COTE_CARREAU": COTE_CARREAU,
    })

    if reference is not None:

        conforme, ecarts, _ = verifierBase(mc)

        if not conforme:
            print()
            print("BASE HORS REFERENCE :", [round(e, 2) for e in ecarts], "degres")
            print("Le modele a ete ajuste avec la base ailleurs : la validation")
            print("mesurerait le glissement de la base, pas la qualite du modele.")
            reponse = input("Continuer quand meme ? (o/n) : ").strip().lower()
            if reponse != "o":
                annoncerJournal(fermerJournal(journal, {"interrompu": "base hors reference"}))
                return 1

    # -----------------------------------------------------
    # Boucle de validation
    # -----------------------------------------------------

    erreurs = []

    for index, (u_cible, v_cible) in enumerate(points, start=1):

        print()
        print("=========================================")
        print("Point", index, "/", len(points))
        print("Cible :", round(u_cible / COTE_CARREAU, 2), "/",
              round(v_cible / COTE_CARREAU, 2), "carreaux  (",
              round(u_cible, 2), "/", round(v_cible, 2), "cm )")
        print("=========================================")

        arrive, angles = pointerSur(mc, modele, u_cible, v_cible)

        if angles is None:
            print("Visee impossible pour ce point, il est ecarte.")
            continue

        if not arrive:
            print("La tete n'a pas atteint sa consigne : point ecarte.")
            continue

        mesure = lireAngles(mc)

        if mesure is None:
            print("Lecture impossible, point ecarte.")
            continue

        print()
        print("Ou est tombe le point, en carreaux ?")

        u_reel = lireNombre("  u (carreaux) : ") * COTE_CARREAU
        v_reel = lireNombre("  v (carreaux) : ") * COTE_CARREAU

        erreur = (u_reel - u_cible, v_reel - v_cible)
        erreurs.append(erreur)

        distance = (erreur[0] ** 2 + erreur[1] ** 2) ** 0.5

        print("  Ecart :", round(erreur[0], 2), "/", round(erreur[1], 2),
              "cm   distance", round(distance, 2), "cm")

        enregistrerPoint(serie, index, angles, mesure, (u_reel, v_reel), (u_cible, v_cible))

        ajouterMesure(
            journal, "validation", "point " + str(index),
            None, mesure, None,
            extra={
                "cible_cm": [round(u_cible, 2), round(v_cible, 2)],
                "impact_cm": [round(u_reel, 2), round(v_reel, 2)],
                "ecart_cm": [round(erreur[0], 2), round(erreur[1], 2)],
                "angles_calcules": [round(angles[0], 2), round(angles[1], 2)],
            }
        )

    # -----------------------------------------------------
    # Rapport
    # -----------------------------------------------------

    if len(erreurs) == 0:
        print()
        print("Aucun point exploitable, pas de validation possible.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "aucun point exploitable"}))
        return 1

    resultat = analyserErreurs(erreurs)
    afficherRapport(resultat)

    print()
    print("Points ecrits dans :", FICHIER_BALAYAGE, "(marques validation)")

    print()
    print("Remarques ? (point flou, lecture difficile, bras qui vibre...)")
    demanderObservation(journal)

    annoncerJournal(fermerJournal(journal, resultat))

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
