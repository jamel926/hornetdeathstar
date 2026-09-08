"""
=========================================================
ROBOT MYCOBOT 280 - BALAYAGE DE LA TETE ET RELEVE D'IMPACT
=========================================================

Objectif :
---------
Produire les donnees qui manquent pour viser : pour une
grille de couples (J4, J5), ou tombe reellement le point
lumineux sur le quadrillage pose au sol.

C'est l'etape E3a de la feuille de route. Le modele
s'ajuste ensuite dessus avec PU_Pointage.

Ce que le script enregistre :
----------------------------
Les angles COMMANDES et les angles MESURES. Le modele
s'ajustera sur les mesures : la campagne du 08/09 donne
1,49 degre de dispersion sur J5, et ajuster sur la consigne
ferait entrer ce jeu dans le modele au lieu de le
contourner.

Chaque point est marque "ajustement" ou "validation". Les
points de validation ne serviront JAMAIS a l'ajustement :
valider un modele sur les points qui l'ont construit ne
mesure que lui-meme.

Prerequis :
----------
    - pointeur rouge classe 2 monte sur la bride, coaxial
      a l'axe de J6, et allume ;
    - quadrillage en place, avec une case choisie comme
      origine et marquee ;
    - reference de base enregistree (PU_Tete.py, option 1) ;
    - fenetre utile de (J4, J5) reperee a la main
      (PU_Tete.py, option 3) : rien ne garantit que le
      faisceau atteigne le quadrillage depuis la pose
      initiale.

Utilisation :
------------
    python3 test_balayage_tete.py

ATTENTION :
----------
Jamais le laser de puissance pour cette campagne.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import csv
import os
from datetime import datetime

from PU_PositionInitiale import connecterRobot, lireAngles

from PU_Tete import (
    bougerTeteAvecApproche,
    lireTete,
    chargerReference,
    verifierBase,
    afficherEtatBase
)

from PU_Pointage import (
    COTE_CARREAU,
    ENTETE_BALAYAGE,
    FICHIER_BALAYAGE,
    DOSSIER_RESULTATS,
    ajusterModele,
    afficherModele,
    enregistrerModele,
    chargerMesures
)

from PU_Journal import (
    ouvrirJournal,
    ajouterMesure,
    demanderObservation,
    fermerJournal,
    annoncerJournal
)


# =========================================================
# 2. PARAMETRES PAR DEFAUT DE LA GRILLE
# =========================================================

"""
Grille 3 x 3 par defaut, soit neuf points.

C'est calibre sur ce qui est faisable en une seance de
releve a la main. Un modele bilineaire a quatre
coefficients par coordonnee s'ajuste correctement dessus,
avec de quoi lire des residus.

Il faut imperativement faire varier J4 ET J5 : une grille
degeneree en ligne rend le systeme singulier et
l'ajustement echoue.
"""

NB_PAS_DEFAUT = 3

"""
Demi-fenetre angulaire conseillee, en degres.

Le modele bilineaire de PU_Pointage decrit une zone
restreinte, pas toute la course de la tete. Sur une
geometrie simulee (faisceau vise un plan horizontal 30 cm
plus bas, J4 en azimut et J5 en elevation), son erreur de
visee vaut :

    +/- 10 deg : 0,07 cm en moyenne, 0,2 cm au pire
    +/- 15 deg : 0,17 cm            0,6 cm
    +/- 20 deg : 0,32 cm            1,3 cm
    +/- 30 deg : 0,85 cm            3,8 cm

Passer de 9 a 16 points ne change RIEN a ces chiffres :
c'est la forme du modele qui limite, pas le nombre de
mesures. Elargir la fenetre se paie donc en precision, et
plus de points n'y remedie pas.

En dessous de 15 degres, l'erreur du modele reste sous le
plancher mecanique du bras (1,49 deg de dispersion sur J5,
soit 0,8 cm a 30 cm) : ce n'est pas elle qui limite.
"""
FENETRE_MAX_CONSEILLEE = 15.0   # degres de part et d'autre du centre


# =========================================================
# 3. ECRITURE DES RESULTATS
# =========================================================

def enregistrerPoint(serie, usage, index, consigne, mesure, impact, commentaire):
    """Ajoute une ligne au fichier de balayage, en creant l'entete au premier point."""

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        fichier_neuf = not os.path.isfile(FICHIER_BALAYAGE)

        with open(FICHIER_BALAYAGE, "a", newline="") as f:

            redacteur = csv.writer(f)

            if fichier_neuf:
                redacteur.writerow(ENTETE_BALAYAGE)

            if impact is None:
                u, v = "", ""
            else:
                u, v = round(impact[0], 2), round(impact[1], 2)

            redacteur.writerow(
                [datetime.now().isoformat(timespec="seconds"), serie, usage, index,
                 round(consigne[0], 2), round(consigne[1], 2)]
                + [round(a, 2) for a in mesure]
                + [u, v, commentaire]
            )

    except Exception as e:
        print("Impossible d'ecrire le fichier de balayage :", e)
        return False

    return True


# =========================================================
# 4. SAISIE DE L'OPERATEUR
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
        print("Saisie invalide, valeur par defaut retenue :", defaut)
        return defaut


def lireImpact():
    """Demande la position du point sur le quadrillage et retourne (u, v) en cm, ou None si saute."""

    print("  Position du point, en carreaux (1 carreau =", COTE_CARREAU, "cm).")
    print("  s = point saute (faisceau hors quadrillage ou invisible)")

    saisie = input("  u (carreaux) ou s : ").strip().lower().replace(",", ".")

    if saisie == "s":
        return None

    try:
        u = float(saisie)

    except ValueError:
        print("  Saisie invalide, point saute.")
        return None

    v = lireNombre("  v (carreaux) : ")

    return u * COTE_CARREAU, v * COTE_CARREAU


# =========================================================
# 5. CONSTRUCTION DE LA GRILLE
# =========================================================

def construireGrille(j4_min, j4_max, j5_min, j5_max, nb_pas):
    """Retourne la liste des couples (J4, J5) a parcourir."""

    grille = []

    for i in range(nb_pas):

        if nb_pas == 1:
            j4 = (j4_min + j4_max) / 2.0
        else:
            j4 = j4_min + (j4_max - j4_min) * i / (nb_pas - 1)

        for k in range(nb_pas):

            if nb_pas == 1:
                j5 = (j5_min + j5_max) / 2.0
            else:
                j5 = j5_min + (j5_max - j5_min) * k / (nb_pas - 1)

            grille.append((j4, j5))

    return grille


# =========================================================
# 6. DEROULEMENT D'UN POINT
# =========================================================

def executerPoint(mc, index, total, consigne, serie, usage, journal, reference):
    """Amene la tete sur un couple d'angles, releve l'impact, et retourne True si le point est garde."""

    print()
    print("=========================================")
    print("Point", index, "/", total, "  J4 =", round(consigne[0], 2),
          " J5 =", round(consigne[1], 2))
    print("=========================================")

    # -----------------------------------------------------
    # La base n'a pas bouge ?
    # -----------------------------------------------------

    """
    Verifie AVANT chaque point, pas seulement au demarrage.
    Si la base glisse en cours de campagne, les points
    releves avant et apres ne decrivent plus la meme
    geometrie et le modele les melange sans le savoir.
    """

    if reference is not None:

        conforme, ecarts, _ = verifierBase(mc)

        if not conforme:
            print()
            print("BASE HORS REFERENCE :", [round(e, 2) for e in ecarts], "degres")
            print("Les points deja releves ne sont plus comparables a ceux qui suivent.")
            reponse = input("Continuer quand meme ? (o/n) : ").strip().lower()
            if reponse != "o":
                return False

    # -----------------------------------------------------
    # Mouvement
    # -----------------------------------------------------

    tete_courante = lireTete(mc)

    if tete_courante is None:
        print("Lecture impossible, point saute.")
        return True

    arrive, mesure = bougerTeteAvecApproche(
        mc, [consigne[0], consigne[1], tete_courante[2]]
    )

    if mesure is None:
        print("Mouvement echoue, point saute.")
        return True

    print("Angles mesures : J4 =", round(mesure[3], 2),
          " J5 =", round(mesure[4], 2), " J6 =", round(mesure[5], 2))

    if not arrive:
        print("ATTENTION : la tete n'a pas atteint sa consigne.")
        print("Le point reste utilisable, le modele s'ajuste sur les angles mesures.")

    # -----------------------------------------------------
    # Releve
    # -----------------------------------------------------

    impact = lireImpact()

    commentaire = "" if arrive else "tete non arrivee"

    enregistrerPoint(serie, usage, index, consigne, mesure, impact, commentaire)

    ajouterMesure(
        journal, usage, "point " + str(index),
        None, mesure, None,
        extra={
            "consigne_j4": round(consigne[0], 2),
            "consigne_j5": round(consigne[1], 2),
            "impact_cm": None if impact is None else [round(impact[0], 2), round(impact[1], 2)],
            "tete_arrivee": arrive,
        }
    )

    if impact is None:
        print("  Point saute.")
    else:
        print("  Impact enregistre :", round(impact[0], 2), "/", round(impact[1], 2), "cm")

    return True


# =========================================================
# 7. EXECUTION AUTONOME
# =========================================================

def main():
    """Deroule le balayage de la tete et releve l'impact du faisceau point par point."""

    print()
    print("=========================================")
    print("   BALAYAGE DE LA TETE - RELEVE D'IMPACT")
    print("=========================================")
    print()
    print("ATTENTION : pointeur rouge classe 2 uniquement,")
    print("jamais le laser de puissance.")
    print()
    print("Avant de commencer, verifier que :")
    print("  - le pointeur est monte et allume ;")
    print("  - le quadrillage est en place et ne bougera pas ;")
    print("  - une case est marquee comme origine (u = 0, v = 0) ;")
    print("  - le sens de u et de v est choisi et note.")
    print()

    # -----------------------------------------------------
    # Reference de base
    # -----------------------------------------------------

    reference = chargerReference()

    if reference is None:
        print("Aucune reference de base enregistree.")
        print("Lancer PU_Tete.py et choisir l'option 1 avant ce balayage.")
        reponse = input("Continuer sans surveillance de la base ? (o/n) : ").strip().lower()
        if reponse != "o":
            return 1
    else:
        print("Reference du robot :", reference["robot"],
              "- enregistree le", reference["date"])
        print("VERIFIER que c'est bien ce robot qui est devant vous.")

    # -----------------------------------------------------
    # Parametres de la campagne
    # -----------------------------------------------------

    serie = input("Etiquette de la serie (ex : campagne1) : ").strip()

    if serie == "":
        serie = "serie_" + datetime.now().strftime("%Y%m%d_%H%M")

    print()
    print("Usage des points de cette serie :")
    print("  1 - ajustement (construction du modele)")
    print("  2 - validation (points gardes de cote, jamais ajustes)")

    usage = "validation" if input("Choix [1] : ").strip() == "2" else "ajustement"
    print("Usage retenu :", usage)

    print()
    print("Fenetre de balayage, reperee a la main au prealable.")

    j4_min = lireNombre("J4 minimum (deg) : ")
    j4_max = lireNombre("J4 maximum (deg) : ")
    j5_min = lireNombre("J5 minimum (deg) : ")
    j5_max = lireNombre("J5 maximum (deg) : ")

    nb_pas = demanderEntier(
        "Nombre de pas par axe [" + str(NB_PAS_DEFAUT) + "] : ",
        NB_PAS_DEFAUT
    )

    grille = construireGrille(j4_min, j4_max, j5_min, j5_max, nb_pas)

    print()
    print(len(grille), "points a relever.")

    """
    Une grille qui ne fait varier qu'un seul axe rend le
    systeme normal singulier : le terme croise et l'un des
    termes lineaires deviennent indiscernables. Autant le
    dire maintenant qu'apres le releve.
    """

    if abs(j4_max - j4_min) < 1.0 or abs(j5_max - j5_min) < 1.0:
        print()
        print("ATTENTION : la grille ne fait varier qu'un seul axe.")
        print("L'ajustement echouera. Faire varier J4 ET J5.")
        reponse = input("Continuer quand meme ? (o/n) : ").strip().lower()
        if reponse != "o":
            return 1

    # -----------------------------------------------------
    # Fenetre trop large pour le modele
    # -----------------------------------------------------

    largeur = max(abs(j4_max - j4_min), abs(j5_max - j5_min)) / 2.0

    if largeur > FENETRE_MAX_CONSEILLEE:
        print()
        print("ATTENTION : fenetre de +/-", round(largeur, 1), "degres.")
        print("Au-dela de +/-", FENETRE_MAX_CONSEILLEE, "degres, le modele bilineaire")
        print("decroche et l'erreur de visee depasse le centimetre.")
        print("Ajouter des points n'y change rien, c'est la forme du modele.")
        print("Preferer plusieurs campagnes sur des fenetres etroites.")
        reponse = input("Continuer quand meme ? (o/n) : ").strip().lower()
        if reponse != "o":
            return 1

    # -----------------------------------------------------
    # Connexion
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    journal = ouvrirJournal("balayage_tete", mc, {
        "serie": serie,
        "usage": usage,
        "j4_min": j4_min, "j4_max": j4_max,
        "j5_min": j5_min, "j5_max": j5_max,
        "nb_pas": nb_pas,
        "COTE_CARREAU": COTE_CARREAU,
        "reference": reference,
    })

    print()
    print("Pose actuelle :", [round(a, 2) for a in lireAngles(mc)])

    if reference is not None:
        afficherEtatBase(mc)

    print()
    confirmation = input("Lancer le balayage ? (o/n) : ").strip().lower()

    if confirmation != "o":
        print("Balayage annule par l'operateur.")
        annoncerJournal(fermerJournal(journal, {"interrompu": "annule par l'operateur"}))
        return 1

    # -----------------------------------------------------
    # Balayage
    # -----------------------------------------------------

    for index, consigne in enumerate(grille, start=1):

        if not executerPoint(mc, index, len(grille), consigne,
                             serie, usage, journal, reference):
            print()
            print("Balayage interrompu au point", index)
            break

    print()
    print("Releves ecrits dans :", FICHIER_BALAYAGE)

    # -----------------------------------------------------
    # Ajustement immediat
    # -----------------------------------------------------

    resume = {"serie": serie, "usage": usage, "nb_points_grille": len(grille)}

    if usage == "ajustement":

        mesures = chargerMesures(usage="ajustement")

        print()
        print(len(mesures), "points d'ajustement disponibles au total.")

        if len(mesures) >= 4:

            reponse = input("Ajuster le modele maintenant ? (o/n) : ").strip().lower()

            if reponse == "o":

                etiquette = reference["robot"] if reference else input("Etiquette du robot : ").strip()
                modele = ajusterModele(mesures, etiquette, "balayage " + serie)

                if modele is not None:
                    afficherModele(modele)
                    enregistrerModele(modele)
                    resume["modele"] = {
                        "nb_points": modele["nb_points"],
                        "residus": modele["residus"],
                    }
        else:
            print("Il en faut au moins 4 pour ajuster le modele.")

    # -----------------------------------------------------
    # Journal
    # -----------------------------------------------------

    print()
    print("Remarques sur la campagne ? (point flou, quadrillage bouge, reflets...)")
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
