"""
=========================================================
TEST DE CALIBRAGE - MODELE V2
=========================================================

A quoi ca sert :
---------------
Verifier sur le vrai bras ce que vaut le modele v2, et
produire au passage des mesures qui le feront progresser.

Chaque tir ecrit une ligne dans

    resultats/tirs_v2_<horodatage>.csv

avec exactement le meme entete que le fichier de validation
d'origine. recalibration_tirs.py relit ces fichiers tout
seul : tirer, annoter, relancer l'ajustement, recommencer.

Ce qui change par rapport a correction_calib_mc_mtm.py :
-------------------------------------------------------
1. PLUS DE POLYNOME DE CORRECTION.
   Le "+1 carreau en Y" et le polynome de degre 4 de
   modele_correction_Calib_2.json rattrapaient un biais du
   modele geometrique. Ce biais n'existe plus : le modele
   v2 est ajuste sur les tirs eux-memes. Empiler une
   correction par dessus reviendrait a corriger deux fois.

2. L'IMPACT SE NOTE AU DEMI-CARREAU.
   C'est le point important de cette campagne. Les 67 tirs
   deja annotes l'ont ete au carreau ENTIER. L'arrondi seul
   vaut alors 0,41 carreau rms, soit 1,22 cm -- et c'est
   exactement ce qui reste d'erreur apres reajustement.
   Autrement dit ces mesures ne peuvent plus rien voir en
   dessous de 1,2 cm.

   Noter au demi-carreau divise ce plancher par deux. Noter
   au dixieme le divise par cinq. Sans ca, tirer cent fois
   de plus ne servira a rien.

3. LE BILAN EST AFFICHE EN DIRECT.
   Biais et dispersion apres chaque tir, pour voir tout de
   suite si le modele derive au lieu de le decouvrir en
   depouillant.

Deux modes :
-----------
    LIBRE   l'operateur tape les cibles qu'il veut.
    GRILLE  le script propose une liste de noeuds repartis
            sur toute la zone, un par un. C'est le mode a
            utiliser pour une vraie campagne : une erreur
            mesuree seulement au centre ne dit rien des
            bords.

SECURITE :
---------
Pointeur rouge classe 2. Le laser de puissance reste
DEBRANCHE, pas seulement eteint. Le pointeur ne s'allume que
pendant le releve et s'eteint quoi qu'il arrive en sortie,
Ctrl+C compris.

Utilisation :
------------
    python3 test_calibrage_v2.py
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import math
import sys

from datetime import datetime


"""
modele_v2 en premier : c'est lui qui rend calibration_grille
importable depuis ce sous-dossier.
"""
from modele_v2 import (
    DOSSIER_RESULTATS_V2,
    anglesPourPointCorrige,
    chargerModeleV2,
    enCentimetres,
    predireImpactCorrige,
    rmsEtBiais,
    verifierBaseV2,
)

"""
allerPositionInitiale est pris dans calibration_grille, pas
dans PU_PositionInitiale : ce dernier importe pymycobot des
le chargement du module, et le script deviendrait
inutilisable sur une machine sans robot. calibration_grille
ne charge pymycobot que dans connecterRobot.

C'est la meme precaution que dans suivi_frelon.py.
"""
from calibration_grille import (
    allerPositionInitiale,
    allumerPointeur,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    lireTexte,
    viserAngles,
)


# =========================================================
# 2. CONFIGURATION
# =========================================================

ENTETE_TIRS = [
    "horodatage",
    "x_demande_carreaux",
    "y_demande_carreaux",
    "J4",
    "J5",
    "x_reel_carreaux",
    "y_reel_carreaux",
    "ecart_carreaux",
]


"""
Les noeuds du mode GRILLE.

Repartis exprès sur les bords et les coins autant que sur
le centre : c'est la ou le modele est le moins contraint,
donc la ou il se trompe le plus. Une campagne concentree au
milieu donnerait un bilan flatteur et faux.
"""
NOEUDS_CAMPAGNE = [
    (0, 0), (12, 0), (24, 0),
    (0, 7), (8, 5), (16, 5), (24, 7),
    (4, 13), (12, 13), (20, 13),
    (0, 14), (24, 14),
    (6, 20), (18, 20),
    (0, 27), (12, 27), (24, 27),
    (3, 24), (21, 24),
    (9, 9), (15, 18),
]


# En dessous de ce pas de lecture, on considere que
# l'operateur a note au carreau entier.
PAS_LECTURE_ATTENDU = 0.5


# =========================================================
# 3. SAISIE
# =========================================================

def lireCouple(message):
    """Lit deux nombres separes par un espace, et retourne le couple, None si vide, ou 'fin'."""

    reponse = lireTexte(message)

    if not reponse:
        return None

    if reponse.strip().lower() in ("fin", "f", "q"):
        return "fin"

    morceaux = reponse.replace(",", " ").replace(";", " ").split()

    if len(morceaux) != 2:
        print("  Il faut deux nombres separes par un espace.")
        return None

    try:
        return (float(morceaux[0]), float(morceaux[1]))

    except ValueError:
        print("  Saisie illisible.")
        return None


def lireCible(modele):
    """Demande une cible en carreaux et la retourne, ou None pour terminer."""

    print()
    print("Cible a viser, en CARREAUX (ex : 12 13). 'fin' pour terminer.")

    while True:

        reponse = lireCouple("cible > ")

        if reponse == "fin":
            return None

        if reponse is None:
            continue

        if not dansEnveloppe(modele, reponse[0], reponse[1]):
            enveloppe = modele["enveloppe"]
            print("  Hors zone calibree : x %.1f..%.1f, y %.1f..%.1f."
                  % (enveloppe["u_min"], enveloppe["u_max"],
                     enveloppe["v_min"], enveloppe["v_max"]))
            print("  Le modele n'a jamais ete verifie la-bas, il n'y repondra pas.")
            continue

        return reponse


def lireImpact(demande):
    """Demande ou le spot est tombe et retourne le couple, ou None si le releve est saute."""

    print()
    print("  Ou le spot est-il tombe ? deux nombres en carreaux,")
    print("  AU DEMI-CARREAU au moins (ex : 12.5 13). Entree pour sauter.")

    reponse = lireCouple("  impact > ")

    if reponse in (None, "fin"):
        return None

    ecart = math.hypot(reponse[0] - demande[0], reponse[1] - demande[1])

    print("  Ecart : %.2f carreau (%.1f cm)" % (ecart, enCentimetres(ecart)))

    return reponse


# =========================================================
# 4. ENREGISTREMENT
# =========================================================

def ouvrirFichierTirs():
    """Cree le fichier de la seance avec son entete, et retourne son chemin."""

    os.makedirs(DOSSIER_RESULTATS_V2, exist_ok=True)

    chemin = os.path.join(
        DOSSIER_RESULTATS_V2,
        "tirs_v2_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"),
    )

    with open(chemin, "w", newline="") as fichier:
        csv.writer(fichier).writerow(ENTETE_TIRS)

    return chemin


def enregistrerTir(chemin, demande, angles_atteints, impact):
    """Ajoute une ligne au fichier de la seance, et retourne True si l'ecriture a reussi."""

    """
    On enregistre les angles ATTEINTS, pas la consigne.

    C'est ce qui rend la ligne reutilisable comme mesure de
    calibration : elle dit "le poignet etait la, le spot est
    tombe ici". La consigne, elle, ne dit rien sur la
    geometrie -- seulement sur ce qu'on avait demande.
    """

    if impact is None:
        ecart = ""
        x_reel = ""
        y_reel = ""

    else:
        ecart = round(math.hypot(impact[0] - demande[0], impact[1] - demande[1]), 3)
        x_reel = round(impact[0], 3)
        y_reel = round(impact[1], 3)

    try:
        with open(chemin, "a", newline="") as fichier:

            csv.writer(fichier).writerow([
                datetime.now().isoformat(timespec="seconds"),
                round(demande[0], 3),
                round(demande[1], 3),
                round(angles_atteints[0], 3),
                round(angles_atteints[1], 3),
                x_reel,
                y_reel,
                ecart,
            ])

        return True

    except Exception as e:
        print("  Impossible d'enregistrer le tir :", e)
        return False


# =========================================================
# 5. BILAN
# =========================================================

def afficherBilanCourant(tirs):
    """Affiche le bilan de la seance apres chaque tir."""

    """
    Deux chiffres a lire, pas un :

      - le BIAIS dit que le modele est decale. Il se
        rattrape, et c'est ce qu'un reajustement enleve.
      - la DISPERSION dit que le bras ne repete pas. Elle
        ne se rattrape pas par du calcul.

    Un biais qui grandit tir apres tir, c'est le modele qui
    ne convient pas. Une dispersion qui grandit, c'est la
    mecanique ou la base qui bouge.
    """

    ecarts = [
        (t["impact"][0] - t["demande"][0], t["impact"][1] - t["demande"][1])
        for t in tirs if t["impact"] is not None
    ]

    if not ecarts:
        return

    rms, biais_u, biais_v, dispersion = rmsEtBiais(ecarts)

    print()
    print("  --- seance : %d tir(s) releve(s) ---" % len(ecarts))
    print("  erreur moyenne (rms) : %.2f carreau (%.1f cm)" % (rms, enCentimetres(rms)))
    print("  biais                : x %+.2f   y %+.2f" % (biais_u, biais_v))
    print("  dispersion           : %.2f carreau (%.1f cm)"
          % (dispersion, enCentimetres(dispersion)))


def verifierFinesseDeLecture(tirs):
    """Previent si tous les impacts ont ete notes au carreau entier."""

    valeurs = []

    for t in tirs:
        if t["impact"] is not None:
            valeurs += [t["impact"][0], t["impact"][1]]

    if len(valeurs) < 6:
        return

    if all(abs(v - round(v)) < 1e-9 for v in valeurs):
        print()
        print("ATTENTION : tous les impacts ont ete notes au CARREAU ENTIER.")
        print("L'arrondi vaut alors 0,41 carreau rms (1,22 cm) a lui seul,")
        print("soit l'ordre de grandeur de l'erreur qu'on cherche a mesurer.")
        print("Cette seance confirmera le modele mais ne l'ameliorera pas.")
        print("Noter au demi-carreau au moins la prochaine fois.")


def afficherBilanFinal(tirs, chemin):
    """Affiche le bilan de fin de seance et rappelle la suite."""

    releves = [t for t in tirs if t["impact"] is not None]

    print()
    print("=" * 57)
    print("BILAN DE LA SEANCE")
    print("=" * 57)
    print()
    print("  tirs effectues : %d" % len(tirs))
    print("  tirs releves   : %d" % len(releves))

    afficherBilanCourant(tirs)

    # -----------------------------------------------------
    # Ce que le modele avait annonce
    # -----------------------------------------------------

    """
    Comparer l'erreur mesuree a l'erreur annoncee est le
    seul vrai test. Si la seance sort nettement au dessus de
    1,3 cm, ce n'est pas le modele qui se trompe : c'est que
    quelque chose a change sur le bras ou sur le quadrillage.
    """

    if releves:
        print()
        print("  Pour reference, le modele v2 annonce 0,43 carreau (1,3 cm) rms.")
        print("  Nettement au dessus : la base a bouge, ou le quadrillage a")
        print("  ete deplace, ou le pointeur n'est plus au meme endroit.")

    verifierFinesseDeLecture(tirs)

    print()
    print("  Fichier : %s" % chemin)
    print()
    print("  Pour integrer cette seance au modele :")
    print("      python3 recalibration_tirs.py")


# =========================================================
# 6. UN TIR
# =========================================================

def effectuerTir(mc, modele, base, j6, cible):
    """Vise la cible, demande l'impact, et retourne le releve, ou None si le tir a echoue."""

    resultat = anglesPourPointCorrige(modele, cible[0], cible[1])

    if resultat is None:
        print("  Le modele ne donne pas d'angles pour ce point.")
        return None

    j4, j5 = resultat

    print("  Angles vises : J4 %+.2f   J5 %+.2f" % (j4, j5))

    vise, pose = viserAngles(mc, base, j4, j5, j6)

    if pose is None:
        print("  Mouvement ou lecture impossible, tir abandonne.")
        return None

    if not vise:
        print("  (angles hors tolerance : le releve reste valable,")
        print("   ce sont les angles ATTEINTS qui seront enregistres)")

    atteints = (pose[3], pose[4])

    # -----------------------------------------------------
    # Ou le modele croit-il pointer ?
    # -----------------------------------------------------

    """
    Le modele repasse les angles LUS dans le sens direct.
    L'ecart avec la cible dit ce que coute le fait de ne pas
    atteindre exactement les angles demandes -- une erreur
    de commande, pas une erreur de modele. Les deux se
    melangent dans l'impact mesure, autant les separer ici.
    """

    prevu = predireImpactCorrige(modele, atteints[0], atteints[1])

    if prevu is not None:

        derive = math.hypot(prevu[0] - cible[0], prevu[1] - cible[1])

        print("  Angles atteints : J4 %+.2f   J5 %+.2f" % atteints)
        print("  Le modele place le spot en (%.2f, %.2f), soit %.2f carreau"
              % (prevu[0], prevu[1], derive))
        print("  de la cible du seul fait des angles non atteints.")

    impact = lireImpact(cible)

    return {
        "demande": cible,
        "angles": atteints,
        "impact": impact,
    }


# =========================================================
# 7. PROGRAMME PRINCIPAL
# =========================================================

def choisirMode():
    """Demande le mode de seance et retourne 'grille' ou 'libre'."""

    print()
    print("Mode de seance :")
    print("  1  GRILLE  %d noeuds repartis sur toute la zone (recommande)"
          % len(NOEUDS_CAMPAGNE))
    print("  2  LIBRE   cibles tapees a la main")

    reponse = lireTexte("choix [1] > ", "1").strip()

    return "libre" if reponse == "2" else "grille"


def main():
    """Deroule une seance de test du modele v2 et enregistre ses tirs."""

    print("=" * 57)
    print("TEST DE CALIBRAGE - MODELE V2")
    print("=" * 57)
    print()
    print("SECURITE : pointeur rouge classe 2 uniquement.")
    print("Le laser de puissance doit etre DEBRANCHE.")
    print()

    # -----------------------------------------------------
    # Modele
    # -----------------------------------------------------

    modele = chargerModeleV2()

    if modele is None:
        return 1

    print()
    print("Aucun polynome de correction n'est applique : le modele v2")
    print("est deja ajuste sur les tirs, en rajouter un corrigerait deux fois.")

    mode = choisirMode()

    # -----------------------------------------------------
    # Robot
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        return 1

    allerPositionInitiale(mc)

    if not verifierBaseV2(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    print()
    print("Base verrouillee :", [round(a, 2) for a in base])

    # -----------------------------------------------------
    # Seance
    # -----------------------------------------------------

    chemin = ouvrirFichierTirs()

    print()
    print("Seance enregistree dans :", os.path.basename(chemin))

    tirs = []

    try:

        allumerPointeur(mc)

        if mode == "grille":

            for numero, cible in enumerate(NOEUDS_CAMPAGNE, start=1):

                print()
                print("-" * 57)
                print("Noeud %d / %d : cible (%g, %g)"
                      % (numero, len(NOEUDS_CAMPAGNE), cible[0], cible[1]))

                suite = lireTexte("Entree pour tirer, 's' pour sauter, 'fin' pour arreter > ", "")

                if suite.strip().lower() in ("fin", "f", "q"):
                    break

                if suite.strip().lower() in ("s", "saut"):
                    continue

                releve = effectuerTir(mc, modele, base, j6, cible)

                if releve is None:
                    continue

                tirs.append(releve)
                enregistrerTir(chemin, releve["demande"], releve["angles"], releve["impact"])
                afficherBilanCourant(tirs)

        else:

            while True:

                cible = lireCible(modele)

                if cible is None:
                    break

                print()
                print("-" * 57)

                releve = effectuerTir(mc, modele, base, j6, cible)

                if releve is None:
                    continue

                tirs.append(releve)
                enregistrerTir(chemin, releve["demande"], releve["angles"], releve["impact"])
                afficherBilanCourant(tirs)

    except KeyboardInterrupt:
        print()
        print("Seance interrompue par l'operateur.")

    finally:

        # Le pointeur s'eteint quoi qu'il arrive, Ctrl+C et
        # exception comprises.
        eteindrePointeur(mc)
        print()
        print("Pointeur eteint.")

    afficherBilanFinal(tirs, chemin)

    return 0


if __name__ == "__main__":
    sys.exit(main())
