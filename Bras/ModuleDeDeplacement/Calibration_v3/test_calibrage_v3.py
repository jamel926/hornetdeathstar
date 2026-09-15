"""
=========================================================
TEST DE CALIBRAGE - MODELE V3
=========================================================

Ce que le v3 change par rapport au v2 :
--------------------------------------
Une seule chose : la consigne envoyee au bras est DECALEE du
manque mesure en debut de seance.

    cible (carreaux)
      -> anglesPourPointCorrige()   geometrie, modele v2
      -> compenserCommande()        manque de la commande
      -> viserAngles()

Le modele geometrique n'est pas touche. Il n'avait rien a se
reprocher : la seance du 15/09 lui a donne 1,19 cm rms
depuis les angles reellement atteints, conforme a ce qu'il
annonce. Les 2,54 cm venaient de ce que le bras s'arretait
1,25 deg sous ses consignes sur J4.

Pourquoi ce fichier ne recopie pas le v2 :
-----------------------------------------
Il l'importe et remplace trois fonctions. La campagne, la
saisie au demi-carreau, le bilan en direct, le format du CSV
sont ceux du v2, inchanges -- donc corriges une seule fois
le jour ou il y aura un bug dedans.

    verifierBaseV2        -> verifie la base PUIS mesure le
                             decalage de commande et le
                             garde pour la seance

    anglesPourPointCorrige -> meme geometrie, suivie de la
                             compensation. C'est effectuerTir
                             qui l'appelle, sans le savoir.

    ouvrirFichierTirs     -> ecrit dans Calibration_v3 sous
                             le nom tirs_v3_<horodatage>.csv

predireImpactCorrige n'est PAS touche : il travaille sur les
angles atteints, le compenser fausserait le diagnostic
affiche a chaque tir.

Ce que le CSV garde :
--------------------
Exactement le meme entete, et toujours les angles ATTEINTS.
La correspondance (angles, impact) reste vraie quel que soit
le decalage applique ce jour la, donc les seances v3
continuent de nourrir l'ajustement du modele comme les
seances v2. C'est reajuster_v3.py qui les relit.

Ce qu'on attend de cette seance :
--------------------------------
    sans compensation (15/09)   0,845 carreau   2,54 cm rms
    avec compensation           ~0,41 carreau   ~1,2 cm rms

et surtout : le biais en y doit cesser de GRANDIR avec y.
Aujourd'hui il passe de -0,30 en bas a -1,10 au fond, ce qui
est la signature d'un manque angulaire constant vu a travers
une sensibilite qui triple. Si cette pente disparait, le
diagnostic est confirme. Si elle reste, il faut chercher
ailleurs.

SECURITE :
---------
Pointeur rouge classe 2 uniquement, laser de puissance
DEBRANCHE. Le referencement se fait pointeur eteint.

Utilisation :
------------
    python3 test_calibrage_v3.py
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import math
import sys

from datetime import datetime


from modele_v3 import (
    DOSSIER_RESULTATS_V3,
    anglesPourPointCorrige,
    compenserCommande,
    enCentimetres,
    resumerDecalage,
    sensibilite,
    verifierBaseV2,
)

from referencement import referencer

from calibration_grille import lireAngles

import test_calibrage_v2.py


# =========================================================
# 2. LE DECALAGE DE LA SEANCE
# =========================================================

"""
Un seul decalage pour toute la seance, mesure une fois au
demarrage. Il est garde ici plutot que passe de fonction en
fonction : les fonctions du v2 qu'on reutilise ont leur
signature, on ne va pas la changer pour ca.
"""
DECALAGE = {"valeur": None}


def decalageCourant():
    """Retourne le decalage mesure pour cette seance, ou None s'il n'a pas ete mesure."""

    return DECALAGE["valeur"]


# =========================================================
# 3. LES REMPLACANTS
# =========================================================

def verifierBaseEtReferencer(mc, modele, tolerance=None):
    """Verifie la pose de base, puis mesure le decalage de commande. Retourne True si tout tient."""

    """
    Le referencement est branche ICI parce que c'est le seul
    point du deroulement du v2 ou le robot est pret, la base
    verifiee, et rien encore commande. L'ordre compte : mesurer
    le manque avant de savoir si la base est bonne n'aurait
    aucun sens, le manque serait celui d'une autre geometrie.
    """

    if not verifierBaseV2(mc, modele, tolerance):
        return False

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible, referencement abandonne.")
        return False

    decalage = referencer(mc, angles[0:3], angles[5], modele)

    if decalage is None:
        print()
        print("REFUS : sans referencement, le v3 ne vaut pas mieux que le v2.")
        print("Relancer, ou utiliser Calibration_v2 si le bras ne repond pas.")
        return False

    DECALAGE["valeur"] = decalage

    return True


def anglesCompenses(modele, u, v, marge=None):
    """Retourne la consigne a envoyer pour viser le point demande, decalage compris, ou None."""

    resultat = anglesPourPointCorrige(modele, u, v, marge)

    if resultat is None:
        return None

    return compenserCommande(resultat[0], resultat[1], decalageCourant())


def ouvrirFichierTirsV3():
    """Cree le fichier de la seance v3 avec son entete, et retourne son chemin."""

    os.makedirs(DOSSIER_RESULTATS_V3, exist_ok=True)

    chemin = os.path.join(
        DOSSIER_RESULTATS_V3,
        "tirs_v3_%s.csv" % datetime.now().strftime("%Y%m%d_%H%M%S"),
    )

    with open(chemin, "w", newline="") as fichier:
        csv.writer(fichier).writerow(test_calibrage_v2.ENTETE_TIRS)

    return chemin


# =========================================================
# 4. BILAN : LA COMPENSATION A-T-ELLE TENU ?
# =========================================================

def verifierCompensation(chemin, modele):
    """Relit la seance et dit ce qui reste du manque angulaire apres compensation."""

    """
    Le test qui compte. Le CSV contient la cible et les angles
    ATTEINTS ; la geometrie donne les angles qu'il AURAIT fallu
    atteindre. La difference est le manque RESIDUEL.

    S'il est retombe pres de zero, la compensation a fait son
    travail et ce qui reste d'erreur au sol vient du modele ou
    de la lecture. S'il vaut encore un degre, le decalage
    mesure au demarrage ne valait pas pour la seance -- le
    bras a derive, ou le referencement a mal mesure.
    """

    try:
        lignes = list(csv.DictReader(open(chemin)))

    except Exception:
        return

    restes = []

    for ligne in lignes:

        try:
            cible_u = float(ligne["x_demande_carreaux"])
            cible_v = float(ligne["y_demande_carreaux"])
            atteint_j4 = float(ligne["J4"])
            atteint_j5 = float(ligne["J5"])

        except (KeyError, TypeError, ValueError):
            continue

        voulu = anglesPourPointCorrige(modele, cible_u, cible_v)

        if voulu is None:
            continue

        restes.append((voulu[0] - atteint_j4, voulu[1] - atteint_j5))

    if len(restes) < 3:
        return

    nombre = len(restes)

    moyenne_j4 = sum(r[0] for r in restes) / nombre
    moyenne_j5 = sum(r[1] for r in restes) / nombre

    decalage = decalageCourant()

    print()
    print("-" * 57)
    print("LA COMPENSATION A-T-ELLE TENU ?")
    print("-" * 57)
    print()

    if decalage is not None:
        print("  decalage applique   : J4 %+.3f   J5 %+.3f deg"
              % (decalage["j4"], decalage["j5"]))

    print("  manque residuel     : J4 %+.3f   J5 %+.3f deg" % (moyenne_j4, moyenne_j5))
    print("  (mesure du 15/09 sans compensation : J4 +1,248)")

    if abs(moyenne_j4) < 0.25:
        print()
        print("  Le manque est absorbe. Ce qui reste d'erreur au sol vient")
        print("  du modele et de la lecture, plus de la commande.")

    else:
        print()
        print("  Il reste %+.2f deg sur J4, soit environ %.2f carreau au fond"
              % (moyenne_j4, abs(moyenne_j4) * 1.02))
        print("  de la zone. Le decalage mesure au demarrage ne valait pas")
        print("  pour la seance : soit le bras a derive en cours de route,")
        print("  soit le referencement est tombe sur un mauvais moment.")
        print("  Relancer referencement.py pour comparer.")


def bilanParBandeDeY(chemin):
    """Affiche le biais par bande de y : la pente est la signature du manque angulaire."""

    """
    C'est LE graphique de cette seance, en trois lignes de
    texte. Un manque angulaire constant se voit comme un biais
    en y qui grandit avec y, parce que la sensibilite triple
    entre le bas et le fond de la zone. Si la compensation a
    marche, les quatre bandes doivent se ressembler.
    """

    try:
        lignes = list(csv.DictReader(open(chemin)))

    except Exception:
        return

    tirs = []

    for ligne in lignes:
        try:
            tirs.append((
                float(ligne["x_demande_carreaux"]), float(ligne["y_demande_carreaux"]),
                float(ligne["x_reel_carreaux"]), float(ligne["y_reel_carreaux"]),
            ))
        except (KeyError, TypeError, ValueError):
            continue

    if len(tirs) < 4:
        return

    print()
    print("  biais par bande de y   (le 15/09 : -0,30 en bas, -1,10 au fond)")
    print("    %-10s %2s  %-10s %-10s" % ("bande", "n", "biais x", "biais y"))

    for bas, haut in [(0, 6), (6, 15), (15, 22), (22, 28)]:

        bande = [t for t in tirs if bas <= t[1] < haut]

        if not bande:
            continue

        print("    y %2d-%-6d %2d  %+.3f     %+.3f"
              % (bas, haut - 1, len(bande),
                 sum(t[2] - t[0] for t in bande) / len(bande),
                 sum(t[3] - t[1] for t in bande) / len(bande)))


def bilanFinalV3(tirs, chemin):
    """Affiche le bilan du v2, puis ce qui est propre au v3."""

    test_calibrage_v2.afficherBilanFinal(tirs, chemin)

    modele = test_calibrage_v2.chargerModeleV2(silencieux=True)

    if modele is None:
        return

    bilanParBandeDeY(chemin)
    verifierCompensation(chemin, modele)

    print()
    print("  Pour integrer cette seance au modele :")
    print("      python3 reajuster_v3.py")


# =========================================================
# 5. LE BRANCHEMENT
# =========================================================

def brancherV3():
    """Remplace dans test_calibrage_v2 les fonctions qui touchent a la commande."""

    """
    Trois remplacements, listes au lancement. Le reste du
    deroulement -- campagne, saisie, enregistrement, bilan en
    direct -- est celui du v2 et n'est pas touche.
    """

    test_calibrage_v2.verifierBaseV2 = verifierBaseEtReferencer
    test_calibrage_v2.anglesPourPointCorrige = anglesCompenses
    test_calibrage_v2.ouvrirFichierTirs = ouvrirFichierTirsV3
    test_calibrage_v2.afficherBilanFinal = bilanFinalV3


# =========================================================
# 6. PROGRAMME PRINCIPAL
# =========================================================

def main():
    """Branche la compensation puis lance la campagne du v2."""

    print("=" * 57)
    print("TEST DE CALIBRAGE - MODELE V3")
    print("=" * 57)
    print()
    print("Campagne identique au v2. Seule la consigne change :")
    print("elle est decalee du manque mesure en debut de seance.")
    print()
    print("  geometrie       : modele v2, inchange")
    print("  compensation    : mesuree au demarrage, pas figee")
    print("  sorties dans    : Calibration_v3/resultats/")
    print()
    print("Attendu : ~1,2 cm rms contre 2,54 cm le 15/09, et")
    print("surtout un biais en y qui cesse de grandir avec y.")
    print()

    brancherV3()

    return test_calibrage_v2.main()


if __name__ == "__main__":
    sys.exit(main())
