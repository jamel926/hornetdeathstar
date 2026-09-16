"""
=========================================================
REAJUSTEMENT V3 - TOUTES LES SEANCES
=========================================================

Meme ajustement que recalibration_tirs.py du v2, mais en
relisant AUSSI les seances faites avec test_calibrage_v3.py.

Pourquoi les seances v3 sont ajustables comme les autres :
---------------------------------------------------------
Parce qu'un tir enregistre les angles ATTEINTS, jamais la
consigne. La ligne dit "le poignet etait la, le spot est
tombe ici" : c'est une mesure de GEOMETRIE, et elle ne
depend pas du decalage de commande applique ce jour la.

C'est la meme propriete qui avait permis de recycler les 67
tirs de validation du v2. Elle tient toujours.

Ce qui est relu :
----------------
    resultats/calibration_grille.csv                 noeuds
    resultats/validation_pointage_Calib_2.csv        tirs d'origine
    resultats/tirs_v2_*.csv                          seances v2
    resultats/tirs_v3_*.csv                          seances v3

Ce qui est ecrit :
-----------------
    resultats/modele_grille_v3.json
    resultats/grille_angles_v3.csv

Le modele v2 n'est pas touche : modele_grille_v2.json reste
tel quel, le v3 ecrit dans son propre fichier.

Utilisation :
------------
    python3 reajuster_v3.py
    python3 reajuster_v3.py --diagnostic    # sans rien ecrire
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import glob
import sys


from modele_v3 import DOSSIER_RESULTATS_V3

import recalibration_tirs


# =========================================================
# 2. CONFIGURATION
# =========================================================

MOTIF_TIRS_V3 = os.path.join(DOSSIER_RESULTATS_V3, "tirs_v3_*.csv")


FICHIER_MODELE_V3 = os.path.join(DOSSIER_RESULTATS_V3, "modele_grille_v3.json")


FICHIER_GRILLE_V3 = os.path.join(DOSSIER_RESULTATS_V3, "grille_angles_v3.csv")


# =========================================================
# 3. LE REMPLACANT
# =========================================================

def chargerToutesLesSeances():
    """Lit les tirs d'origine plus toutes les seances v2 et v3, et retourne la liste."""

    """
    Meme fonction que celle du v2, avec un motif de plus. On
    la remplace entierement plutot que de bricoler son motif :
    le v2 n'en connait qu'un, et lui en substituer un autre
    ferait perdre ses propres seances.
    """

    tirs = []

    if os.path.exists(recalibration_tirs.FICHIER_TIRS):
        tirs += recalibration_tirs.chargerTirs(recalibration_tirs.FICHIER_TIRS)

    for motif, etiquette in ((recalibration_tirs.MOTIF_TIRS_V2, "v2"),
                             (MOTIF_TIRS_V3, "v3")):

        for chemin in sorted(glob.glob(motif)):

            ajout = recalibration_tirs.chargerTirs(chemin)

            if ajout:
                print("  seance %s relue : %s (%d tirs)"
                      % (etiquette, os.path.basename(chemin), len(ajout)))
                tirs += ajout

    return tirs


# =========================================================
# 4. LE BRANCHEMENT
# =========================================================

def brancherV3():
    """Remplace dans recalibration_tirs la lecture des tirs et les chemins de sortie."""

    recalibration_tirs.chargerTousLesTirs = chargerToutesLesSeances
    recalibration_tirs.FICHIER_MODELE_NEUF = FICHIER_MODELE_V3
    recalibration_tirs.FICHIER_GRILLE_ANGLES = FICHIER_GRILLE_V3


# =========================================================
# 5. PROGRAMME PRINCIPAL
# =========================================================

def main():
    """Branche les chemins v3 puis lance l'ajustement de recalibration_tirs."""

    print("=" * 57)
    print("REAJUSTEMENT V3")
    print("=" * 57)
    print()
    print("Ajustement du v2, elargi aux seances v3.")
    print("Le modele v2 n'est pas touche.")
    print()

    brancherV3()

    return recalibration_tirs.main()


if __name__ == "__main__":
    sys.exit(main())
