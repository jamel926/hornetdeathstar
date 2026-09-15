"""
=========================================================
SUIVI DE FRELON - MODELE V3
=========================================================

Le suivi du v2, plus la compensation du manque de commande.

Empilement :
-----------
    suivi_frelon.py        le suivi, jamais touche
      + suivi_frelon_v2    modele v2 au lieu de Calib 2 + polynome
      + ce fichier         consigne decalee du manque mesure

Chaque etage remplace des noms dans le module du dessous, en
un seul endroit, affiche au lancement.

Ce que ca corrige :
------------------
Sur les six relevees de suivi disponibles, v1 comprise, le
bras s'arrete sous sa consigne : J4_consigne - J4_atteint
vaut +0,66 a +1,10 deg selon la seance. Le suivi tirait donc
systematiquement court, et personne ne le voyait parce que
la colonne erreur_carreaux ne mesure pas ca.

A quoi s'attendre, et surtout a quoi NE PAS s'attendre :
-------------------------------------------------------
Le gain portera sur la PRECISION DE POINTAGE, qui n'est pas
mesuree par le CSV. Il ne portera pas sur erreur_carreaux en
mode flux : la pose y est lue en vol, sans recul ni
stabilisation, et un decalage de consigne n'y change rien.

Autrement dit, si erreur_carreaux ne bouge pas, ce n'est pas
un echec. Pour voir le gain il faut le spot et le
quadrillage, c'est-a-dire test_calibrage_v3.py.

Lecture du CSV apres compensation :
----------------------------------
La colonne ecart_J4 vaudra desormais environ le decalage
applique (~1 deg), et ce n'est PAS une degradation : la
consigne a ete volontairement decalee, le bras la manque
d'autant, et il arrive donc au bon endroit. Comparer ce
chiffre a celui des relevees v1/v2 n'a plus de sens.

Le chiffre qui reste comparable est erreur_carreaux, qui
repasse les angles ATTEINTS dans le modele -- sans
compensation, volontairement.

Utilisation :
------------
    python3 suivi_frelon_v3.py

Memes questions, memes modes, memes sorties que
suivi_frelon.py. Les fichiers atterrissent dans
Calibration_v3/resultats/.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import sys


from modele_v3 import (
    DOSSIER_RESULTATS_V3,
    anglesPourPointCorrige,
    compenserCommande,
    verifierBaseV2,
)

from referencement import referencer

from calibration_grille import lireAngles

import suivi_frelon
import suivi_frelon_v2


# =========================================================
# 2. LE DECALAGE DE LA SEANCE
# =========================================================

DECALAGE = {"valeur": None}


def decalageCourant():
    """Retourne le decalage mesure pour cette seance, ou None."""

    return DECALAGE["valeur"]


# =========================================================
# 3. LES REMPLACANTS
# =========================================================

def verifierBaseEtReferencer(mc, modele, tolerance=None):
    """Verifie la pose de base, puis mesure le decalage de commande. Retourne True si tout tient."""

    """
    Branche ici parce que suivi_frelon.main() appelle
    verifierBase au seul moment ou le robot est pret, la base
    verrouillee, et rien encore commande -- exactement ce
    qu'il faut pour mesurer un manque qui vaut pour la suite.
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
        return False

    DECALAGE["valeur"] = decalage

    return True


def anglesCompenses(modele, u, v, marge=None):
    """Retourne la consigne a envoyer pour viser le point demande, decalage compris, ou None."""

    """
    C'est convertirPosition() qui appelle cette fonction, en
    croyant appeler anglesPourPoint. Elle est sur le chemin
    chaud du suivi : quelques microsecondes de plus par
    point, negligeable devant les ~30 ms d'un aller-retour
    serie.
    """

    resultat = anglesPourPointCorrige(modele, u, v, marge)

    if resultat is None:
        return None

    return compenserCommande(resultat[0], resultat[1], decalageCourant())


# =========================================================
# 4. LE BRANCHEMENT
# =========================================================

def brancherV3():
    """Empile la compensation du v3 par dessus les remplacements du v2."""

    """
    L'ordre compte : le v2 pose le modele et la correction, le
    v3 vient ensuite decaler la consigne. Inverser ecraserait
    la compensation.
    """

    suivi_frelon_v2.brancherModeleV2()

    suivi_frelon.verifierBase = verifierBaseEtReferencer
    suivi_frelon.anglesPourPoint = anglesCompenses
    suivi_frelon.DOSSIER_RESULTATS = DOSSIER_RESULTATS_V3


# =========================================================
# 5. PROGRAMME PRINCIPAL
# =========================================================

def main():
    """Branche le modele v3 puis lance le suivi de suivi_frelon."""

    print("=" * 57)
    print("SUIVI DE FRELON - MODELE V3")
    print("=" * 57)
    print()
    print("Le suivi est celui de suivi_frelon.py, inchange.")
    print("Deux etages par dessus :")
    print()
    print("  modele          : modele_grille_v2.json (v2)")
    print("  correction       : aucune")
    print("  compensation    : manque de commande, mesure au demarrage")
    print("  sorties dans    : Calibration_v3/resultats/")
    print()
    print("Rappel : le gain porte sur la precision de POINTAGE, que ce")
    print("CSV ne mesure pas. Si erreur_carreaux ne bouge pas, ce n'est")
    print("pas un echec -- c'est test_calibrage_v3.py qui tranche.")
    print()
    print("ecart_J4 vaudra desormais ~le decalage applique : c'est voulu.")
    print()

    brancherV3()

    return suivi_frelon.main()


if __name__ == "__main__":
    sys.exit(main())
