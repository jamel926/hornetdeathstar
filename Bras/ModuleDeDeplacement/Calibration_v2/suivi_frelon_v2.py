"""
=========================================================
SUIVI DE FRELON - MODELE V2
=========================================================

A quoi ca sert :
---------------
Rejouer le suivi de trajectoire de suivi_frelon.py, mais en
faisant passer la chaine de visee par le modele v2 au lieu
de Calib 2 + polynome de correction.

Tout le reste est identique : generation de trajectoire,
mode enchaine, mode flux, CSV, figure, bilan. C'est le
meme code, litteralement -- ce fichier ne le recopie pas,
il l'importe et remplace quatre fonctions.

Pourquoi un fichier separe plutot qu'une modification :
------------------------------------------------------
suivi_frelon.py est partage et relu par d'autres. On n'y
touche pas. Un suivi v2 rate ne doit rien casser du suivi
qui marche : les deux tournent cote a cote, et on compare.

Ce qui est remplace, et rien d'autre :
-------------------------------------
    construireModeleCalib2   -> charge modele_grille_v2.json
                                au lieu de reajuster Calib 2
                                sur ses 35 noeuds

    chargerModeleCorrection  -> retourne None. Le polynome
                                de degre 4 rattrapait un
                                biais du modele geometrique ;
                                ce biais n'existe plus dans
                                le v2. L'empiler corrigerait
                                deux fois.

    anglesPourPoint          -> version qui inverse aussi le
                                residu quadratique. C'est
                                convertirPosition() qui
                                l'appelle, sans le savoir.

    predireImpact            -> idem dans le sens direct.
                                C'est completerImpacts() qui
                                l'appelle, pour remplir la
                                colonne erreur_carreaux.

    verifierBase             -> meme garde-fou, mais avec la
                                tolerance reellement observee
                                sur la campagne (1,2 deg) au
                                lieu de 0,2. Voir modele_v2.

    DOSSIER_RESULTATS        -> le sous-dossier resultats/ de
                                Calibration_v2, pour ne pas
                                melanger les sorties avec
                                celles du suivi d'origine.

Ce que ca ne change pas :
------------------------
La contrepartie du suivi rapide reste entiere. Sans recul
ni stabilisation, la pose est lue en vol et le jeu des
reducteurs n'est plus repris du meme cote. Un modele plus
juste ne rachete pas ca : il enleve l'erreur de VISEE, pas
l'erreur de COMMANDE.

Autrement dit, attendre un gain spectaculaire en mode flux
serait se tromper de probleme. Le gain se verra surtout en
mode enchaine, et sur la colonne erreur_carreaux.

Utilisation :
------------
    python3 suivi_frelon_v2.py

Memes questions, memes sorties, memes modes que
suivi_frelon.py.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import sys


"""
modele_v2 en premier : c'est lui qui ajoute le dossier
parent au chemin, sans quoi suivi_frelon serait
introuvable.
"""
from modele_v2 import (
    DOSSIER_RESULTATS_V2,
    anglesPourPointCorrige,
    chargerModeleV2,
    predireImpactCorrige,
    verifierBaseV2,
)

import suivi_frelon


# =========================================================
# 2. LES REMPLACANTS
# =========================================================

def construireModeleV2():
    """Charge le modele v2 a la place de la reconstruction de Calib 2, ou None."""

    """
    suivi_frelon.main() appelle cette fonction en croyant
    reajuster Calib 2 sur ses 35 noeuds. Elle retourne a la
    place le modele v2, deja ajuste sur 112 correspondances
    et enregistre sur disque.

    Le dictionnaire a exactement la meme forme : les
    fonctions d'aval ne voient aucune difference.
    """

    modele = chargerModeleV2()

    if modele is None:
        return None

    # suivi_frelon lit cette cle dans ses messages.
    modele["serie"] = "Calib 2 v2"

    return modele


def pasDeCorrection():
    """Retourne None : le modele v2 n'a pas besoin d'un polynome par dessus."""

    print()
    print("Modele de correction : AUCUN, volontairement.")
    print("  Le polynome de degre 4 corrigeait un biais du modele")
    print("  geometrique. Le v2 est ajuste sur les tirs eux-memes,")
    print("  ce biais n'existe plus. L'appliquer corrigerait deux fois.")

    return None


# =========================================================
# 3. LE BRANCHEMENT
# =========================================================

def brancherModeleV2():
    """Remplace dans suivi_frelon les fonctions qui touchent a la calibration."""

    """
    On remplace des noms DANS LE MODULE suivi_frelon, ce qui
    suffit : convertirPosition() et completerImpacts() vont
    chercher anglesPourPoint et predireImpact dans les
    globales de leur propre module, pas dans celles d'ici.

    C'est volontairement fait en un seul endroit, visible,
    et la liste est affichee au lancement. Un remplacement
    disperse dans le fichier serait illisible pour qui relit
    le code.
    """

    suivi_frelon.construireModeleCalib2 = construireModeleV2
    suivi_frelon.chargerModeleCorrection = pasDeCorrection
    suivi_frelon.anglesPourPoint = anglesPourPointCorrige
    suivi_frelon.predireImpact = predireImpactCorrige
    suivi_frelon.verifierBase = verifierBaseV2
    suivi_frelon.DOSSIER_RESULTATS = DOSSIER_RESULTATS_V2


# =========================================================
# 4. PROGRAMME PRINCIPAL
# =========================================================

def main():
    """Branche le modele v2 puis lance le suivi de suivi_frelon."""

    print("=" * 57)
    print("SUIVI DE FRELON - MODELE V2")
    print("=" * 57)
    print()
    print("Le suivi est celui de suivi_frelon.py, inchange.")
    print("Seule la chaine de visee est remplacee :")
    print()
    print("  modele          : modele_grille_v2.json (112 points)")
    print("  correction      : aucune")
    print("  inversion       : homographie + residu quadratique")
    print("  sorties dans    : Calibration_v2/resultats/")
    print()
    print("Rappel : un modele plus juste enleve l'erreur de VISEE,")
    print("pas celle de COMMANDE. En mode flux, la pose est lue en")
    print("vol : le gros de l'ecart restera.")
    print()

    brancherModeleV2()

    return suivi_frelon.main()


if __name__ == "__main__":
    sys.exit(main())
