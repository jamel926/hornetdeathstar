"""Recoit la position du frelon par le reseau et pointe le laser dessus."""

"""
===========================================================
PROCESSUS 2, VERSION RESEAU : LE BRAS SUR LA JETSON
===========================================================

    UDP depuis le PC -> carreaux -> J4/J5 -> send_angles

Meme programme que bras_frelon.py, a une difference pres :
la derniere position vient du cable reseau (pont_reseau.py)
au lieu du fichier positions_frelon.csv.

Tout le reste est IMPORTE de bras_frelon.py, rien n'est
recopie : conversion en carreaux, referencement, boucle de
pointage, extinction du pointeur.

Seule exception : le repere camera -> bras. L'echange des
axes x/y a ete mesure au banc le 17/09 avec la camera sur le
PC ; il est applique ici par defaut (voir REPERE_RESEAU)
plutot qu'ecrit dans bras_frelon.py, qui reste inchange.

La calibration du bras et la boucle fermee sur J4/J5 restent
sur la Jetson : seule la cible voyage sur le reseau, jamais
une commande moteur.


Lancement (sur la Jetson)
-------------------------
    python abrascamera/pont_reseau.py --ecouter     # d'abord : le cable marche-t-il ?
    python abrascamera/bras_frelon_reseau.py
    python abrascamera/bras_frelon_reseau.py --sans-echange-xy   # repere de bras_frelon.py

Puis lancer camera_frelon_reseau.py sur le PC.


LE LASER
--------
Memes regles que bras_frelon.py : POINTEUR ROUGE DE CLASSE 2.
Si le cable est debranche ou si le PC s'arrete, plus rien
n'arrive : au bout de pont.AGE_MAX (1 s), le bras
s'immobilise et eteint le pointeur.
"""


import argparse
import os
import sys


DOSSIER = os.path.dirname(os.path.abspath(__file__))

if DOSSIER not in sys.path:
    sys.path.insert(0, DOSSIER)


import bras_frelon
import pont_reseau


# =========================================================
# 1. LE REPERE CAMERA -> BRAS
# =========================================================


"""
Reglage du repere valide au banc le 17/09/2026, camera USB
branchee sur le PC au-dessus du quadrillage : la camera
croise les axes, aucun des deux n'est inverse.

Resultat avec ce reglage, leurre immobile : touche au centre,
0,5 a 1 carreau sur les bords.

bras_frelon.py garde ses propres valeurs (tout a False) pour
ne pas changer le comportement de la version sur une seule
machine. Si la camera est deplacee ou tournee, c'est ici
qu'il faut regler, avec reglage_repere.py et les options
ci-dessous pour essayer sans modifier le fichier.
"""
REPERE_RESEAU = {
    "ECHANGER_XY": True,
    "INVERSER_U": False,
    "INVERSER_V": False,
}


# =========================================================
# 2. POINT D'ENTREE
# =========================================================


def lireArguments():
    """Lit les options de la ligne de commande."""

    analyseur = argparse.ArgumentParser(
        description="Pointe le laser sur la derniere position de frelon recue du PC.",
    )

    analyseur.add_argument(
        "--frequence", type=float, default=bras_frelon.FREQUENCE,
        help="cadence des consignes en Hz (defaut : %.1f)" % bras_frelon.FREQUENCE,
    )

    analyseur.add_argument(
        "--port", type=int, default=pont_reseau.PORT,
        help="port UDP d'ecoute (defaut : %d)" % pont_reseau.PORT,
    )

    analyseur.add_argument(
        "--silencieux", action="store_true",
        help="ne pas afficher chaque consigne",
    )

    """
    Options pour essayer un autre repere au banc sans modifier
    REPERE_RESEAU : un essai = un relancement avec une option
    de plus ou de moins. Une option a la fois.
    """

    analyseur.add_argument(
        "--sans-echange-xy", action="store_true",
        help="ne pas croiser les axes x/y (applique par defaut)",
    )

    analyseur.add_argument(
        "--echanger-xy", action="store_true",
        help="sans effet : l'echange est deja le defaut. Garde pour les anciennes commandes",
    )

    analyseur.add_argument(
        "--inverser-u", action="store_true",
        help="inverser le sens de u par rapport au reglage par defaut",
    )

    analyseur.add_argument(
        "--inverser-v", action="store_true",
        help="inverser le sens de v par rapport au reglage par defaut",
    )

    return analyseur.parse_args()


def appliquerReglagesRepere(options):
    """Pose dans bras_frelon le repere du reseau, modifie par les options."""

    """
    enCarreaux() relit ECHANGER_XY, INVERSER_U et INVERSER_V
    dans les globales de bras_frelon a CHAQUE appel. Les
    changer ici, avant la boucle, suffit donc.
    """

    bras_frelon.ECHANGER_XY = REPERE_RESEAU["ECHANGER_XY"]

    bras_frelon.INVERSER_U = REPERE_RESEAU["INVERSER_U"]

    bras_frelon.INVERSER_V = REPERE_RESEAU["INVERSER_V"]

    if options.sans_echange_xy:
        bras_frelon.ECHANGER_XY = False

    if options.inverser_u:
        bras_frelon.INVERSER_U = not bras_frelon.INVERSER_U

    if options.inverser_v:
        bras_frelon.INVERSER_V = not bras_frelon.INVERSER_V

    print("Repere : ECHANGER_XY=%s  INVERSER_U=%s  INVERSER_V=%s" % (
        bras_frelon.ECHANGER_XY,
        bras_frelon.INVERSER_U,
        bras_frelon.INVERSER_V,
    ))


def main():
    """Lance le suivi de la derniere position recue du PC."""

    options = lireArguments()

    appliquerReglagesRepere(options)

    """
    Le socket est ouvert AVANT le referencement de 30
    secondes : si le port est deja pris (un autre
    bras_frelon_reseau.py oublie), on le sait tout de suite
    et le bras n'a pas bouge pour rien.
    """
    try:
        recepteur = pont_reseau.RecepteurDernierePosition(options.port)

    except OSError as erreur:
        print("Impossible d'ecouter le port UDP %d : %s" % (options.port, erreur))
        print("Un autre bras_frelon_reseau.py tourne probablement encore.")
        return 1

    print("Zone calibree : %.0f x %.0f carreaux, soit %.0f x %.0f cm." % (
        bras_frelon.U_ETENDUE, bras_frelon.V_ETENDUE,
        bras_frelon.U_ETENDUE * bras_frelon.COTE_CARREAU,
        bras_frelon.V_ETENDUE * bras_frelon.COTE_CARREAU,
    ))

    contexte = bras_frelon.preparerModele()

    if contexte is None:
        print("Modele de calibration introuvable. Lancer autotest.py du v3.")
        recepteur.fermer()
        return 1

    mc, base, j6 = bras_frelon.preparerRobot(contexte)

    if mc is None:
        recepteur.fermer()
        return 1

    print()
    print("Base verrouillee :", [round(a, 2) for a in base])
    print("Ecoute UDP       : port %d" % options.port)
    print()
    print("MONTER UN POINTEUR ROUGE DE CLASSE 2, pas le laser de puissance.")
    print()

    input("Entree pour lancer le suivi (Ctrl+C pour arreter) > ")

    """
    Pendant le referencement et l'attente de la touche
    Entree, la camera a pu envoyer des positions. Elles sont
    jetees : sinon le bras demarrerait sur une cible vieille
    de 30 secondes. Voir oublier() pour pourquoi un simple
    lire() ne suffit pas.
    """
    recepteur.oublier()

    compteurs = bras_frelon.Compteurs()

    print()
    print("Suivi en cours...")
    print()

    try:
        bras_frelon.pointer(
            mc, base, j6, contexte, recepteur, compteurs,
            options.frequence,
            not options.silencieux,
        )

    except KeyboardInterrupt:
        print()
        print("Suivi interrompu.")

    finally:
        bras_frelon.eteindrePointeur(mc)

        recepteur.fermer()

    bras_frelon.afficherBilan(compteurs)

    print("  positions recues du PC    %d" % recepteur.recues)

    if recepteur.illisibles:
        print("  datagrammes illisibles    %d" % recepteur.illisibles)

    print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
