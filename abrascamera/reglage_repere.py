"""Outil de calage du repere camera sur le quadrillage du bras."""

"""
===========================================================
REGLER LE REPERE, UNE FOIS
===========================================================

Personne n'a jamais mesure comment le repere de la camera
est pose par rapport au quadrillage du bras : quel coin de
l'image correspond au carreau (0, 0), et dans quel sens
tournent les axes. Tant que ce n'est pas fait, le laser
pointe a cote.

Cet outil sert a le mesurer en quelques minutes, sans
tatonner dans le code.


Deux modes
----------
    --hors-ligne
        Aucune camera, aucun robot. Affiche ce que donnent
        quelques positions typiques avec les reglages
        actuels. A lancer en premier pour verifier que la
        conversion elle-meme tient debout.

    (sans option)
        Suit le journal ecrit par camera_frelon.py et
        affiche, pour chaque position recue, les carreaux et
        les angles correspondants. C'est le mode a utiliser
        au banc.


Methode au banc
---------------
1. Lancer camera_frelon.py d'un cote, cet outil de l'autre.
2. Poser un leurre dans le coin du quadrillage que le bras
   appelle (0, 0).
3. Lire ce qu'affiche cet outil.
      - s'il affiche environ (0, 0) carreaux : rien a faire ;
      - s'il affiche le coin oppose : les deux inversions
        sont a lever ;
      - s'il affiche un coin adjacent : une seule.
4. Modifier le drapeau correspondant en tete de
   bras_frelon.py, relancer, verifier.
5. Recommencer avec le leurre au centre, puis dans un autre
   coin.

Un drapeau a la fois, et on verifie entre chaque. Changer
les trois ensemble et regarder si "ca marche mieux" est le
moyen le plus sur de tourner en rond.
"""


import argparse
import os
import sys
import time


DOSSIER = os.path.dirname(os.path.abspath(__file__))

RACINE = os.path.dirname(DOSSIER)

CALIBRATION_V3 = os.path.join(
    RACINE,
    "Bras", "ModuleDeDeplacement", "Calibration_v3",
)


for chemin in (CALIBRATION_V3, DOSSIER):
    if chemin not in sys.path:
        sys.path.insert(0, chemin)


import bras_frelon
import pont

import suivi_frelon


# =========================================================
# 1. LE MODELE, SANS ROBOT
# =========================================================


def chargerContexte():
    """Charge le modele de calibration v3 sans toucher au robot."""

    """
    Le modele se charge et se calcule entierement hors ligne
    : ce sont des coefficients dans un JSON. Seul l'envoi des
    consignes a besoin du bras. C'est ce qui permet de
    regler le repere assis a son bureau.
    """

    return bras_frelon.preparerModele()


def decrire(contexte, x_pct, y_pct):
    """Retourne une ligne lisible pour une position en pourcentage."""

    u, v = bras_frelon.enCarreaux(x_pct, y_pct)

    consigne = suivi_frelon.convertirPosition(contexte, u, v)

    if consigne is None:
        return "%6.2f %% %6.2f %%  ->  %5.1f %5.1f carreaux  ->  HORS ZONE" % (
            x_pct, y_pct, u, v,
        )

    return "%6.2f %% %6.2f %%  ->  %5.1f %5.1f carreaux  ->  J4 %+7.2f  J5 %+7.2f" % (
        x_pct, y_pct, u, v,
        consigne["j4"], consigne["j5"],
    )


# =========================================================
# 2. MODE HORS LIGNE
# =========================================================


def horsLigne(contexte):
    """Affiche la conversion de quelques positions typiques."""

    modele = contexte["modele"]

    print()
    print("Reglages actuels de bras_frelon.py")
    print("-" * 57)
    print("  ECHANGER_XY  %s" % bras_frelon.ECHANGER_XY)
    print("  INVERSER_U   %s" % bras_frelon.INVERSER_U)
    print("  INVERSER_V   %s" % bras_frelon.INVERSER_V)
    print()
    print("  zone calibree du modele : u %.1f a %.1f, v %.1f a %.1f carreaux" % (
        modele["enveloppe"]["u_min"], modele["enveloppe"]["u_max"],
        modele["enveloppe"]["v_min"], modele["enveloppe"]["v_max"],
    ))

    print()
    print("Conversion des cinq points de reference")
    print("-" * 57)
    print()

    reperes = [
        ("origine camera   ", 0.0, 0.0),
        ("coin x           ", 100.0, 0.0),
        ("centre           ", 50.0, 50.0),
        ("coin y           ", 0.0, 100.0),
        ("coin oppose      ", 100.0, 100.0),
    ]

    for nom, x_pct, y_pct in reperes:
        print("  %s %s" % (nom, decrire(contexte, x_pct, y_pct)))

    print()
    print("Un HORS ZONE aux coins est normal : le quadrillage calibre")
    print("n'est pas exactement la zone vue par la camera. Un HORS ZONE")
    print("au CENTRE, en revanche, veut dire que le reglage est faux.")
    print()

    return 0


# =========================================================
# 3. MODE SUIVI DU JOURNAL
# =========================================================


def suivreJournal(contexte, chemin, frequence):
    """Affiche en continu la conversion de ce que la camera ecrit."""

    lecteur = pont.LecteurDernierePosition(chemin)

    periode = 1.0 / frequence

    print()
    print("Journal suivi : %s" % chemin)
    print()
    print("Poser un leurre dans un coin connu du quadrillage,")
    print("et comparer les carreaux affiches a ce que dit le bras.")
    print()
    print("Ctrl+C pour arreter.")
    print()

    precedent = None

    try:

        while True:

            time.sleep(periode)

            position = lecteur.lire()

            if position is None:
                continue

            x_pct, y_pct, confiance, age = position

            if age > pont.AGE_MAX:
                continue

            """
            On n'affiche que les positions qui changent :
            sinon une cible immobile remplit l'ecran de la
            meme ligne dix fois par seconde et on ne voit
            plus rien.
            """
            actuel = (round(x_pct, 1), round(y_pct, 1))

            if actuel == precedent:
                continue

            precedent = actuel

            print("  %s   conf %.2f" % (
                decrire(contexte, x_pct, y_pct),
                confiance,
            ))

    except KeyboardInterrupt:
        print()
        print("Arret.")

    return 0


# =========================================================
# 4. POINT D'ENTREE
# =========================================================


def main():
    """Lance l'outil de reglage dans le mode demande."""

    analyseur = argparse.ArgumentParser(
        description="Regle le repere camera sur le quadrillage du bras.",
    )

    analyseur.add_argument(
        "--hors-ligne", action="store_true",
        help="afficher la conversion de points types, sans camera ni robot",
    )

    analyseur.add_argument(
        "--journal", default=pont.CHEMIN_POSITIONS,
        help="fichier de positions a suivre",
    )

    analyseur.add_argument(
        "--frequence", type=float, default=10.0,
        help="cadence de lecture en Hz (defaut : 10)",
    )

    options = analyseur.parse_args()

    contexte = chargerContexte()

    if contexte is None:
        print("Modele de calibration introuvable.")
        print("Lancer Bras/ModuleDeDeplacement/Calibration_v3/autotest.py.")
        return 1

    if options.hors_ligne:
        return horsLigne(contexte)

    return suivreJournal(contexte, options.journal, options.frequence)


if __name__ == "__main__":
    raise SystemExit(main())
