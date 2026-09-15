"""
=========================================================
AUTO-TEST DE L'INSTALLATION V3
=========================================================

A lancer quand un script du dossier refuse de demarrer :

    python3 autotest.py

Ce fichier ne depend de RIEN : ni de modele_v3, ni du v2, ni
de quoi que ce soit hors bibliotheque standard. C'est la
condition pour qu'il tourne encore quand tout le reste est
casse -- autrement dit exactement quand on en a besoin.

Ce qu'il separe :
----------------
Un "ModuleNotFoundError: No module named 'test_calibrage_v2'"
peut vouloir dire trois choses tres differentes :

    1. le dossier Calibration_v2 n'est pas la (git pull ?) ;
    2. il est la mais ailleurs que la ou on le cherche ;
    3. il est la, mais une erreur A L'INTERIEUR de
       test_calibrage_v2 empeche son chargement.

Le message de Python ne les distingue pas. Celui-ci si : il
regarde d'abord les fichiers sur le disque, puis tente chaque
import separement en affichant la vraie exception.
"""


# =========================================================
# 1. IMPORTS - STDLIB UNIQUEMENT
# =========================================================

import os
import sys
import platform
import importlib
import traceback


# =========================================================
# 2. LES CHEMINS, RECALCULES ICI
# =========================================================

"""
Recalcules sur place plutot qu'importes de modele_v3 : si
modele_v3 ne se charge pas, l'auto-test doit quand meme
pouvoir dire pourquoi.
"""

DOSSIER = os.path.dirname(os.path.abspath(__file__))

DOSSIER_PARENT = os.path.dirname(DOSSIER)

DOSSIER_V2 = os.path.join(DOSSIER_PARENT, "Calibration_v2")


for chemin in (DOSSIER_V2, DOSSIER_PARENT, DOSSIER):
    if chemin not in sys.path:
        sys.path.insert(0, chemin)


FICHIERS = [
    (os.path.join(DOSSIER_V2, "modele_v2.py"), "noyau v2"),
    (os.path.join(DOSSIER_V2, "test_calibrage_v2.py"), "campagne de tirs v2"),
    (os.path.join(DOSSIER_V2, "suivi_frelon_v2.py"), "suivi v2"),
    (os.path.join(DOSSIER_V2, "recalibration_tirs.py"), "ajustement v2"),
    (os.path.join(DOSSIER_V2, "resultats", "modele_grille_v2.json"), "modele ajuste"),
    (os.path.join(DOSSIER_PARENT, "calibration_grille.py"), "modele geometrique"),
    (os.path.join(DOSSIER_PARENT, "suivi_frelon.py"), "suivi d'origine"),
]


MODULES = [
    ("calibration_grille", "modele geometrique"),
    ("modele_v2", "noyau v2"),
    ("suivi_frelon", "suivi d'origine"),
    ("test_calibrage_v2", "campagne de tirs v2"),
    ("suivi_frelon_v2", "suivi v2"),
    ("recalibration_tirs", "ajustement v2"),
    ("modele_v3", "noyau v3"),
    ("referencement", "referencement v3"),
    ("test_calibrage_v3", "campagne de tirs v3"),
    ("suivi_frelon_v3", "suivi v3"),
    ("reajuster_v3", "ajustement v3"),
]


# =========================================================
# 3. LE TEST
# =========================================================

def main():
    """Verifie fichiers puis imports, et retourne 0 si tout passe."""

    print("=" * 57)
    print("AUTO-TEST DE L'INSTALLATION V3")
    print("=" * 57)
    print()
    print("Python       :", platform.python_version())
    print("Executable   :", sys.executable)
    print("Lance depuis :", os.getcwd())
    print()
    print("Dossiers attendus :")
    print("  v3         :", DOSSIER)
    print("  v2         :", DOSSIER_V2)
    print("  scripts    :", DOSSIER_PARENT)
    print()

    # -----------------------------------------------------
    # Etape 1 : les fichiers sont-ils sur le disque ?
    # -----------------------------------------------------

    print("-" * 57)
    print("1. FICHIERS")
    print("-" * 57)
    print()

    absents = []

    for chemin, role in FICHIERS:

        present = os.path.exists(chemin)

        print("  %s  %-22s %s"
              % ("OK    " if present else "ABSENT", role, os.path.basename(chemin)))

        if not present:
            absents.append((chemin, role))

    if absents:
        print()
        print("  Chemins complets des fichiers absents :")
        for chemin, _ in absents:
            print("   ", chemin)

    # -----------------------------------------------------
    # Etape 2 : les imports passent-ils, un par un ?
    # -----------------------------------------------------

    print()
    print("-" * 57)
    print("2. IMPORTS")
    print("-" * 57)
    print()

    echecs = []

    for nom, role in MODULES:

        try:
            importlib.import_module(nom)
            print("  OK      %-22s %s" % (role, nom))

        except Exception as e:
            print("  ECHEC   %-22s %s" % (role, nom))
            print("          %s: %s" % (type(e).__name__, e))
            echecs.append((nom, e))

    # -----------------------------------------------------
    # Etape 3 : verdict
    # -----------------------------------------------------

    print()
    print("-" * 57)
    print("VERDICT")
    print("-" * 57)
    print()

    if not absents and not echecs:
        print("  Tout est en place. Les scripts v3 peuvent tourner.")
        return 0

    if absents:
        print("  %d fichier(s) manquant(s). C'est la cause a regler EN PREMIER :" % len(absents))
        print("  les echecs d'import en decoulent.")
        print()
        print("  A verifier, dans cet ordre :")
        print("    1. git pull")
        print("    2. Calibration_v2 et Calibration_v3 doivent etre")
        print("       COTE A COTE dans Bras/ModuleDeDeplacement/")
        print("    3. lancer depuis le dossier du script :")
        print("         cd Bras/ModuleDeDeplacement/Calibration_v3")
        print("         python3 test_calibrage_v3.py")

    elif echecs:
        print("  Les fichiers sont la, mais %d import(s) echouent." % len(echecs))
        print("  Ce n'est donc pas un probleme de chemin : c'est une erreur")
        print("  DANS le module. Detail du premier :")
        print()

        nom, erreur = echecs[0]

        """
        Le traceback part sur stdout et non sur stderr : melange
        aux print(), l'ordre d'affichage devient imprevisible et
        la sortie collee pour diagnostic est illisible.
        """

        try:
            importlib.import_module(nom)

        except Exception:
            print(traceback.format_exc())

    print()
    print("  Copier cette sortie entiere pour diagnostic.")

    return 1


if __name__ == "__main__":
    sys.exit(main())
