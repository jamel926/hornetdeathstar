"""
=========================================================
ESSAI - QUELLE COMMANDE ALLUME REELLEMENT LE LASER ?
=========================================================

Objectif :
---------
Trancher sur le banc, en cinq minutes, une contradiction
qu'aucune relecture de code ne peut resoudre :

    - Annexes/Electronique/README.md dit : broche G33, sur
      la tete du bras, etat HAUT = laser allume ;
    - PU_GPIO.py ecrit set_basic_output(2, 0), donc une
      autre carte, une autre broche et la logique inverse.

Les deux ne peuvent pas etre vrais. Ce script essaie les
combinaisons une par une et demande a l'operateur ce qu'il
voit. Le resultat est ecrit dans resultats/ pour que le
reste de l'equipe n'ait pas a refaire l'essai.

Ce qu'on apprend aussi en cas d'echec total : si aucune
combinaison n'allume le laser, le probleme n'est pas dans
le choix de la fonction Python mais dans le cablage ou
l'etage de puissance -- et ca vaut la peine de le savoir
avant d'ecrire une ligne de plus.

Prealables :
-----------
    - laser BRANCHE (c'est le seul essai du projet ou il
      doit l'etre), lunettes de protection portees ;
    - faisceau dirige vers une cible sure, personne dans
      l'axe ;
    - aucun autre script en cours sur le port serie.

Lancement :
    python3 test_broche_laser.py
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import json
import os
import time
from datetime import datetime

from PU_Laser import connecterPourLaser, MODE_SORTIE


# =========================================================
# 2. COMBINAISONS A ESSAYER
# =========================================================

"""
Chaque entree decrit UNE hypothese complete : la methode
pymycobot, la broche, et le niveau cense allumer.

Ordre volontaire : l'hypothese de la documentation
electronique d'abord, celle du code existant ensuite. Si la
premiere marche, l'essai s'arrete la et on n'a pas allume
le laser plus que necessaire.

Rappel des broches acceptees par pymycobot 4.0.4 sur un
MyCobot280 (pymycobot/error.py) :
    set_digital_output : 19, 22, 23, 33, 39   (tete, Atom)
    set_basic_output   : 2, 5, 35, 36         (socle M5)
Une broche hors liste leve une exception, ce qui est en soi
une reponse : la combinaison est impossible.
"""

COMBINAISONS = [
    {
        "nom": "doc electronique",
        "methode": "set_digital_output",
        "broche": 33,
        "niveau_allume": 1,
        "commentaire": "G33 sur la tete, logique directe (Annexes/Electronique/README.md)"
    },
    {
        "nom": "doc electronique, logique inverse",
        "methode": "set_digital_output",
        "broche": 33,
        "niveau_allume": 0,
        "commentaire": "meme broche, au cas ou l'etage de puissance inverse"
    },
    {
        "nom": "code PU_GPIO.py",
        "methode": "set_basic_output",
        "broche": 2,
        "niveau_allume": 0,
        "commentaire": "hypothese du code existant, socle M5 (absent sur la version Jetson)"
    },
]

## Duree d'allumage a chaque essai : assez pour voir, assez court pour ne rien abimer
DUREE_ESSAI = 2.0        # secondes

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")


# =========================================================
# 3. ESSAI D'UNE COMBINAISON
# =========================================================

def essayerCombinaison(mc, combinaison):
    """Applique une combinaison, demande a l'operateur ce qu'il a vu, retourne le compte rendu."""

    methode = combinaison["methode"]
    broche = combinaison["broche"]
    niveau_allume = combinaison["niveau_allume"]
    niveau_eteint = 1 - niveau_allume

    resultat = dict(combinaison)
    resultat["erreur"] = None
    resultat["allume_observe"] = None

    print()
    print("-----------------------------------------")
    print("Hypothese :", combinaison["nom"])
    print(combinaison["commentaire"])
    print("Appel :", methode + "(" + str(broche) + ", " + str(niveau_allume) + ")")

    if input("Lancer cet essai ? (o/n) : ").strip().lower() != "o":
        resultat["erreur"] = "essai passe par l'operateur"
        return resultat

    # -----------------------------------------------------
    # Mise en sortie, seulement pour les broches de la tete
    # -----------------------------------------------------

    """
    set_pin_mode ne concerne que la tete. L'appliquer a une
    broche du socle n'aurait pas de sens et pourrait
    configurer une broche sans rapport.
    """

    if methode == "set_digital_output":

        try:
            mc.set_pin_mode(broche, MODE_SORTIE)
            time.sleep(0.05)

        except Exception as e:
            print("set_pin_mode a echoue :", e)
            resultat["erreur"] = "set_pin_mode : " + str(e)
            return resultat

    # -----------------------------------------------------
    # Allumage puis extinction
    # -----------------------------------------------------

    ecrire = getattr(mc, methode, None)

    if ecrire is None:
        print("Methode absente de cette version de pymycobot :", methode)
        resultat["erreur"] = "methode absente"
        return resultat

    try:
        ecrire(broche, niveau_allume)

        try:
            time.sleep(DUREE_ESSAI)

        finally:
            ecrire(broche, niveau_eteint)

    except Exception as e:
        print("La commande a echoue :", e)
        resultat["erreur"] = str(e)
        return resultat

    # -----------------------------------------------------
    # Ce que l'operateur a vu
    # -----------------------------------------------------

    reponse = input("Le laser s'est-il allume ? (o/n) : ").strip().lower()
    resultat["allume_observe"] = (reponse == "o")

    return resultat


# =========================================================
# 4. ENREGISTREMENT
# =========================================================

def enregistrerResultats(resultats):
    """Ecrit le compte rendu de l'essai dans resultats/ et retourne le chemin du fichier."""

    if not os.path.isdir(DOSSIER_RESULTATS):
        os.makedirs(DOSSIER_RESULTATS)

    horodatage = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    chemin = os.path.join(DOSSIER_RESULTATS, "broche_laser_" + horodatage + ".json")

    contenu = {
        "essai": "identification de la broche de commande du laser",
        "horodatage": horodatage,
        "duree_essai_s": DUREE_ESSAI,
        "combinaisons": resultats
    }

    with open(chemin, "w", encoding="utf-8") as fichier:
        json.dump(contenu, fichier, indent=2, ensure_ascii=False)

    return chemin


# =========================================================
# 5. PROGRAMME PRINCIPAL
# =========================================================

def main():
    """Essaie les combinaisons dans l'ordre et s'arrete a la premiere qui allume le laser."""

    print("=========================================")
    print("IDENTIFICATION DE LA COMMANDE DU LASER")
    print("=========================================")
    print()
    print("ATTENTION : cet essai ALLUME le laser.")
    print("Lunettes de protection, faisceau vers une cible")
    print("sure, personne dans l'axe.")
    print()

    if input("Ces conditions sont-elles reunies ? (o/n) : ").strip().lower() != "o":
        print("Essai annule.")
        return 1

    mc = connecterPourLaser()

    if mc is None:
        print("Connexion impossible : verifier qu'aucun autre")
        print("script n'occupe deja le port serie.")
        return 1

    resultats = []
    gagnante = None

    for combinaison in COMBINAISONS:

        resultat = essayerCombinaison(mc, combinaison)
        resultats.append(resultat)

        if resultat["allume_observe"]:
            gagnante = resultat
            break

    # -----------------------------------------------------
    # Conclusion
    # -----------------------------------------------------

    print()
    print("=========================================")

    if gagnante is not None:
        print("Commande retenue :", gagnante["methode"] + "(" + str(gagnante["broche"]) + ", ...)")
        print("Niveau qui allume :", gagnante["niveau_allume"])
        print()
        print("A reporter dans PU_Laser.py : BROCHE_LASER =", gagnante["broche"])
        print("et NIVEAU_ALLUME =", gagnante["niveau_allume"])

    else:
        print("Aucune combinaison n'a allume le laser.")
        print()
        print("Le probleme n'est donc pas le choix de la")
        print("fonction Python. A verifier dans l'ordre :")
        print("  - le laser est-il bien branche et alimente ?")
        print("  - continuite entre G33 et l'entree de la carte ?")
        print("  - masse commune entre la tete et la carte ?")
        print("  - l'etage de puissance a-t-il ete teste seul,")
        print("    en forcant l'entree a 3,3 V au fil ?")

    chemin = enregistrerResultats(resultats)
    print()
    print("Compte rendu :", chemin)

    return 0


if __name__ == "__main__":

    try:
        code_retour = main()

    except KeyboardInterrupt:
        print()
        print("Essai interrompu par l'operateur.")
        print("ATTENTION : verifier de visu que le laser est eteint.")
        code_retour = 1

    print()
    print("Programme termine.")

    raise SystemExit(code_retour)
