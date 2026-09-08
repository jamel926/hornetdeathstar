"""
=========================================================
ROBOT MYCOBOT 280 - JOURNAL DE SESSION D'ESSAIS
=========================================================

Objectif :
---------
Ecrire tout ce qu'une session d'essais a produit dans UN
fichier autonome, transmissible tel quel a quelqu'un qui
n'etait pas devant le robot.

Le CSV des scripts de test reste la trace lisible a l'oeil.
Le journal, lui, garde ce que le CSV perd :

    - les lectures BRUTES, pas seulement leur moyenne ;
    - la consigne exacte envoyee a chaque mouvement ;
    - les constantes du script au moment de l'essai
      (pose initiale, vitesses, tolerances, timeouts) ;
    - la version de pymycobot et du firmware ;
    - les observations libres de l'operateur.

Sans ces elements, une mesure surprenante est indiagnostiquable
a distance : on ne sait pas distinguer un bras qui bouge mal
d'un script qui demande autre chose que ce qu'on croit.

Utilisation :
------------
    from PU_Journal import ouvrirJournal, ajouterMesure, fermerJournal

    journal = ouvrirJournal("repetabilite_initiale", mc, constantes)
    ajouterMesure(journal, "essai", "essai 1", consigne, mesure, lectures)
    chemin = fermerJournal(journal, resume)

Le fichier produit est un JSON dans resultats/, nomme
<test>_<horodatage>.json.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import json
import os
import platform
import sys
from datetime import datetime

from PU_PositionInitiale import PORT, BAUDRATE, ANGLES_POSITION_INITIALE


# =========================================================
# 2. EMPLACEMENT DES JOURNAUX
# =========================================================

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")


# =========================================================
# 3. IDENTIFICATION DU ROBOT
# =========================================================

"""
Methodes interrogees pour identifier le robot.

Elles ne sont PAS garanties : pymycobot expose des methodes
differentes d'une version a l'autre, et certaines echouent
selon le firmware. On les appelle donc une par une, sous
garde, et on note ce qu'on a obtenu. Une entree absente est
une information en soi.
"""

METHODES_IDENTIFICATION = [
    "get_system_version",
    "get_basic_version",
    "get_robot_version",
    "get_fresh_mode",
]


def lireIdentiteRobot(mc):
    """Interroge le robot sur sa version et son mode, sans echouer si une methode manque."""

    identite = {}

    if mc is None:
        return identite

    for nom in METHODES_IDENTIFICATION:

        methode = getattr(mc, nom, None)

        if methode is None:
            identite[nom] = "methode absente de cette version de pymycobot"
            continue

        try:
            identite[nom] = methode()

        except Exception as e:
            identite[nom] = "erreur : " + str(e)

    return identite


def lireVersionPymycobot():
    """Retourne la version de pymycobot installee, ou une explication si elle est illisible."""

    try:
        import pymycobot
        return getattr(pymycobot, "__version__", "attribut __version__ absent")

    except Exception as e:
        return "erreur : " + str(e)


# =========================================================
# 4. OUVERTURE
# =========================================================

def ouvrirJournal(nom_test, mc=None, constantes=None):
    """Cree un journal de session en memoire et y consigne le contexte de depart."""

    journal = {

        "test": nom_test,
        "debut": datetime.now().isoformat(timespec="seconds"),
        "fin": None,

        "environnement": {
            "python": sys.version.split()[0],
            "machine": platform.node(),
            "systeme": platform.platform(),
            "pymycobot": lireVersionPymycobot(),
            "port": PORT,
            "baudrate": BAUDRATE,
        },

        "robot": lireIdentiteRobot(mc),

        # Les constantes sont copiees dans le journal plutot que
        # citees de memoire : une mesure ancienne reste
        # interpretable meme apres modification des scripts.
        "constantes": dict(constantes) if constantes else {},

        "pose_initiale_projet": list(ANGLES_POSITION_INITIALE),

        "evenements": [],
        "observations": [],
        "resume": None,
    }

    return journal


# =========================================================
# 5. ENREGISTREMENT
# =========================================================

def ajouterMesure(journal, categorie, detail, consigne, mesure, lectures=None, extra=None):
    """Consigne un mouvement : ce qui a ete demande, ce qui a ete mesure, et les lectures brutes."""

    if journal is None:
        return

    evenement = {
        "horodatage": datetime.now().isoformat(timespec="seconds"),
        "categorie": categorie,
        "detail": detail,
        "consigne": list(consigne) if consigne is not None else None,
        "mesure": [round(a, 3) for a in mesure] if mesure is not None else None,
    }

    # -----------------------------------------------------
    # Ecart articulation par articulation
    # -----------------------------------------------------

    """
    L'ecart est recalcule ici plutot que transmis par
    l'appelant : c'est la grandeur qu'on relira en premier,
    autant qu'elle soit systematiquement presente et
    toujours calculee de la meme facon.
    """

    if consigne is not None and mesure is not None:
        evenement["ecart"] = [round(mesure[i] - consigne[i], 3) for i in range(len(consigne))]

    # -----------------------------------------------------
    # Lectures brutes
    # -----------------------------------------------------

    if lectures is not None:
        evenement["lectures_brutes"] = [[round(a, 3) for a in lecture] for lecture in lectures]

    if extra is not None:
        evenement["complement"] = extra

    journal["evenements"].append(evenement)


def ajouterObservation(journal, texte):
    """Consigne une remarque libre de l'operateur, horodatee."""

    if journal is None or texte is None or texte.strip() == "":
        return

    journal["observations"].append({
        "horodatage": datetime.now().isoformat(timespec="seconds"),
        "texte": texte.strip(),
    })


def demanderObservation(journal, message="Observation libre (entree pour passer) : "):
    """Demande une remarque a l'operateur et la consigne dans le journal."""

    try:
        texte = input(message)

    except EOFError:
        return

    ajouterObservation(journal, texte)


# =========================================================
# 6. FERMETURE
# =========================================================

def fermerJournal(journal, resume=None):
    """Ecrit le journal sur le disque et retourne le chemin du fichier produit."""

    if journal is None:
        return None

    journal["fin"] = datetime.now().isoformat(timespec="seconds")
    journal["resume"] = resume

    horodatage = datetime.now().strftime("%Y%m%d_%H%M%S")
    nom_fichier = journal["test"] + "_" + horodatage + ".json"
    chemin = os.path.join(DOSSIER_RESULTATS, nom_fichier)

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        with open(chemin, "w") as f:
            json.dump(journal, f, indent=2, ensure_ascii=False, default=str)

    except Exception as e:
        print("Impossible d'ecrire le journal de session :", e)
        return None

    return chemin


def annoncerJournal(chemin):
    """Affiche le chemin du journal et rappelle a quoi il sert."""

    if chemin is None:
        print()
        print("Aucun journal de session n'a pu etre ecrit.")
        return

    print()
    print("-----------------------------------------")
    print("Journal de session :")
    print(" ", chemin)
    print()
    print("Ce fichier contient les lectures brutes, les")
    print("consignes envoyees et les reglages du script.")
    print("C'est celui a transmettre pour un diagnostic.")
    print("-----------------------------------------")
