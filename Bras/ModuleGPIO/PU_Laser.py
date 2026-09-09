"""
=========================================================
ROBOT MYCOBOT 280 - COMMANDE DU LASER (BROCHE G33)
=========================================================

Objectif :
---------
Allumer et eteindre le laser depuis Python, par la broche
G33 de la tete du bras, avec la garantie qu'on ne le laisse
jamais allume sans l'avoir voulu.

Chaine de commande (Annexes/Img/laserblocdiagramm.svg) :

    Jetson Nano (ce code)
        --> liaison serie UART /dev/ttyTHS1
            --> MCU de la tete du bras (M5 Atom)
                --> GPIO G33
                    --> etage de puissance (transistor)
                        --> LASER

Pourquoi ce module remplace PU_GPIO.py :
---------------------------------------
PU_GPIO.py commande `set_basic_output(2, ...)`. Trois
problemes, tous verifies dans le paquet pymycobot 4.0.4 et
dans Annexes/Electronique/README.md :

    1. MAUVAISE CARTE. `set_basic_output` s'adresse a la
       carte M5 Basic (le socle des myCobot version M5).
       Notre bras est la version Jetson Nano : ce socle
       n'existe pas. La broche du laser est sur la TETE
       (M5 Atom), qui se commande par `set_digital_output`.
       Voir pymycobot/generate.py : "Set the terminal atom
       io status" pour l'une, "for M5 version" pour l'autre.

    2. MAUVAISE BROCHE. pymycobot/error.py n'autorise que
       {2, 5, 35, 36} pour set_basic_output et
       {19, 22, 23, 33, 39} pour set_digital_output. G33
       n'est atteignable QUE par set_digital_output.

    3. LOGIQUE INVERSEE. PU_GPIO.py ecrit 0 pour allumer.
       La carte d'interface V2 documentee dans
       Annexes/Electronique/README.md est en logique
       directe : G33 a l'etat HAUT ferme l'interrupteur
       et allume le laser.

Le point 3 est le seul qui depende du materiel plutot que
de la bibliotheque : si la carte reelle ne correspond pas a
sa documentation, il n'y a qu'une constante a changer,
NIVEAU_ALLUME. Le script test_broche_laser.py sert
justement a trancher ca sur le banc.

Deux securites integrees :
-------------------------
    - ARMEMENT. allumerLaser() refuse de tirer tant que
      armerLaser() n'a pas ete appele explicitement. Un
      script de deplacement qui importe ce module par
      erreur ne peut donc pas allumer le laser.

    - EXTINCTION GARANTIE. tirer() eteint dans un `finally`,
      donc l'extinction a lieu meme sur Ctrl-C, meme sur
      exception, meme si la duree demandee etait trop
      longue.

ATTENTION - etat de la broche a la mise sous tension :
-----------------------------------------------------
Entre l'alimentation du bras et le premier appel a
initialiserLaser(), l'etat de G33 n'est pas garanti par le
logiciel : le laser peut s'allumer tout seul. Le vrai
correctif est electronique (resistance de rappel a la masse
sur la commande du transistor), pas logiciel. En attendant,
le laser reste DEBRANCHE pendant les essais de deplacement,
et on branche seulement apres avoir appele
initialiserLaser().

ATTENTION - un seul processus sur le port serie :
------------------------------------------------
Ce module n'ouvre PAS le port a l'import : il recoit l'objet
robot deja connecte, comme PU_Tete et PU_Pointage. C'est ce
qui permet de piloter le bras et le laser depuis le meme
programme, avec une seule connexion.

ATTENTION - on ne peut pas relire la broche :
--------------------------------------------
pymycobot n'expose aucune lecture des sorties de la tete.
etatLaser() renvoie donc ce que le programme a COMMANDE en
dernier, pas ce que le laser fait reellement. Un cable
debranche ne se voit pas ici : la seule verification est
visuelle, avec les lunettes.

Utilisation comme module :
-------------------------
    from PU_Laser import initialiserLaser, armerLaser, tirer

    mc = connecterRobot()
    initialiserLaser(mc)      # broche en sortie, laser eteint
    armerLaser()              # autorisation explicite
    tirer(mc, 0.5)            # allume 0,5 s puis eteint

Utilisation en autonome :
------------------------
    python3 PU_Laser.py       # console interactive

SECURITE LASER :
---------------
Laser de classe 3B/4 : lesion oculaire irreversible et
risque d'incendie. Lunettes certifiees pour la longueur
d'onde du module, zone de tir confinee, personne dans l'axe.
Pour toute mise au point du POINTAGE, utiliser le pointeur
rouge de classe 2 en substitut, jamais le laser de
puissance.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import sys
import time
import ModuleDeDeplacement


# =========================================================
# 2. BROCHE ET LOGIQUE DE COMMANDE
# =========================================================

"""
Broche de commande, cote tete du bras (M5 Atom).

Valeurs acceptees par pymycobot pour set_digital_output sur
un MyCobot280 : 19, 22, 23, 33, 39. Toute autre valeur leve
une MyCobot280DataException a l'envoi.
"""

BROCHE_LASER = 33

## 1 = sortie (0 = entree, 2 = entree avec rappel au +)
MODE_SORTIE = 1

"""
Logique de la carte d'interface V2 : etat HAUT = laser
allume. Si le banc montre l'inverse, c'est CETTE constante
qu'on change, et rien d'autre dans le fichier.
"""

NIVEAU_ALLUME = 1
NIVEAU_ETEINT = 1 - NIVEAU_ALLUME


# =========================================================
# 3. GARDE-FOUS
# =========================================================

"""
Duree maximale d'un tir commande par tirer().

Ce n'est pas une regle de securite oculaire (elle ne protege
personne), c'est un garde-fou contre la faute de frappe :
saisir 60 au lieu de 6 ne doit pas laisser le laser allume
une minute.
"""

DUREE_TIR_MAX = 5.0      # secondes

## Temps laisse au MCU entre deux ecritures successives sur la broche
DELAI_ECRITURE = 0.05    # secondes


# =========================================================
# 4. ETAT MEMORISE
# =========================================================

"""
pymycobot ne sait pas relire une sortie de la tete : il n'y
a pas de get_digital_output. On memorise donc ce qu'on a
envoye, en sachant que c'est une INTENTION et pas une
mesure.

_broche_configuree evite le piege suivant : ecrire sur une
broche restee en mode entree ne provoque aucune erreur, la
commande part et il ne se passe rien. On refuse donc
d'allumer tant que initialiserLaser() n'a pas reussi.
"""

_broche_configuree = False
_laser_allume = False
_laser_arme = False
_instant_allumage = None


# =========================================================
# 5. ECRITURE BAS NIVEAU
# =========================================================

def ecrireBroche(mc, niveau):
    """Ecrit un niveau logique sur la broche du laser et retourne True si la commande est partie."""

    if mc is None:
        print("Pas de robot connecte : commande laser ignoree.")
        return False

    if niveau not in (0, 1):
        print("Niveau logique invalide :", niveau)
        return False

    try:
        mc.set_digital_output(BROCHE_LASER, niveau)

    except Exception as e:
        print("Echec d'ecriture sur la broche G" + str(BROCHE_LASER) + ".")
        print("Erreur :", e)
        return False

    """
    Le MCU de la tete n'accuse pas reception. La valeur de
    retour de set_digital_output ne prouve donc rien : on
    ne teste pas dessus, on rend la main apres avoir laisse
    le message partir.
    """

    time.sleep(DELAI_ECRITURE)
    return True


# =========================================================
# 6. INITIALISATION
# =========================================================

def initialiserLaser(mc):
    """Configure G33 en sortie et force le laser eteint ; retourne True si les deux ont reussi."""

    global _broche_configuree, _laser_allume

    print("-----------------------------------------")
    print("Initialisation de la commande laser")
    print("Broche : G" + str(BROCHE_LASER) + " (tete du bras)")
    print("Etat allume :", NIVEAU_ALLUME, " / etat eteint :", NIVEAU_ETEINT)

    if mc is None:
        print("Pas de robot connecte.")
        return False

    # -----------------------------------------------------
    # Mise de la broche en sortie
    # -----------------------------------------------------

    try:
        mc.set_pin_mode(BROCHE_LASER, MODE_SORTIE)

    except Exception as e:
        print("Impossible de configurer G" + str(BROCHE_LASER) + " en sortie.")
        print("Erreur :", e)
        return False

    time.sleep(DELAI_ECRITURE)
    _broche_configuree = True

    # -----------------------------------------------------
    # Extinction immediate
    # -----------------------------------------------------

    """
    L'ordre compte : tant que la broche n'est pas en sortie,
    ecrire dessus ne fait rien. On configure donc d'abord,
    on eteint tout de suite apres, et on ne rend la main
    qu'une fois le laser dans un etat connu.
    """

    if not ecrireBroche(mc, NIVEAU_ETEINT):
        print("Broche configuree mais extinction non confirmee.")
        _broche_configuree = False
        return False

    _laser_allume = False

    print("Commande laser prete, laser eteint.")
    return True


# =========================================================
# 7. ARMEMENT
# =========================================================

def armerLaser():
    """Autorise les allumages ; a appeler explicitement, jamais depuis un import."""

    global _laser_arme

    _laser_arme = True
    print("Laser ARME : les commandes d'allumage sont acceptees.")


def desarmerLaser(mc=None):
    """Interdit les allumages et eteint le laser s'il etait allume."""

    global _laser_arme

    if _laser_allume and mc is not None:
        eteindreLaser(mc)

    _laser_arme = False
    print("Laser DESARME : les commandes d'allumage sont refusees.")


def estArme():
    """Retourne True si les allumages sont autorises."""

    return _laser_arme


# =========================================================
# 8. COMMANDE DU LASER
# =========================================================

def peutAllumer():
    """Retourne True si un allumage est autorise, et dit pourquoi il ne l'est pas sinon."""

    if not _broche_configuree:
        print("Refus : appeler initialiserLaser() avant d'allumer.")
        return False

    if not _laser_arme:
        print("Refus : laser non arme. Appeler armerLaser() explicitement.")
        return False

    return True


def allumerLaser(mc):
    """Allume le laser si la broche est prete et le laser arme ; retourne True si la commande est partie."""

    global _laser_allume, _instant_allumage

    if not peutAllumer():
        return False

    if not ecrireBroche(mc, NIVEAU_ALLUME):
        return False

    _laser_allume = True
    _instant_allumage = time.time()

    print("LASER ALLUME.")
    return True


def eteindreLaser(mc):
    """Eteint le laser ; retourne True si la commande est partie."""

    global _laser_allume, _instant_allumage

    """
    Volontairement sans garde-fou : on doit pouvoir eteindre
    meme si l'etat memorise est faux, meme si le laser n'a
    jamais ete arme, meme apres une erreur. Une extinction
    qu'on peut refuser n'est pas une securite.
    """

    if not ecrireBroche(mc, NIVEAU_ETEINT):
        print("ATTENTION : extinction non confirmee, couper l'alimentation du laser.")
        return False

    _laser_allume = False
    _instant_allumage = None

    print("Laser eteint.")
    return True


def tirer(mc, duree):
    """Allume le laser pendant une duree bornee puis l'eteint, meme en cas d'interruption."""

    if duree <= 0:
        print("Duree de tir invalide :", duree)
        return False

    if duree > DUREE_TIR_MAX:
        print("Duree", duree, "s ramenee au maximum autorise :", DUREE_TIR_MAX, "s.")
        duree = DUREE_TIR_MAX

    """
    Les refus (broche non prete, laser non arme) sont
    verifies AVANT le try : dans ces cas la, rien n'a ete
    envoye et il n'y a rien a eteindre.
    """

    if not peutAllumer():
        return False

    """
    L'allumage, lui, est DANS le try/finally, pas avant.
    C'est la difference qui compte : entre l'ecriture du
    niveau haut et le retour de allumerLaser() il y a une
    attente, et un Ctrl-C tombant pile la laissait le laser
    allume sans jamais passer par l'extinction. Ici,
    Ctrl-C pendant le tir, coupure de la liaison serie ou
    exception imprevue passent tous par eteindreLaser().
    C'est la seule partie du module ou l'ordre des lignes
    est une question de securite.
    """

    allume = False

    try:
        allume = allumerLaser(mc)

        if allume:
            time.sleep(duree)

    finally:
        eteindreLaser(mc)

    return allume


# =========================================================
# 9. ETAT
# =========================================================

def etatLaser():
    """Retourne l'etat COMMANDE en dernier (broche prete, arme, allume, duree d'allumage)."""

    duree = None

    if _laser_allume and _instant_allumage is not None:
        duree = round(time.time() - _instant_allumage, 2)

    return {
        "broche": BROCHE_LASER,
        "configuree": _broche_configuree,
        "armee": _laser_arme,
        "allume": _laser_allume,
        "duree_allumage": duree
    }


def afficherEtatLaser():
    """Affiche l'etat commande, en rappelant que ce n'est pas une mesure."""

    etat = etatLaser()

    print()
    print("Broche             : G" + str(etat["broche"]))
    print("Broche configuree  :", "oui" if etat["configuree"] else "non")
    print("Laser arme         :", "oui" if etat["armee"] else "non")
    print("Laser allume       :", "OUI" if etat["allume"] else "non")

    if etat["duree_allumage"] is not None:
        print("Allume depuis      :", etat["duree_allumage"], "s")

    print("(etat COMMANDE, pas mesure : la broche n'est pas relisible)")


# =========================================================
# 10. CONNEXION POUR L'USAGE AUTONOME
# =========================================================

"""
connecterRobot() vit dans ModuleDeDeplacement, dossier
voisin. Le projet n'etant pas installe comme paquet Python,
on ajoute ce dossier au chemin d'import plutot que de
redefinir ici le port et le baudrate : deux definitions du
port serie finiraient par diverger.
"""

import importlib.util
DOSSIER_DEPLACEMENT = os.path.abspath(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "ModuleDeDeplacement"
    )
)


def connecterPourLaser():
    """Charge PU_PositionInitiale.py explicitement et ouvre la liaison série."""

    chemin = os.path.join(
        DOSSIER_DEPLACEMENT,
        "PU_PositionInitiale.py"
    )

    print("Recherche de PU_PositionInitiale ici :")
    print(chemin)

    if not os.path.isfile(chemin):
        print("ERREUR : fichier PU_PositionInitiale.py introuvable.")
        return None

    try:
        spec = importlib.util.spec_from_file_location(
            "PU_PositionInitiale",
            chemin
        )

        if spec is None or spec.loader is None:
            print("Impossible de charger PU_PositionInitiale.")
            return None

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

    except Exception as e:
        print("Erreur lors du chargement de PU_PositionInitiale :")
        print(type(e).__name__, ":", e)
        return None

    if not hasattr(module, "connecterRobot"):
        print("ERREUR : connecterRobot() n'existe pas dans PU_PositionInitiale.py")
        return None

    return module.connecterRobot()



# =========================================================
# 11. CONSOLE INTERACTIVE
# =========================================================

def lireDuree(message):
    """Demande une duree en secondes et retourne None si la saisie n'est pas un nombre."""

    saisie = input(message).strip()

    try:
        return float(saisie.replace(",", "."))

    except ValueError:
        print("Saisie invalide :", saisie)
        return None


def afficherMenu():
    """Affiche le menu de la console laser."""

    print()
    print("-----------------------------------------")
    print("1 - Armer le laser")
    print("2 - Desarmer le laser")
    print("3 - Allumer")
    print("4 - Eteindre")
    print("5 - Tir minute (duree bornee a", DUREE_TIR_MAX, "s)")
    print("6 - Etat")
    print("q - Quitter (eteint le laser)")


def main():
    """Console interactive de commande du laser sur G33."""

    print("=========================================")
    print("COMMANDE DU LASER - BROCHE G" + str(BROCHE_LASER))
    print("=========================================")
    print()
    print("ATTENTION : lunettes de protection, zone de tir")
    print("degagee, personne dans l'axe du faisceau.")
    print()

    mc = connecterPourLaser()

    if mc is None:
        print("Connexion impossible : verifier qu'aucun autre")
        print("script n'occupe deja le port serie.")
        return 1

    if not initialiserLaser(mc):
        print("Initialisation echouee : on ne va pas plus loin.")
        return 1

    """
    Le try/finally couvre toute la session : quelle que soit
    la facon d'en sortir, on repasse par une extinction.
    """

    try:

        while True:

            afficherMenu()
            choix = input("Choix : ").strip().lower()

            if choix == "1":
                armerLaser()

            elif choix == "2":
                desarmerLaser(mc)

            elif choix == "3":
                allumerLaser(mc)

            elif choix == "4":
                eteindreLaser(mc)

            elif choix == "5":

                duree = lireDuree("Duree du tir en secondes : ")

                if duree is not None:
                    tirer(mc, duree)

            elif choix == "6":
                afficherEtatLaser()

            elif choix == "q":
                return 0

            else:
                print("Choix inconnu.")

    finally:
        eteindreLaser(mc)
        desarmerLaser()


if __name__ == "__main__":

    try:
        code_retour = main()

    except KeyboardInterrupt:
        print()
        print("Programme interrompu par l'operateur.")
        code_retour = 1

    print()
    print("Programme termine.")

    raise SystemExit(code_retour)
