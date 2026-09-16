"""
=========================================================
ROBOT MYCOBOT 280 - BRAS MOU (DEPLACEMENT A LA MAIN)
=========================================================

Objectif :
---------
Relacher les moteurs pour pouvoir deplacer le bras a la
main, puis les reverrouiller sur la pose obtenue.

Deux usages :
    - perturber le bras entre deux essais de repetabilite
      (voir test_repetabilite_initiale.py) ;
    - relever une nouvelle pose de reference et recopier
      les six angles dans PU_PositionInitiale.py.

Utilisation comme module :
-------------------------
    from PU_PositionInitiale import connecterRobot
    from PU_BrasMou import relacherBras, verrouillerBras

    mc = connecterRobot()
    relacherBras(mc)      # le bras devient mou : LE TENIR
    ...                   # deplacement a la main
    verrouillerBras(mc)   # il se fige la ou il est

Utilisation en autonome :
------------------------
    python3 PU_BrasMou.py

=========================================================
ATTENTION - A LIRE AVANT DE LANCER
=========================================================

1. TENIR LE BRAS AVANT DE RELACHER. Sans couple moteur il
   tombe d'un coup sous son propre poids. Une main sur
   l'avant-bras avant d'appuyer sur entree, pas apres.

2. Laser debranche, pas seulement eteint.

3. Ne jamais lancer un mouvement tant que le bras n'a pas
   ete reverrouille : les servos relaches ignorent la
   consigne, le robot repond mais ne bouge pas.

4. Un seul programme a la fois sur le port serie.
=========================================================
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import time

from PU_PositionInitiale import (
    connecterRobot,
    lireAngles,
    verifierAngles,
    allerPositionInitiale,
    ANGLES_POSITION_INITIALE,
    NOMS_ARTICULATIONS
)

from PU_Visee import lireCoords, NOMS_COORDONNEES, UNITES_COORDONNEES


# =========================================================
# 2. PARAMETRES
# =========================================================

# Vitesse utilisee pour reverrouiller le bras. La consigne
# envoyee etant la pose deja atteinte, le bras ne parcourt
# aucune distance : cette vitesse ne sert qu'a reprendre le
# couple sans a-coup.
VITESSE_VERROUILLAGE = 100   # int, 1 a 100 (impose par pymycobot)

# Periode de rafraichissement du suivi en direct.
PERIODE_SUIVI = 0.5         # secondes

# Duree maximale du suivi en direct, garde-fou si personne
# n'interrompt la boucle.
DUREE_SUIVI_MAX = 300       # secondes


# =========================================================
# 3. RELACHEMENT DES MOTEURS
# =========================================================

def relacherBras(mc):
    """Coupe le couple des six servos pour permettre le deplacement a la main."""

    """
    release_all_servos() ne previent pas et ne retient
    rien : le bras devient mou instantanement. L'appelant
    est responsable de le tenir. On l'ecrit une fois de
    plus ici parce que c'est la ligne dangereuse du
    module.
    """

    try:
        mc.release_all_servos()

    except Exception as e:
        print("Impossible de relacher les moteurs.")
        print("Erreur :", e)
        return False

    # Laisser le temps aux servos de lacher reellement
    time.sleep(0.5)

    print("Moteurs relaches, le bras est mou.")
    return True


# =========================================================
# 4. REVERROUILLAGE
# =========================================================

def verrouillerBras(mc, vitesse=VITESSE_VERROUILLAGE):
    """Realimente les moteurs et fige le bras sur la pose ou il se trouve."""

    # -----------------------------------------------------
    # Pose a maintenir
    # -----------------------------------------------------

    """
    On lit la pose AVANT de realimenter : c'est elle qui
    servira de consigne. Realimenter sans consigne laisse
    certains firmwares au repos, le bras retombe alors
    doucement au lieu de tenir.
    """

    angles = lireAngles(mc)

    if angles is None:
        print("Impossible de lire la pose courante, verrouillage annule.")
        return False

    # -----------------------------------------------------
    # Alimentation
    # -----------------------------------------------------

    try:
        mc.power_on()

    except Exception as e:
        print("Impossible d'alimenter les moteurs.")
        print("Erreur :", e)
        return False

    time.sleep(1)

    # -----------------------------------------------------
    # Maintien sur la pose lue
    # -----------------------------------------------------

    """
    Une pose placee a la main peut sortir des limites que
    pymycobot accepte en consigne. Dans ce cas on ne force
    pas : les moteurs sont alimentes, on previent, et
    l'operateur ramene le bras dans le domaine.
    """

    if not verifierAngles(angles):
        print("La pose courante est hors des limites de consigne.")
        print("Moteurs alimentes, mais aucune consigne de maintien envoyee.")
        print("Ramener le bras dans le domaine autorise avant de continuer.")
        return False

    try:
        mc.send_angles([float(a) for a in angles], int(vitesse))

    except Exception as e:
        print("Erreur pendant l'envoi de la consigne de maintien :", e)
        return False

    time.sleep(1)

    print("Moteurs verrouilles sur la pose courante.")
    return True


# =========================================================
# 5. LECTURE ET AFFICHAGE
# =========================================================

def formaterAngles(angles):
    """Retourne une ligne lisible J1=... J2=... a partir des six angles."""

    morceaux = []

    for i in range(6):
        morceaux.append(NOMS_ARTICULATIONS[i] + "=" + str(round(angles[i], 2)))

    return "  ".join(morceaux)


def afficherPose(mc):
    """Affiche les angles et les coordonnees cartesiennes courants du bras."""

    angles = lireAngles(mc)
    coords = lireCoords(mc)

    if angles is None:
        print("Angles : lecture impossible.")
    else:
        print("Angles (deg) :", formaterAngles(angles))

    """
    La lecture cartesienne peut echouer la ou la lecture
    articulaire passe : le firmware refuse de resoudre la
    geometrie dans certaines poses. Ce n'est pas une
    panne de communication, on continue.
    """

    if coords is None:
        print("Coords : lecture impossible dans cette pose.")
    else:
        morceaux = []
        for i in range(6):
            morceaux.append(
                NOMS_COORDONNEES[i] + "=" + str(round(coords[i], 2))
                + " " + UNITES_COORDONNEES[i]
            )
        print("Coords       :", "  ".join(morceaux))

    return angles


def suivrePose(mc, periode=PERIODE_SUIVI, duree_max=DUREE_SUIVI_MAX):
    """Affiche la pose en continu pendant que l'operateur deplace le bras (Ctrl-C pour sortir)."""

    print()
    print("Suivi en direct. Ctrl-C pour arreter.")
    print()

    debut = time.time()

    try:

        while time.time() - debut < duree_max:

            angles = lireAngles(mc)

            if angles is None:
                print("lecture impossible")
            else:
                print(formaterAngles(angles))

            time.sleep(periode)

    except KeyboardInterrupt:
        print()
        print("Suivi arrete.")

    return lireAngles(mc)


# =========================================================
# 6. CAPTURE D'UNE POSE
# =========================================================

def capturerPose(mc, nb_lectures=5, periode=0.2):
    """Moyenne plusieurs lectures de la pose courante et retourne les six angles."""

    """
    Une lecture unique porte le bruit du capteur. On en
    moyenne plusieurs : c'est ce chiffre qui sera recopie
    dans PU_PositionInitiale.py, autant qu'il soit propre.
    """

    lectures = []

    for _ in range(nb_lectures):

        angles = lireAngles(mc)

        if angles is not None:
            lectures.append(angles)

        time.sleep(periode)

    if len(lectures) == 0:
        print("Aucune lecture exploitable, capture abandonnee.")
        return None

    moyenne = []

    for i in range(6):

        somme = 0.0
        for lecture in lectures:
            somme += lecture[i]

        moyenne.append(round(somme / len(lectures), 2))

    print()
    print("Pose capturee sur", len(lectures), "lectures :")
    print(formaterAngles(moyenne))

    # -----------------------------------------------------
    # Forme recopiable dans PU_PositionInitiale.py
    # -----------------------------------------------------

    print()
    print("A recopier dans PU_PositionInitiale.py :")
    print()
    print("ANGLES_POSITION_INITIALE = [")

    for i in range(6):
        print("    " + str(moyenne[i]) + ",    # " + NOMS_ARTICULATIONS[i])

    print("]")
    print()

    return moyenne


# =========================================================
# 7. EXECUTION AUTONOME
# =========================================================

def afficherMenu():
    """Affiche les actions disponibles."""

    print()
    print("-----------------------------------------")
    print(" 1 - Relacher le bras (LE TENIR AVANT)")
    print(" 2 - Verrouiller le bras sur sa pose")
    print(" 3 - Afficher la pose courante")
    print(" 4 - Suivi de la pose en direct")
    print(" 5 - Capturer la pose (moyenne de 5 lectures)")
    print(" 6 - Retour en position initiale")
    print(" q - Quitter")
    print("-----------------------------------------")

def allumerPointeur(mc):
    """Allume le pointeur rouge et retourne True si la commande est passee."""

    try:
        mc.set_pin_mode(33, 1)
        time.sleep(0.5)
        mc.set_digital_output(33, 0)
        return True

    except Exception as e:
        print("Impossible d'allumer le pointeur :", e)
        return False

def main():
    """Console interactive pour relacher, deplacer a la main et reverrouiller le bras."""

    print()
    print("=========================================")
    print("   BRAS MOU - DEPLACEMENT A LA MAIN")
    print("=========================================")
    print()
    print("ATTENTION : laser debranche, espace degage.")
    print("Tenir le bras AVANT de le relacher.")
    print()

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1
    allumerPointeur(mc)
    # Vrai des que les moteurs ont ete relaches sans etre
    # reverrouilles : sert a refuser les mouvements.
    bras_relache = False

    while True:

        afficherMenu()

        if bras_relache:
            print("Etat : bras RELACHE (mou)")
        else:
            print("Etat : bras verrouille")

        choix = input("Choix : ").strip().lower()

        # -------------------------------------------------
        # Relacher
        # -------------------------------------------------

        if choix == "1":

            print()
            print("Le bras va devenir mou et tomber s'il n'est pas tenu.")
            confirmation = input("Le bras est-il tenu ? (o/n) : ").strip().lower()

            if confirmation != "o":
                print("Relachement annule.")
                continue

            if relacherBras(mc):
                bras_relache = True

        # -------------------------------------------------
        # Verrouiller
        # -------------------------------------------------

        elif choix == "2":

            if verrouillerBras(mc):
                bras_relache = False
                afficherPose(mc)

        # -------------------------------------------------
        # Lecture
        # -------------------------------------------------

        elif choix == "3":
            afficherPose(mc)

        elif choix == "4":
            suivrePose(mc)

        elif choix == "5":
            capturerPose(mc)

        # -------------------------------------------------
        # Retour en position initiale
        # -------------------------------------------------

        elif choix == "6":

            """
            Un bras relache ne suit aucune consigne : le
            mouvement echouerait en silence et le compte
            rendu annoncerait un echec sans cause visible.
            """

            if bras_relache:
                print("Le bras est relache : le verrouiller (choix 2) avant de le deplacer.")
                continue

            print()
            print("Position visee :", ANGLES_POSITION_INITIALE)
            confirmation = input("Lancer le mouvement ? (o/n) : ").strip().lower()

            if confirmation == "o":
                allerPositionInitiale(mc)

        # -------------------------------------------------
        # Sortie
        # -------------------------------------------------

        elif choix == "q":

            if bras_relache:
                print()
                print("Le bras est encore relache.")
                confirmation = input("Le verrouiller avant de quitter ? (o/n) : ").strip().lower()

                if confirmation == "o":
                    verrouillerBras(mc)

            print()
            print("Les moteurs restent alimentes.")
            return 0

        else:
            print("Choix inconnu.")


if __name__ == "__main__":

    try:
        code_retour = main()

    except KeyboardInterrupt:
        print()
        print("Programme interrompu par l'operateur.")
        print("ATTENTION : si le bras etait relache, il l'est toujours.")
        code_retour = 1

    print()
    print("Programme termine.")

    raise SystemExit(code_retour)
