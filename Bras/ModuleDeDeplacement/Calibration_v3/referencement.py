"""
=========================================================
REFERENCEMENT - MESURE DU MANQUE DE LA COMMANDE
=========================================================

Ce que ca mesure :
-----------------
De combien le bras s'arrete SOUS la consigne qu'on lui
envoie. On commande quelques couples d'angles, on relit ce
que les encodeurs annoncent, on fait la difference.

    decalage = angle demande - angle atteint

Positif = le bras s'arrete en dessous. Pour qu'il arrive au
bon endroit, il faudra donc lui demander PLUS.

Pourquoi ca ne demande rien a l'operateur :
------------------------------------------
Le manque se lit sur les encodeurs du bras, pas sur la
position du spot. Aucun releve au sol, aucune saisie : on
envoie, on relit, c'est fini. Quelques dizaines de secondes.

C'est ce qui rend le referencement supportable en debut de
chaque seance -- et donc utile, parce que le manque derive :

    14/09 apres-midi    +0,66 a +0,70 deg sur J4
    15/09 matin         +0,88 deg
    15/09 apres-midi    +0,98 a +1,10 deg  (1,25 au pointage)

Une constante ecrite en dur serait juste un jour et fausse
la semaine suivante, sans que rien ne previenne.

Le choix des points :
--------------------
Ils balaient toute la plage utile de J4 (5 a 60 deg) et les
DEUX signes de J5. Ce n'est pas de la coquetterie : le
manque sur J5 depend du signe -- +0,48 deg quand J5 est
negatif, +0,77 quand il est positif. Mesurer d'un seul cote
donnerait un decalage faux de 0,15 deg, soit 1,5 mm au sol.

Les points sont derives du MODELE, pas ecrits en dur : on
part de cibles du quadrillage et on demande au modele les
angles correspondants. Ils sont donc garantis dans la zone
calibree, meme si le modele change.

Ce que ca ne mesure PAS :
------------------------
La geometrie. Si le quadrillage a bouge ou si le pointeur a
ete remonte ailleurs, le referencement n'en saura rien et
annoncera un bras en pleine forme. Il verifie la COMMANDE,
pas la VISEE.

Utilisation :
------------
    python3 referencement.py

Le lancer seul ne fait que constater l'etat du bras et
l'enregistrer. Les scripts de tir et de suivi l'appellent
eux-memes au demarrage.

SECURITE :
---------
Le pointeur reste ETEINT pendant le referencement : il n'y a
rien a regarder au sol. Le laser de puissance doit etre
DEBRANCHE comme toujours.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import math
import sys

from datetime import datetime


from modele_v3 import (
    ageDecalageHeures,
    anglesPourPointCorrige,
    chargerDernierDecalage,
    chargerModeleV2,
    enCentimetres,
    enregistrerDecalage,
    resumerDecalage,
    sensibilite,
    verifierBaseV2,
)

from calibration_grille import (
    allerPositionInitiale,
    bougerTete,
    connecterRobot,
    eteindrePointeur,
    lireAngles,
)


# =========================================================
# 2. CONFIGURATION
# =========================================================

"""
Cibles du quadrillage servant a fabriquer les points de
mesure. Deux exigences, et la seconde est facile a rater :

1. Balayer J4 de bout en bout, 5 a 61 degres.
2. EQUILIBRER les signes de J5. Le manque sur J5 vaut
   +0,48 deg du cote negatif et +0,77 du cote positif : un
   echantillon desequilibre tirerait la moyenne vers le cote
   sur-represente et donnerait un decalage faux.

    (12,  0)   J4 ~  5 deg, J5 ~  +5     bas de zone
    ( 2,  7)   J4 ~ 26 deg, J5 ~ +30     positif
    (22,  7)   J4 ~ 26 deg, J5 ~ -23     negatif
    ( 4, 14)   J4 ~ 42 deg, J5 ~ +22     positif
    (20, 14)   J4 ~ 42 deg, J5 ~ -14     negatif
    ( 3, 24)   J4 ~ 58 deg, J5 ~ +18     positif, loin
    (21, 24)   J4 ~ 58 deg, J5 ~ -13     negatif, loin
    (12, 27)   J4 ~ 61 deg, J5 ~  +3     fond de zone

Soit 3 franchement positifs, 3 franchement negatifs, 2
proches de zero. Huit mouvements, une trentaine de secondes.

Si l'ecart-type sur J5 reste gros malgre cet equilibrage,
c'est que le manque depend vraiment du signe : la suite
serait alors un decalage par signe plutot qu'un decalage
unique. Le fichier enregistre garde chaque point, de quoi
trancher sans refaire la mesure.
"""
CIBLES_REFERENCE = [
    (12, 0),
    (2, 7),
    (22, 7),
    (4, 14),
    (20, 14),
    (3, 24),
    (21, 24),
    (12, 27),
]


# Au dela de cet ecart-type, le manque n'est pas constant :
# un decalage unique ne le corrigerait qu'en moyenne, et on
# le signale plutot que de faire comme si de rien n'etait.
ECART_TYPE_ACCEPTABLE = 0.3     # degres


# En dessous, il n'y a rien a compenser.
MANQUE_NEGLIGEABLE = 0.1        # degres


# =========================================================
# 3. LES POINTS DE MESURE
# =========================================================

def pointsDeReference(modele, cibles=None):
    """Convertit les cibles du quadrillage en couples d'angles, et retourne la liste."""

    if cibles is None:
        cibles = CIBLES_REFERENCE

    points = []

    for u, v in cibles:

        angles = anglesPourPointCorrige(modele, float(u), float(v))

        if angles is None:
            print("  cible (%g, %g) hors modele, ignoree" % (u, v))
            continue

        points.append({
            "cible": (float(u), float(v)),
            "j4": angles[0],
            "j5": angles[1],
        })

    return points


# =========================================================
# 4. LA MESURE
# =========================================================

def mesurerDecalageCommande(mc, base, j6, modele, cibles=None, bavard=True):
    """Mesure de combien le bras s'arrete sous la consigne, et retourne le decalage, ou None."""

    """
    Un aller simple par point. On ne fait PAS de rattrapage
    ici : c'est justement le manque du premier mouvement
    qu'on veut mesurer, puisque c'est celui que subissent
    viserAngles() et le suivi.

    bougerTete attend la stabilisation avant de rendre la
    main, donc la pose relue est bien la pose finale et non
    une lecture en vol.
    """

    points = pointsDeReference(modele, cibles)

    if len(points) < 3:
        print("ERREUR : moins de 3 points de reference exploitables.")
        return None

    if bavard:
        print()
        print("Referencement : %d points, pointeur eteint." % len(points))
        print("  %-14s %-18s %-18s" % ("cible", "consigne J4/J5", "manque J4/J5"))

    mesures = []

    for point in points:

        arrive, pose = bougerTete(mc, base, point["j4"], point["j5"], j6)

        if not arrive or pose is None:
            if bavard:
                print("  (%g, %g) : mouvement echoue, point ignore" % point["cible"])
            continue

        manque_j4 = point["j4"] - pose[3]
        manque_j5 = point["j5"] - pose[4]

        mesures.append({
            "cible": list(point["cible"]),
            "consigne_j4": point["j4"],
            "consigne_j5": point["j5"],
            "atteint_j4": pose[3],
            "atteint_j5": pose[4],
            "manque_j4": manque_j4,
            "manque_j5": manque_j5,
        })

        if bavard:
            print("  (%2g, %2g)      %+7.2f %+7.2f    %+6.3f %+6.3f"
                  % (point["cible"][0], point["cible"][1],
                     point["j4"], point["j5"], manque_j4, manque_j5))

    if len(mesures) < 3:
        print("ERREUR : moins de 3 points ont abouti, referencement abandonne.")
        return None

    # -----------------------------------------------------
    # Moyenne et dispersion
    # -----------------------------------------------------

    """
    L'ecart-type est le chiffre qui decide. Un manque
    constant se corrige par un decalage unique ; un manque
    disperse ne se corrige pas comme ca, et il vaut mieux le
    dire que de laisser croire que c'est regle.
    """

    nombre = len(mesures)

    moyenne_j4 = sum(m["manque_j4"] for m in mesures) / nombre
    moyenne_j5 = sum(m["manque_j5"] for m in mesures) / nombre

    ecart_j4 = math.sqrt(sum((m["manque_j4"] - moyenne_j4) ** 2 for m in mesures) / nombre)
    ecart_j5 = math.sqrt(sum((m["manque_j5"] - moyenne_j5) ** 2 for m in mesures) / nombre)

    fiable = (ecart_j4 <= ECART_TYPE_ACCEPTABLE and ecart_j5 <= ECART_TYPE_ACCEPTABLE)

    return {
        "date": datetime.now().isoformat(timespec="seconds"),
        "j4": moyenne_j4,
        "j5": moyenne_j5,
        "ecart_type_j4": ecart_j4,
        "ecart_type_j5": ecart_j5,
        "nb_points": nombre,
        "fiable": fiable,
        "base": [round(b, 3) for b in base],
        "mesures": mesures,
    }


# =========================================================
# 5. LECTURE DU RESULTAT
# =========================================================

def traduireEnCarreaux(decalage, modele):
    """Affiche ce que le manque coute au sol, du bas au fond de la zone."""

    """
    Un degre ne dit rien tant qu'on ne sait pas ce qu'il fait
    au sol. La sensibilite triple entre le bas et le fond de
    la zone : c'est pour ca qu'un manque CONSTANT produit une
    erreur qui GRANDIT avec y, et c'est cette signature qui a
    permis d'identifier le probleme.
    """

    if decalage is None or decalage.get("nb_points", 0) == 0:
        return

    print()
    print("Ce que ce manque coute au sol, si on ne le compense pas :")
    print("  %-8s %-22s %-16s" % ("cible", "sensibilite J4", "erreur attendue"))

    for u, v in [(12, 0), (12, 13), (12, 20), (12, 27)]:

        angles = anglesPourPointCorrige(modele, float(u), float(v))

        if angles is None:
            continue

        sens = sensibilite(modele, angles[0], angles[1])

        if sens is None:
            continue

        erreur = abs(decalage["j4"]) * sens[0]

        print("  (%2d, %2d)  %.2f carreau par degre    %.2f carreau (%.1f cm)"
              % (u, v, sens[0], erreur, enCentimetres(erreur)))


def comparerAuPrecedent(decalage):
    """Compare le decalage mesure au precedent enregistre, pour rendre la derive visible."""

    precedent = chargerDernierDecalage()

    if precedent is None or precedent.get("nb_points", 0) == 0:
        return

    age = ageDecalageHeures(precedent)

    print()
    print("Mesure precedente : J4 %+.3f, J5 %+.3f%s"
          % (precedent["j4"], precedent["j5"],
             ("  (il y a %.1f h)" % age) if age is not None else ""))

    derive_j4 = decalage["j4"] - precedent["j4"]

    print("Derive depuis     : J4 %+.3f deg" % derive_j4)

    if abs(derive_j4) > 0.3:
        print("  Le manque a nettement bouge. C'est normal entre deux seances,")
        print("  ca ne l'est pas dans la meme : si c'est le cas, le bras chauffe")
        print("  ou quelque chose s'est desserre.")


# =========================================================
# 6. PROGRAMME PRINCIPAL
# =========================================================

def referencer(mc, base, j6, modele, bavard=True):
    """Mesure et enregistre le decalage de commande, et le retourne, ou None."""

    decalage = mesurerDecalageCommande(mc, base, j6, modele, bavard=bavard)

    if decalage is None:
        return None

    print()
    resumerDecalage(decalage)

    if abs(decalage["j4"]) < MANQUE_NEGLIGEABLE and abs(decalage["j5"]) < MANQUE_NEGLIGEABLE:
        print()
        print("Le bras atteint ses consignes : il n'y a rien a compenser.")
        print("Si le pointage reste mauvais, le probleme est ailleurs")
        print("(pose de base, quadrillage deplace, pointeur remonte).")

    comparerAuPrecedent(decalage)

    chemin = enregistrerDecalage(decalage)

    if chemin:
        print()
        print("Enregistre :", chemin.split("Calibration_v3/")[-1])

    return decalage


def main():
    """Mesure le decalage de commande et l'enregistre, sans rien viser."""

    print("=" * 57)
    print("REFERENCEMENT DE LA COMMANDE")
    print("=" * 57)
    print()
    print("Mesure de combien le bras s'arrete sous ses consignes.")
    print("Aucun releve au sol : tout se lit sur les encodeurs.")
    print("Le pointeur reste eteint.")
    print()

    modele = chargerModeleV2()

    if modele is None:
        return 1

    mc = connecterRobot()

    if mc is None:
        return 1

    allerPositionInitiale(mc)

    if not verifierBaseV2(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    try:
        # Le pointeur n'a rien a faire ici, mais on s'assure
        # qu'il est eteint plutot que de le supposer.
        eteindrePointeur(mc)

        decalage = referencer(mc, base, j6, modele)

    except KeyboardInterrupt:
        print()
        print("Referencement interrompu.")
        return 1

    finally:
        eteindrePointeur(mc)

    if decalage is None:
        return 1

    traduireEnCarreaux(decalage, modele)

    print()
    print("Ce decalage sera repris automatiquement par")
    print("test_calibrage_v3.py et suivi_frelon_v3.py.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
