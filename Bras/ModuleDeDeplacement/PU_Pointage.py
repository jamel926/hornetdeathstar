"""
=========================================================
ROBOT MYCOBOT 280 - POINTAGE DU LASER, BASE FIGEE
=========================================================

Objectif :
---------
Repondre a la question que le projet pose vraiment : ou
mettre J4 et J5 pour que le faisceau tombe sur un point
donne du quadrillage.

Pourquoi J4 et J5 seulement :
----------------------------
Le laser est monte COAXIAL a l'axe de rotation de J6.
Tourner J6 fait donc tourner le faisceau sur lui-meme sans
deplacer sa droite : J6 n'a aucun effet optique. Restent
deux angles pour deux coordonnees d'impact, le probleme
est exactement determine.

Le modele :
----------
Sur la zone balayee, l'impact est decrit par une fonction
bilineaire des deux angles :

    u = a0 + a1*J4 + a2*J5 + a3*J4*J5
    v = b0 + b1*J4 + b2*J5 + b3*J4*J5

Le terme croise J4*J5 rend compte du couplage des deux
rotations en serie. Le modele est LINEAIRE en ses huit
coefficients : l'ajustement se fait par moindres carres
resolus a la main, sans numpy ni scipy, dont la presence
sur la Jetson n'est pas garantie.

Ce que le modele ne fait pas :
-----------------------------
Il est LOCAL. Il decrit la zone reellement balayee et rien
d'autre. Hors de cette zone, anglesPourPoint() retourne
None au lieu d'extrapoler : une extrapolation silencieuse
enverrait le faisceau n'importe ou en ayant l'air de
fonctionner.

Il decrit aussi UN plan, celui du quadrillage. Viser dans
l'espace demanderait une seconde campagne a une autre
hauteur.

Utilisation :
------------
    python3 PU_Pointage.py        # console de visee

    from PU_Pointage import chargerModele, pointerSur
    modele = chargerModele()
    pointerSur(mc, modele, 5.0, -2.5)   # centimetres

ATTENTION :
----------
Toute la calibration se fait au POINTEUR ROUGE CLASSE 2,
jamais avec le laser de puissance.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import csv
import json
import os
from datetime import datetime

from PU_Tete import bougerTeteAvecApproche, lireTete


# =========================================================
# 2. PARAMETRES
# =========================================================

# Cote d'un carreau du quadrillage pose au sol.
COTE_CARREAU = 2.5       # centimetres

"""
Marge angulaire acceptee au-dela des angles reellement
balayes. Une petite extrapolation reste raisonnable, une
grande ne l'est pas : au-dela de cette marge, la fonction
d'inversion refuse de repondre.
"""
MARGE_ENVELOPPE = 3.0    # degres

# Reglages de l'inversion par la methode de Newton.
ITERATIONS_MAX = 40
PRECISION_INVERSION = 1e-6   # centimetres


# =========================================================
# 3. FORMAT DU FICHIER DE BALAYAGE
# =========================================================

"""
Colonnes du CSV produit par test_balayage_tete.py.

Les angles MESURES (J1 a J6) et les angles COMMANDES sont
tous les deux enregistres. Le modele s'ajuste sur les
angles mesures : avec la dispersion relevee sur J5, ajuster
sur la consigne reviendrait a faire entrer le jeu
mecanique dans le modele.
"""

ENTETE_BALAYAGE = [
    "horodatage", "serie", "usage", "index",
    "j4_consigne", "j5_consigne",
    "J1", "J2", "J3", "J4", "J5", "J6",
    "u_cm", "v_cm", "commentaire"
]

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")
FICHIER_BALAYAGE = os.path.join(DOSSIER_RESULTATS, "balayage_tete.csv")
FICHIER_MODELE = os.path.join(DOSSIER_RESULTATS, "modele_pointage.json")


# =========================================================
# 4. ALGEBRE LINEAIRE
# =========================================================

def resoudreSysteme(matrice, second_membre):
    """Resout un systeme lineaire par elimination de Gauss avec pivot partiel, ou None si singulier."""

    taille = len(matrice)

    # Copie de travail : on ne modifie pas les donnees de l'appelant
    a = [list(ligne) + [second_membre[i]] for i, ligne in enumerate(matrice)]

    for colonne in range(taille):

        # -------------------------------------------------
        # Choix du pivot
        # -------------------------------------------------

        """
        Pivot partiel : sans lui, un coefficient proche de
        zero en diagonale fait exploser les erreurs
        d'arrondi. C'est un vrai risque ici, les termes
        croises etant beaucoup plus petits que les termes
        constants.
        """

        ligne_pivot = colonne

        for ligne in range(colonne + 1, taille):
            if abs(a[ligne][colonne]) > abs(a[ligne_pivot][colonne]):
                ligne_pivot = ligne

        if abs(a[ligne_pivot][colonne]) < 1e-12:
            return None

        a[colonne], a[ligne_pivot] = a[ligne_pivot], a[colonne]

        # -------------------------------------------------
        # Elimination
        # -------------------------------------------------

        for ligne in range(colonne + 1, taille):

            facteur = a[ligne][colonne] / a[colonne][colonne]

            for k in range(colonne, taille + 1):
                a[ligne][k] -= facteur * a[colonne][k]

    # -----------------------------------------------------
    # Remontee
    # -----------------------------------------------------

    solution = [0.0] * taille

    for ligne in range(taille - 1, -1, -1):

        somme = a[ligne][taille]

        for colonne in range(ligne + 1, taille):
            somme -= a[ligne][colonne] * solution[colonne]

        solution[ligne] = somme / a[ligne][ligne]

    return solution


def moindresCarres(lignes, valeurs):
    """Ajuste des coefficients par equations normales et retourne la solution, ou None."""

    nb_coefficients = len(lignes[0])

    # Matrice X'X
    normale = []

    for i in range(nb_coefficients):

        ligne = []

        for j in range(nb_coefficients):

            somme = 0.0
            for k in range(len(lignes)):
                somme += lignes[k][i] * lignes[k][j]

            ligne.append(somme)

        normale.append(ligne)

    # Second membre X'y
    membre = []

    for i in range(nb_coefficients):

        somme = 0.0
        for k in range(len(lignes)):
            somme += lignes[k][i] * valeurs[k]

        membre.append(somme)

    return resoudreSysteme(normale, membre)


# =========================================================
# 5. TERMES DU MODELE
# =========================================================

def termes(j4, j5, centre_j4, centre_j5):
    """Retourne les quatre termes du modele bilineaire pour un couple d'angles."""

    """
    Les angles sont centres sur la zone balayee avant tout
    calcul. Sans ce recentrage, le terme croise J4*J5 vaut
    plusieurs milliers alors que le terme constant vaut un :
    le systeme normal devient mal conditionne et la solution
    perd ses chiffres significatifs.
    """

    dj4 = j4 - centre_j4
    dj5 = j5 - centre_j5

    return [1.0, dj4, dj5, dj4 * dj5]


# =========================================================
# 6. AJUSTEMENT
# =========================================================

def ajusterModele(mesures, etiquette_robot="", commentaire=""):
    """Ajuste le modele bilineaire sur les mesures et retourne le dictionnaire du modele."""

    if len(mesures) < 4:
        print("ERREUR : il faut au moins 4 points pour ajuster 4 coefficients.")
        print("Recu :", len(mesures))
        return None

    # -----------------------------------------------------
    # Centrage
    # -----------------------------------------------------

    centre_j4 = sum(m["j4"] for m in mesures) / len(mesures)
    centre_j5 = sum(m["j5"] for m in mesures) / len(mesures)

    lignes = [termes(m["j4"], m["j5"], centre_j4, centre_j5) for m in mesures]

    coefficients_u = moindresCarres(lignes, [m["u"] for m in mesures])
    coefficients_v = moindresCarres(lignes, [m["v"] for m in mesures])

    if coefficients_u is None or coefficients_v is None:
        print("ERREUR : ajustement impossible, le systeme est singulier.")
        print("Cause probable : les points balayes sont alignes ou trop peu varies.")
        print("Faire varier J4 ET J5, pas seulement l'un des deux.")
        return None

    # -----------------------------------------------------
    # Enveloppe de validite
    # -----------------------------------------------------

    """
    On retient les bornes reellement balayees. C'est le
    domaine ou le modele a ete verifie, et donc le seul ou
    l'on accepte de viser.
    """

    modele = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "robot": etiquette_robot,
        "commentaire": commentaire,
        "nb_points": len(mesures),
        "unite": "cm",
        "centre_j4": centre_j4,
        "centre_j5": centre_j5,
        "coefficients_u": coefficients_u,
        "coefficients_v": coefficients_v,
        "enveloppe": {
            "j4_min": min(m["j4"] for m in mesures),
            "j4_max": max(m["j4"] for m in mesures),
            "j5_min": min(m["j5"] for m in mesures),
            "j5_max": max(m["j5"] for m in mesures),
            "u_min": min(m["u"] for m in mesures),
            "u_max": max(m["u"] for m in mesures),
            "v_min": min(m["v"] for m in mesures),
            "v_max": max(m["v"] for m in mesures),
        },
    }

    modele["residus"] = calculerResidus(modele, mesures)

    return modele


def calculerResidus(modele, mesures):
    """Retourne l'ecart entre impacts predits et mesures : moyenne quadratique et maximum."""

    ecarts = []

    for m in mesures:

        u, v = predireImpact(modele, m["j4"], m["j5"])
        ecarts.append(((u - m["u"]) ** 2 + (v - m["v"]) ** 2) ** 0.5)

    somme_carres = 0.0
    for e in ecarts:
        somme_carres += e * e

    return {
        "rms_cm": (somme_carres / len(ecarts)) ** 0.5,
        "max_cm": max(ecarts),
        "par_point_cm": [round(e, 3) for e in ecarts],
    }


# =========================================================
# 7. PREDICTION
# =========================================================

def predireImpact(modele, j4, j5):
    """Retourne (u, v) en centimetres, l'endroit ou le faisceau tombe pour ces deux angles."""

    t = termes(j4, j5, modele["centre_j4"], modele["centre_j5"])

    u = sum(modele["coefficients_u"][i] * t[i] for i in range(4))
    v = sum(modele["coefficients_v"][i] * t[i] for i in range(4))

    return u, v


# =========================================================
# 8. INVERSION
# =========================================================

def dansEnveloppe(modele, j4, j5, marge=MARGE_ENVELOPPE):
    """Indique si un couple d'angles reste dans la zone reellement balayee, a la marge pres."""

    e = modele["enveloppe"]

    return (e["j4_min"] - marge <= j4 <= e["j4_max"] + marge
            and e["j5_min"] - marge <= j5 <= e["j5_max"] + marge)


def anglesPourPoint(modele, u, v, marge=MARGE_ENVELOPPE):
    """Retourne (J4, J5) pour viser le point demande, ou None si le modele ne couvre pas ce point."""

    # -----------------------------------------------------
    # Point de depart
    # -----------------------------------------------------

    """
    On part du centre de la zone balayee. Le systeme est
    bilineaire, donc quasi lineaire sur une zone reduite :
    Newton converge en quelques iterations depuis ce point.
    """

    j4 = modele["centre_j4"]
    j5 = modele["centre_j5"]

    a = modele["coefficients_u"]
    b = modele["coefficients_v"]

    for _ in range(ITERATIONS_MAX):

        u_courant, v_courant = predireImpact(modele, j4, j5)

        ecart_u = u_courant - u
        ecart_v = v_courant - v

        if abs(ecart_u) < PRECISION_INVERSION and abs(ecart_v) < PRECISION_INVERSION:
            break

        # -------------------------------------------------
        # Jacobien du systeme bilineaire
        # -------------------------------------------------

        dj4 = j4 - modele["centre_j4"]
        dj5 = j5 - modele["centre_j5"]

        jacobien = [
            [a[1] + a[3] * dj5, a[2] + a[3] * dj4],
            [b[1] + b[3] * dj5, b[2] + b[3] * dj4],
        ]

        pas = resoudreSysteme(jacobien, [-ecart_u, -ecart_v])

        if pas is None:
            print("Inversion impossible : le modele est degenere en ce point.")
            return None

        j4 += pas[0]
        j5 += pas[1]

    else:

        """
        Sortie par epuisement des iterations : Newton n'a
        pas converge. Mieux vaut ne rien renvoyer qu'un
        couple d'angles approximatif dont personne ne
        saurait qu'il est faux.
        """

        print("Inversion non convergee apres", ITERATIONS_MAX, "iterations.")
        return None

    # -----------------------------------------------------
    # Domaine de validite
    # -----------------------------------------------------

    if not dansEnveloppe(modele, j4, j5, marge):
        print("Point hors de la zone balayee.")
        print("  angles calcules : J4 =", round(j4, 2), " J5 =", round(j5, 2))
        print("  zone balayee    : J4 de", round(modele["enveloppe"]["j4_min"], 1),
              "a", round(modele["enveloppe"]["j4_max"], 1),
              " J5 de", round(modele["enveloppe"]["j5_min"], 1),
              "a", round(modele["enveloppe"]["j5_max"], 1))
        print("Le modele n'a pas ete verifie la-bas : refus d'extrapoler.")
        return None

    return j4, j5


# =========================================================
# 9. VISEE
# =========================================================

def pointerSur(mc, modele, u, v):
    """Oriente la tete pour que le faisceau tombe sur (u, v) et retourne (succes, angles vises)."""

    angles = anglesPourPoint(modele, u, v)

    if angles is None:
        return False, None

    j4, j5 = angles

    """
    J6 est laisse tel quel : le laser etant coaxial a son
    axe, le tourner ne deplace pas le faisceau. Le bouger
    ne ferait qu'ajouter du jeu sans rien apporter.
    """

    tete_courante = lireTete(mc)

    if tete_courante is None:
        print("Lecture impossible, visee annulee.")
        return False, None

    j6 = tete_courante[2]

    print("Visee de (", round(u, 2), ",", round(v, 2), ") cm")
    print("  J4 =", round(j4, 2), " J5 =", round(j5, 2), " J6 inchange =", round(j6, 2))

    arrive, _ = bougerTeteAvecApproche(mc, [j4, j5, j6])

    return arrive, (j4, j5)


# =========================================================
# 10. LECTURE DES MESURES
# =========================================================

def chargerMesures(chemin=None, usage="ajustement", serie=None):
    """Relit le CSV de balayage et retourne les points utilisables pour l'ajustement."""

    if chemin is None:
        chemin = FICHIER_BALAYAGE

    if not os.path.isfile(chemin):
        print("Aucun fichier de balayage :", chemin)
        return []

    mesures = []

    try:

        with open(chemin, "r", newline="") as f:

            for ligne in csv.DictReader(f):

                if usage is not None and ligne.get("usage") != usage:
                    continue

                if serie is not None and ligne.get("serie") != serie:
                    continue

                """
                Une ligne sans impact est un point saute
                pendant le balayage (faisceau hors de la
                feuille). On l'ignore sans le signaler comme
                une erreur : c'est un cas normal.
                """

                if ligne.get("u_cm") in (None, "") or ligne.get("v_cm") in (None, ""):
                    continue

                try:
                    mesures.append({
                        "j4": float(ligne["J4"]),
                        "j5": float(ligne["J5"]),
                        "u": float(ligne["u_cm"]),
                        "v": float(ligne["v_cm"]),
                    })

                except (ValueError, KeyError):
                    continue

    except Exception as e:
        print("Impossible de relire le fichier de balayage :", e)
        return []

    return mesures


# =========================================================
# 11. ENREGISTREMENT DU MODELE
# =========================================================

def enregistrerModele(modele, chemin=None):
    """Ecrit le modele sur le disque et retourne son chemin."""

    if chemin is None:
        chemin = FICHIER_MODELE

    try:

        if not os.path.isdir(DOSSIER_RESULTATS):
            os.makedirs(DOSSIER_RESULTATS)

        with open(chemin, "w") as f:
            json.dump(modele, f, indent=2, ensure_ascii=False)

    except Exception as e:
        print("Impossible d'ecrire le modele :", e)
        return None

    return chemin


def chargerModele(chemin=None):
    """Relit le modele de pointage enregistre, ou None s'il n'existe pas."""

    if chemin is None:
        chemin = FICHIER_MODELE

    if not os.path.isfile(chemin):
        return None

    try:
        with open(chemin) as f:
            return json.load(f)

    except Exception as e:
        print("Modele illisible :", e)
        return None


def afficherModele(modele):
    """Affiche les caracteristiques du modele et la qualite de son ajustement."""

    print()
    print("-----------------------------------------")
    print("Modele de pointage")
    print("  robot     :", modele.get("robot", "non renseigne"))
    print("  date      :", modele.get("date"))
    print("  points    :", modele.get("nb_points"))

    e = modele["enveloppe"]
    print("  zone J4   :", round(e["j4_min"], 1), "a", round(e["j4_max"], 1), "deg")
    print("  zone J5   :", round(e["j5_min"], 1), "a", round(e["j5_max"], 1), "deg")
    print("  zone u    :", round(e["u_min"], 1), "a", round(e["u_max"], 1), "cm")
    print("  zone v    :", round(e["v_min"], 1), "a", round(e["v_max"], 1), "cm")

    r = modele.get("residus", {})
    print("  residus   :", round(r.get("rms_cm", 0), 2), "cm rms,",
          round(r.get("max_cm", 0), 2), "cm au pire")

    """
    Le residu ne dit PAS la precision de visee. Il dit a
    quel point le modele colle aux points qui ont servi a
    le construire, ce qui est toujours flatteur. La vraie
    precision se mesure sur des points ecartes de
    l'ajustement : c'est test_validation_pointage.py.
    """

    print()
    print("  Le residu mesure l'ajustement, pas la precision de visee.")
    print("  Pour celle-ci : test_validation_pointage.py")
    print("-----------------------------------------")


# =========================================================
# 12. EXECUTION AUTONOME
# =========================================================

def lireNombre(message):
    """Demande un nombre a l'operateur jusqu'a ce que la saisie soit valide."""

    while True:

        saisie = input(message).strip().replace(",", ".")

        try:
            return float(saisie)

        except ValueError:
            print("Saisie invalide, entrer un nombre.")


def main():
    """Console de visee : ajuste le modele si besoin, puis vise les points demandes."""

    print()
    print("=========================================")
    print("   POINTAGE DU LASER - BASE FIGEE")
    print("=========================================")
    print()
    print("ATTENTION : pointeur rouge classe 2 uniquement.")
    print()

    # -----------------------------------------------------
    # Modele
    # -----------------------------------------------------

    modele = chargerModele()

    if modele is None:

        print("Aucun modele enregistre, ajustement depuis le balayage.")

        mesures = chargerMesures()

        if len(mesures) < 4:
            print("Pas assez de points dans", FICHIER_BALAYAGE)
            print("Lancer d'abord test_balayage_tete.py.")
            return 1

        etiquette = input("Etiquette du robot (ex : demo) : ").strip()
        modele = ajusterModele(mesures, etiquette)

        if modele is None:
            return 1

        enregistrerModele(modele)

    afficherModele(modele)

    print()
    print("VERIFIER que ce modele vient bien du robot devant vous.")

    # -----------------------------------------------------
    # Connexion
    # -----------------------------------------------------

    from PU_PositionInitiale import connecterRobot

    mc = connecterRobot()

    if mc is None:
        print()
        print("Robot indisponible, programme interrompu.")
        return 1

    # -----------------------------------------------------
    # Boucle de visee
    # -----------------------------------------------------

    while True:

        print()
        print("-----------------------------------------")
        print("Point a viser, en carreaux (1 carreau =",
              COTE_CARREAU, "cm). q pour quitter.")

        saisie = input("Entree pour continuer / q : ").strip().lower()

        if saisie == "q":
            break

        u = lireNombre("u (carreaux) : ") * COTE_CARREAU
        v = lireNombre("v (carreaux) : ") * COTE_CARREAU

        arrive, angles = pointerSur(mc, modele, u, v)

        if angles is None:
            continue

        if not arrive:
            print("La tete n'a pas atteint sa consigne : mesure a ne pas retenir.")

    print()
    print("Les moteurs restent alimentes.")

    return 0


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
