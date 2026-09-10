"""
=========================================================
CALIBRATION GRILLE -> ANGLES J4 / J5
=========================================================

Objectif :
---------
Repondre a la question : ou mettre J4 et J5 pour que le
faisceau tombe sur un point donne du quadrillage pose au
sol, 30 cm sous le robot.

On pointe une vingtaine de noeuds de la grille a la main,
on releve les angles moteur a chaque fois, et on ajuste un
modele qui interpole entre ces points.

Deroulement d'une seance :
-------------------------
Le bras part en position initiale, puis pour chaque point :
la tete est RELACHEE, l'operateur l'oriente a la main
jusqu'a ce que le spot tombe sur un noeud du quadrillage,
et une touche valide la position.

Seuls les servos 4 et 5 sont relaches. J1 a J3 gardent
leur couple : relacher les six ferait tomber le bras d'un
coup sous son propre poids.

Phase 1 : la base ne bouge pas
------------------------------
J1, J2 et J3 sont relus au demarrage et renvoyes tels
quels a chaque mouvement. Ils ne sont JAMAIS commandes.
C'est ce qui rend ce script insensible a J3 bloque a
-124 degres sur le banc d'essai.

Le faisceau est coaxial a l'axe de J6 : tourner J6 fait
tourner le faisceau sur lui-meme sans deplacer sa droite.
J6 n'a donc aucun effet optique, il reste fige. Deux
angles pour deux coordonnees : le probleme est exactement
determine.

Le modele :
----------
Le faisceau part d'un point fixe et frappe un plan : c'est
une projection centrale. La direction du faisceau dans le
repere du poignet vaut

    Psi(t, f) = ( cos t cos f , sin t cos f , -sin f )

et la relation entre cette direction et le point d'impact
est alors EXACTEMENT une homographie 3x3 :

    [ u , v , 1 ]  proportionnel a  H . Psi(J4, J5 + b)

Le decalage b sur J5 rend compte du fait que le laser
n'est pas rigoureusement perpendiculaire a l'axe de J5.
Le decalage equivalent sur J4, lui, n'a pas besoin d'etre
estime : Psi(t + a, f) = Rz(a) . Psi(t, f), une rotation
constante a gauche, deja absorbee par H.

Neuf parametres en tout : les 8 degres de liberte de H,
plus b. Vingt points en donnent quarante equations.

Pourquoi pas un modele bilineaire :
----------------------------------
Le modele bilineaire de PU_Pointage.py est LOCAL. Son
propre docstring le chiffre : 0,6 cm au pire a +/-15 deg,
mais 3,8 cm a +/-30 deg. Couvrir la zone de 75 x 75 cm a
30 cm de hauteur demande +/-51 deg, hors de son domaine.
Et le meme docstring previent que densifier la grille n'y
change rien : c'est la forme du modele qui limite.

Le modele ci-dessus est geometrique, pas local : la plage
angulaire ne le gene pas. En prime son ajustement est
LINEAIRE (a b fige) et son inversion ANALYTIQUE, donc ni
numpy ni scipy, ni methode de Newton qui pourrait ne pas
converger. La Jetson tourne Python 3.13 hors environnement
conda, on ne suppose rien d'installe.

Ce que le modele ne fait pas :
-----------------------------
Il decrit UN plan, celui du quadrillage. Viser a une autre
hauteur demanderait une seconde campagne.

Il suppose le point d'emission fixe. Le laser etant
decale du centre de rotation du poignet, ce point se
deplace un peu quand la tete tourne. Cet ecart n'est pas
modelise : il ressort dans les residus, ce qui est le
comportement voulu.

Il suppose enfin |J5 + b| < 90 degres. Le script previent
si un point releve sort de ce domaine.

Utilisation :
------------
    python3 calibration_grille.py                  # seance de relevé
    python3 calibration_grille.py --ajuster-seulement   # sans robot

ATTENTION :
----------
Toute la calibration se fait au POINTEUR ROUGE CLASSE 2.
Le laser de puissance reste DEBRANCHE, pas seulement
eteint.

TENIR LA TETE avant chaque relachement. Sans couple, elle
retombe sous son propre poids. Une main dessus avant de
valider, pas apres.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import sys
import csv
import json
import math
import time
from datetime import datetime

"""
Import optionnel de la pose de reference de l'equipe.

PU_PositionInitiale n'ouvre pas le port serie a l'import,
cet appel est donc sans effet de bord. S'il venait a
manquer, le script continue avec sa propre copie de la
pose : il ne doit dependre d'aucun autre module pour
fonctionner.
"""

try:
    from PU_PositionInitiale import ANGLES_POSITION_INITIALE

except Exception:
    ANGLES_POSITION_INITIALE = None


# =========================================================
# 2. PARAMETRES
# =========================================================

PORT = "/dev/ttyTHS1"
BAUDRATE = 1000000

# Cote d'un carreau du quadrillage pose au sol.
COTE_CARREAU = 3.0         # centimetres

"""
Commande du pointeur.

Trois versions coexistent dans le depot : broche 33 au
niveau 0 (Version_Finale/test_laser.py), broche 33 au
niveau 1 (resultats/broche_laser_*.json) et broche 2 via
set_basic_output (ModuleGPIO/PU_GPIO.py).

On retient test_laser.py, le seul dont une execution
reussie soit tracee. Si le pointeur s'allume a l'envers,
il suffit d'echanger les deux constantes ci-dessous.
"""
BROCHE_LASER = 33
NIVEAU_ALLUME = 0
NIVEAU_ETEINT = 1

"""
Servos a relacher pour orienter la tete a la main.

Numerotation pymycobot : 1 a 6 pour J1 a J6. On ne
relache QUE ces deux la. release_all_servos() couperait
aussi l'epaule et le coude, et le bras s'effondrerait.
"""
SERVOS_TETE = [4, 5]       # J4 et J5

"""
Pose de depart. La valeur de reference est celle de
PU_PositionInitiale, importee plus haut quand ce module
est disponible : c'est la source de verite de l'equipe.
La copie locale ne sert que de repli.
"""
if ANGLES_POSITION_INITIALE is None:
    ANGLES_POSITION_INITIALE = [160.0, 50.0, -145.0, 40.0, 0.0, 0.0]

VITESSE_INIT = 40          # int, 1 a 100 : lent, le bras traverse son espace
TIMEOUT_INIT = 25          # secondes

# Retour visuel sur la LED de la carte Atom.
COULEUR_MAIN = (0, 255, 0)        # vert : la tete est libre
COULEUR_VERROUILLE = (255, 80, 0) # orange : la tete est tenue par les moteurs

VITESSE_TETE = 20          # int, 1 a 100 (impose par pymycobot)
TIMEOUT_TETE = 10          # secondes
TOLERANCE_TETE = 2.0       # degres
PERIODE_SCRUTATION = 0.2   # secondes

"""
Ecart tolere sur J1 a J3 avant de considerer que la base a
bouge. La calibration entiere est attachee a cette pose :
si elle change en cours de seance, les points releves
avant et apres ne decrivent plus la meme geometrie.
"""
TOLERANCE_BASE = 1.0       # degres

"""
Recul applique avant chaque arrivee.

Le bras est toujours amene a sa consigne depuis le meme
cote, pour que le jeu des reducteurs soit rattrape dans le
meme sens a chaque fois. Mesure sur ce bras : 2,19 deg de
dispersion en approche libre contre 0,09 deg en approche
identique.

C'est le seul levier sur le plancher mecanique, et ce
plancher domine au bord de la zone : 1,49 deg de
dispersion sur J5 valent 0,8 cm au centre mais environ
2,0 cm a 51 deg, la sensibilite croissant en 1/sin^2.
"""
RECUL_APPROCHE = 8.0       # degres

# Limites constructeur (pymycobot/robot_info.py).
J4_MIN, J4_MAX = -150.0, 150.0     # degres
J5_MIN, J5_MAX = -155.0, 160.0     # degres

# Marge acceptee au-dela de la zone reellement pointee.
MARGE_ENVELOPPE = 2.0      # centimetres

"""
Etendue angulaire minimale exigee sur CHACUN des deux axes.

Une grille degeneree en ligne n'est pas singuliere pour ce
modele : il s'ajuste parfaitement le long de la ligne, avec
un residu numeriquement nul, et extrapole n'importe quoi
ailleurs. Un residu nul n'est donc pas une preuve de bonne
calibration. On exige que les deux angles aient reellement
varie.
"""
ETENDUE_MINIMALE = 5.0     # degres

# Bornes de recherche du decalage sur J5.
OFFSET_J5_MIN = -60.0      # degres
OFFSET_J5_MAX = 60.0       # degres
OFFSET_J5_PAS = 2.0        # degres


# =========================================================
# 3. FICHIERS
# =========================================================

"""
Colonnes du CSV. L'entete est repris a l'identique de
PU_Pointage.ENTETE_BALAYAGE : les donnees restent lisibles
par le module existant.

Les angles MESURES et les angles COMMANDES sont tous les
deux enregistres. Le modele s'ajuste sur les MESURES :
ajuster sur la consigne ferait entrer le jeu mecanique
dans le modele.
"""

ENTETE_CSV = [
    "horodatage", "serie", "usage", "index",
    "j4_consigne", "j5_consigne",
    "J1", "J2", "J3", "J4", "J5", "J6",
    "u_cm", "v_cm", "commentaire"
]

DOSSIER_RESULTATS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "resultats")
FICHIER_MESURES = os.path.join(DOSSIER_RESULTATS, "calibration_grille.csv")
FICHIER_MODELE = os.path.join(DOSSIER_RESULTATS, "modele_grille.json")


# =========================================================
# 4. ALGEBRE LINEAIRE
# =========================================================

def resoudreSysteme(matrice, second_membre):
    """Resout un systeme carre par elimination de Gauss a pivot partiel, ou None si singulier."""

    n = len(matrice)

    # Copie de travail : on ne modifie pas les donnees de l'appelant.
    a = [list(ligne) + [second_membre[i]] for i, ligne in enumerate(matrice)]

    for colonne in range(n):

        # -------------------------------------------------
        # Recherche du pivot
        # -------------------------------------------------

        meilleure = colonne
        for ligne in range(colonne + 1, n):
            if abs(a[ligne][colonne]) > abs(a[meilleure][colonne]):
                meilleure = ligne

        if abs(a[meilleure][colonne]) < 1e-12:
            return None

        a[colonne], a[meilleure] = a[meilleure], a[colonne]

        # -------------------------------------------------
        # Elimination
        # -------------------------------------------------

        pivot = a[colonne][colonne]
        for ligne in range(colonne + 1, n):
            facteur = a[ligne][colonne] / pivot
            if facteur == 0.0:
                continue
            for k in range(colonne, n + 1):
                a[ligne][k] -= facteur * a[colonne][k]

    # -----------------------------------------------------
    # Remontee
    # -----------------------------------------------------

    solution = [0.0] * n
    for ligne in range(n - 1, -1, -1):
        somme = a[ligne][n]
        for k in range(ligne + 1, n):
            somme -= a[ligne][k] * solution[k]
        solution[ligne] = somme / a[ligne][ligne]

    return solution


def moindresCarres(lignes, valeurs):
    """Resout le systeme surdetermine par les equations normales, ou None si singulier."""

    if not lignes:
        return None

    n = len(lignes[0])

    normale = [[0.0] * n for _ in range(n)]
    membre = [0.0] * n

    for ligne, valeur in zip(lignes, valeurs):
        for i in range(n):
            membre[i] += ligne[i] * valeur
            for j in range(n):
                normale[i][j] += ligne[i] * ligne[j]

    return resoudreSysteme(normale, membre)


# =========================================================
# 5. LE MODELE
# =========================================================

def directionFaisceau(j4, j5_corrige):
    """Retourne la direction du faisceau dans le repere du poignet, pour deux angles en degres."""

    t = math.radians(j4)
    f = math.radians(j5_corrige)

    return [
        math.cos(t) * math.cos(f),
        math.sin(t) * math.cos(f),
        -math.sin(f),
    ]


def ajusterHomographie(mesures, offset_j5):
    """Ajuste les 8 coefficients de l'homographie a decalage fige, et retourne H en 9 nombres."""

    """
    Mise en equations dite DLT. La relation

        [u, v, 1]  proportionnel a  H . p

    n'est pas lineaire telle quelle a cause du facteur de
    proportionnalite. On la rend lineaire en ecrivant
    l'egalite des produits croises :

        u . (h31 p1 + h32 p2 + h33 p3) = h11 p1 + h12 p2 + h13 p3

    H etant defini a un facteur pres, on fige h33 = 1 : il
    reste 8 inconnues et deux equations par point.
    """

    lignes = []
    valeurs = []

    for m in mesures:

        p = directionFaisceau(m["j4"], m["j5"] + offset_j5)
        u = m["u"]
        v = m["v"]

        lignes.append([p[0], p[1], p[2], 0.0, 0.0, 0.0, -u * p[0], -u * p[1]])
        valeurs.append(u * p[2])

        lignes.append([0.0, 0.0, 0.0, p[0], p[1], p[2], -v * p[0], -v * p[1]])
        valeurs.append(v * p[2])

    coefficients = moindresCarres(lignes, valeurs)

    if coefficients is None:
        return None

    return list(coefficients) + [1.0]


def evaluerHomographie(h, p):
    """Retourne (u, v, w) pour une direction donnee, w etant le facteur de profondeur."""

    numerateur_u = h[0] * p[0] + h[1] * p[1] + h[2] * p[2]
    numerateur_v = h[3] * p[0] + h[4] * p[1] + h[5] * p[2]
    w = h[6] * p[0] + h[7] * p[1] + h[8] * p[2]

    if abs(w) < 1e-9:
        return None

    return (numerateur_u / w, numerateur_v / w, w)


def ecartQuadratique(h, offset_j5, mesures):
    """Retourne l'ecart quadratique moyen entre impacts predits et mesures, en centimetres."""

    total = 0.0

    for m in mesures:

        resultat = evaluerHomographie(h, directionFaisceau(m["j4"], m["j5"] + offset_j5))

        if resultat is None:
            return None

        total += (resultat[0] - m["u"]) ** 2 + (resultat[1] - m["v"]) ** 2

    return math.sqrt(total / len(mesures))


def chercherOffsetJ5(mesures):
    """Cherche le decalage sur J5 qui minimise l'ecart, et retourne (offset, H) ou None."""

    """
    Seul parametre non lineaire du modele, donc seul a
    demander une recherche. Balayage grossier sur toute la
    plage plausible, puis deux raffinements au dixieme.
    L'ajustement de H etant lineaire, chaque essai coute un
    systeme 8x8 : quelques centaines d'essais sont
    instantanes.
    """

    meilleur_offset = None
    meilleur_h = None
    meilleur_ecart = None

    borne_min = OFFSET_J5_MIN
    borne_max = OFFSET_J5_MAX
    pas = OFFSET_J5_PAS

    for _ in range(3):

        nombre = int(round((borne_max - borne_min) / pas))

        for indice in range(nombre + 1):

            offset = borne_min + indice * pas

            h = ajusterHomographie(mesures, offset)
            if h is None:
                continue

            ecart = ecartQuadratique(h, offset, mesures)
            if ecart is None:
                continue

            if meilleur_ecart is None or ecart < meilleur_ecart:
                meilleur_ecart = ecart
                meilleur_offset = offset
                meilleur_h = h

        if meilleur_offset is None:
            return None

        borne_min = meilleur_offset - pas
        borne_max = meilleur_offset + pas
        pas = pas / 10.0

    return (meilleur_offset, meilleur_h)


def ajusterModele(mesures, etiquette_robot="", base=None, commentaire=""):
    """Ajuste le modele complet sur les mesures et retourne le dictionnaire du modele, ou None."""

    if len(mesures) < 5:
        print("ERREUR : il faut au moins 5 points pour ajuster le modele.")
        print("Recu :", len(mesures))
        return None

    etendue_j4 = max(m["j4"] for m in mesures) - min(m["j4"] for m in mesures)
    etendue_j5 = max(m["j5"] for m in mesures) - min(m["j5"] for m in mesures)

    if etendue_j4 < ETENDUE_MINIMALE or etendue_j5 < ETENDUE_MINIMALE:
        print("ERREUR : les points ne font pas varier les deux angles.")
        print("Etendue relevee : J4 %.1f deg, J5 %.1f deg (minimum %.1f)."
              % (etendue_j4, etendue_j5, ETENDUE_MINIMALE))
        print("Un ajustement sur une ligne donne un residu nul et une")
        print("prediction fausse partout ailleurs. Refaire la grille.")
        return None

    resultat = chercherOffsetJ5(mesures)

    if resultat is None:
        print("ERREUR : ajustement impossible, le systeme est singulier.")
        print("Cause probable : les points releves sont alignes, ou un seul")
        print("des deux angles a varie. Faire varier J4 ET J5.")
        return None

    offset_j5, h = resultat

    # -----------------------------------------------------
    # Signe du facteur de profondeur
    # -----------------------------------------------------

    """
    L'inversion donne la direction a un signe pres : p et
    -p designent le meme point. Le signe correct est celui
    du facteur w, qui ne change pas sur la zone (il ne
    s'annulerait qu'a l'horizon du plan). On le releve donc
    une fois pour toutes sur les points ajustes.
    """

    signes = []
    for m in mesures:
        evaluation = evaluerHomographie(h, directionFaisceau(m["j4"], m["j5"] + offset_j5))
        if evaluation is not None:
            signes.append(1.0 if evaluation[2] > 0.0 else -1.0)

    if not signes:
        print("ERREUR : le modele ajuste ne permet d'evaluer aucun point.")
        return None

    if min(signes) != max(signes):
        print("ATTENTION : le facteur de profondeur change de signe sur la zone.")
        print("Le modele n'est pas valide sur toute l'etendue relevee.")

    signe_w = signes[0]

    # -----------------------------------------------------
    # Domaine de validite
    # -----------------------------------------------------

    modele = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "robot": etiquette_robot,
        "commentaire": commentaire,
        "modele": "homographie",
        "unite": "cm",
        "nb_points": len(mesures),
        "cote_carreau": COTE_CARREAU,
        "base": list(base) if base is not None else None,
        "homographie": h,
        "offset_j5": offset_j5,
        "signe_w": signe_w,
        "enveloppe": {
            "u_min": min(m["u"] for m in mesures),
            "u_max": max(m["u"] for m in mesures),
            "v_min": min(m["v"] for m in mesures),
            "v_max": max(m["v"] for m in mesures),
            "j4_min": min(m["j4"] for m in mesures),
            "j4_max": max(m["j4"] for m in mesures),
            "j5_min": min(m["j5"] for m in mesures),
            "j5_max": max(m["j5"] for m in mesures),
        },
    }

    modele["residus"] = calculerResidus(modele, mesures)

    return modele


def predireImpact(modele, j4, j5):
    """Retourne (u, v) en centimetres pour un couple d'angles, ou None si le modele n'evalue pas."""

    p = directionFaisceau(j4, j5 + modele["offset_j5"])

    resultat = evaluerHomographie(modele["homographie"], p)

    if resultat is None:
        return None

    return (resultat[0], resultat[1])


def anglesPourPoint(modele, u, v, marge=MARGE_ENVELOPPE):
    """Retourne (J4, J5) pour viser le point demande, ou None si le modele ne couvre pas ce point."""

    if not dansEnveloppe(modele, u, v, marge):
        return None

    # -----------------------------------------------------
    # Inversion
    # -----------------------------------------------------

    """
    Inversion analytique : on resout H . q = [u, v, 1], ce
    qui donne la direction du faisceau a un facteur pres,
    puis on lit les deux angles dessus. Pas d'iteration,
    donc pas de non-convergence possible.
    """

    h = modele["homographie"]

    matrice = [
        [h[0], h[1], h[2]],
        [h[3], h[4], h[5]],
        [h[6], h[7], h[8]],
    ]

    q = resoudreSysteme(matrice, [u, v, 1.0])

    if q is None:
        return None

    norme = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2)

    if norme < 1e-12:
        return None

    signe = modele.get("signe_w", 1.0)
    p = [signe * q[0] / norme, signe * q[1] / norme, signe * q[2] / norme]

    # -----------------------------------------------------
    # Lecture des angles
    # -----------------------------------------------------

    troisieme = max(-1.0, min(1.0, p[2]))

    j5 = math.degrees(-math.asin(troisieme)) - modele["offset_j5"]
    j4 = math.degrees(math.atan2(p[1], p[0]))

    if not (J4_MIN <= j4 <= J4_MAX and J5_MIN <= j5 <= J5_MAX):
        return None

    return (j4, j5)


def dansEnveloppe(modele, u, v, marge=MARGE_ENVELOPPE):
    """Indique si le point demande tombe dans la zone reellement relevee, a la marge pres."""

    """
    Hors de cette zone, le modele n'a ete verifie par
    aucune mesure. Une extrapolation silencieuse enverrait
    le faisceau n'importe ou en ayant l'air de fonctionner.
    """

    e = modele["enveloppe"]

    return (e["u_min"] - marge <= u <= e["u_max"] + marge
            and e["v_min"] - marge <= v <= e["v_max"] + marge)


def calculerResidus(modele, mesures):
    """Retourne l'ecart entre impacts predits et mesures : biais, dispersion et maximum."""

    """
    Le biais et la dispersion sont separes a dessein : un
    biais constant se rattrape par un decalage, une
    dispersion non.
    """

    ecarts = []

    for m in mesures:

        prediction = predireImpact(modele, m["j4"], m["j5"])

        if prediction is None:
            continue

        ecarts.append((prediction[0] - m["u"], prediction[1] - m["v"]))

    if not ecarts:
        return None

    nombre = len(ecarts)

    biais_u = sum(e[0] for e in ecarts) / nombre
    biais_v = sum(e[1] for e in ecarts) / nombre

    distances = [math.sqrt(e[0] ** 2 + e[1] ** 2) for e in ecarts]

    dispersion = math.sqrt(
        sum((e[0] - biais_u) ** 2 + (e[1] - biais_v) ** 2 for e in ecarts) / nombre
    )

    return {
        "nb_points": nombre,
        "rms_cm": math.sqrt(sum(d ** 2 for d in distances) / nombre),
        "max_cm": max(distances),
        "biais_u_cm": biais_u,
        "biais_v_cm": biais_v,
        "dispersion_cm": dispersion,
        "par_point_cm": distances,
    }


# =========================================================
# 6. TEMOIN : LE MODELE BILINEAIRE
# =========================================================

def ajusterBilineaire(mesures):
    """Ajuste le modele bilineaire de PU_Pointage sur les mesures, et retourne ses coefficients."""

    """
    Present uniquement pour comparaison. C'est le modele
    qu'utilise PU_Pointage.py :

        u = a0 + a1 J4 + a2 J5 + a3 J4 J5

    Il sert de temoin : afficher les deux residus cote a
    cote permet de choisir la forme du modele sur des
    chiffres mesures plutot que sur un raisonnement.
    """

    if len(mesures) < 4:
        return None

    centre_j4 = sum(m["j4"] for m in mesures) / len(mesures)
    centre_j5 = sum(m["j5"] for m in mesures) / len(mesures)

    lignes = []
    for m in mesures:
        dj4 = m["j4"] - centre_j4
        dj5 = m["j5"] - centre_j5
        lignes.append([1.0, dj4, dj5, dj4 * dj5])

    coefficients_u = moindresCarres(lignes, [m["u"] for m in mesures])
    coefficients_v = moindresCarres(lignes, [m["v"] for m in mesures])

    if coefficients_u is None or coefficients_v is None:
        return None

    return {
        "centre_j4": centre_j4,
        "centre_j5": centre_j5,
        "coefficients_u": coefficients_u,
        "coefficients_v": coefficients_v,
    }


def residusBilineaire(bilineaire, mesures):
    """Retourne l'ecart quadratique moyen et l'ecart maximum du temoin bilineaire, en centimetres."""

    if bilineaire is None:
        return None

    distances = []

    for m in mesures:

        dj4 = m["j4"] - bilineaire["centre_j4"]
        dj5 = m["j5"] - bilineaire["centre_j5"]
        base = [1.0, dj4, dj5, dj4 * dj5]

        u = sum(c * t for c, t in zip(bilineaire["coefficients_u"], base))
        v = sum(c * t for c, t in zip(bilineaire["coefficients_v"], base))

        distances.append(math.sqrt((u - m["u"]) ** 2 + (v - m["v"]) ** 2))

    if not distances:
        return None

    return {
        "rms_cm": math.sqrt(sum(d ** 2 for d in distances) / len(distances)),
        "max_cm": max(distances),
    }


# =========================================================
# 7. LECTURE ET ECRITURE DES FICHIERS
# =========================================================

def enregistrerPoint(serie, usage, index, consigne, mesure, impact, commentaire, chemin=None):
    """Ajoute une ligne au fichier de mesures, en creant l'entete au premier point."""

    """
    L'ecriture a lieu point par point, et non en fin de
    seance : une interruption ne doit rien faire perdre.
    """

    if chemin is None:
        chemin = FICHIER_MESURES

    try:
        os.makedirs(os.path.dirname(chemin), exist_ok=True)

        nouveau = not os.path.exists(chemin)

        with open(chemin, "a", newline="") as fichier:

            redacteur = csv.writer(fichier)

            if nouveau:
                redacteur.writerow(ENTETE_CSV)

            redacteur.writerow([
                datetime.now().isoformat(timespec="seconds"),
                serie, usage, index,
                round(consigne[0], 3), round(consigne[1], 3),
                round(mesure[0], 3), round(mesure[1], 3), round(mesure[2], 3),
                round(mesure[3], 3), round(mesure[4], 3), round(mesure[5], 3),
                round(impact[0], 3), round(impact[1], 3),
                commentaire,
            ])

        return True

    except Exception as e:
        print("Impossible d'ecrire le point dans le fichier.")
        print("Erreur :", e)
        return False


def chargerMesures(chemin=None, usage=None, serie=None):
    """Lit le fichier de mesures et retourne la liste des points exploitables."""

    if chemin is None:
        chemin = FICHIER_MESURES

    mesures = []

    try:
        with open(chemin, "r", newline="") as fichier:

            for ligne in csv.DictReader(fichier):

                if usage is not None and ligne.get("usage") != usage:
                    continue

                if serie is not None and ligne.get("serie") != serie:
                    continue

                # Un point saute n'a pas d'impact releve.
                if not ligne.get("u_cm") or not ligne.get("v_cm"):
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

    except FileNotFoundError:
        print("Fichier de mesures introuvable :", chemin)
        return []

    except Exception as e:
        print("Impossible de lire le fichier de mesures.")
        print("Erreur :", e)
        return []

    return mesures


def enregistrerModele(modele, chemin=None):
    """Ecrit le modele au format JSON et retourne True si l'ecriture a reussi."""

    if chemin is None:
        chemin = FICHIER_MODELE

    try:
        os.makedirs(os.path.dirname(chemin), exist_ok=True)

        with open(chemin, "w") as fichier:
            json.dump(modele, fichier, indent=2)

        return True

    except Exception as e:
        print("Impossible d'ecrire le modele.")
        print("Erreur :", e)
        return False


def chargerModele(chemin=None):
    """Lit le modele depuis son fichier JSON et le retourne, ou None."""

    if chemin is None:
        chemin = FICHIER_MODELE

    try:
        with open(chemin, "r") as fichier:
            return json.load(fichier)

    except FileNotFoundError:
        print("Aucun modele enregistre :", chemin)
        print("Lancer d'abord calibration_grille.py.")
        return None

    except Exception as e:
        print("Impossible de lire le modele.")
        print("Erreur :", e)
        return None


def afficherResidus(titre, residus):
    """Affiche le detail d'un jeu de residus."""

    print(titre)

    if residus is None:
        print("  aucun point exploitable.")
        return

    print("  points            :", residus["nb_points"])
    print("  ecart moyen (rms) : %.2f cm" % residus["rms_cm"])
    print("  ecart maximum     : %.2f cm" % residus["max_cm"])
    print("  biais             : u %+.2f cm   v %+.2f cm" % (residus["biais_u_cm"], residus["biais_v_cm"]))
    print("  dispersion        : %.2f cm" % residus["dispersion_cm"])


# =========================================================
# 8. LE ROBOT
# =========================================================

def connecterRobot(port=PORT, baudrate=BAUDRATE):
    """Ouvre la liaison serie, alimente les moteurs et retourne l'objet robot (None si echec)."""

    print("-----------------------------------------")
    print("Connexion au robot...")
    print("Port :", port)
    print("Baudrate :", baudrate)

    try:
        from pymycobot.mycobot280 import MyCobot280

    except Exception as e:
        print("Impossible d'importer pymycobot.")
        print("Erreur :", e)
        return None

    try:
        mc = MyCobot280(port, baudrate)

    except Exception as e:
        print("Impossible d'ouvrir la liaison serie.")
        print("Un autre script utilise peut-etre deja le port :")
        print("un seul processus a la fois sur", port)
        print("Erreur :", e)
        return None

    try:
        mc.power_on()

    except Exception as e:
        print("Impossible d'alimenter les moteurs.")
        print("Erreur :", e)
        return None

    # Laisser le temps aux servos de repondre.
    time.sleep(2)

    angles = lireAngles(mc)

    if angles is None:
        print("Le robot ne renvoie pas ses angles : verifier la carte Atom.")
        return None

    print("Robot connecte. Pose courante :", [round(a, 2) for a in angles])

    return mc


def lireAngles(mc):
    """Retourne les six angles mesures, ou None si la lecture est inexploitable."""

    try:
        angles = mc.get_angles()

    except Exception as e:
        print("Erreur de lecture des angles :", e)
        return None

    if not angles or len(angles) != 6:
        return None

    return [float(a) for a in angles]


def allumerPointeur(mc):
    """Allume le pointeur rouge et retourne True si la commande est passee."""

    try:
        mc.set_pin_mode(BROCHE_LASER, 1)
        time.sleep(0.5)
        mc.set_digital_output(BROCHE_LASER, NIVEAU_ALLUME)
        return True

    except Exception as e:
        print("Impossible d'allumer le pointeur :", e)
        return False


def eteindrePointeur(mc):
    """Eteint le pointeur rouge et retourne True si la commande est passee."""

    try:
        mc.set_digital_output(BROCHE_LASER, NIVEAU_ETEINT)
        return True

    except Exception as e:
        print("Impossible d'eteindre le pointeur :", e)
        return False


def bougerTete(mc, base, j4, j5, j6, vitesse=VITESSE_TETE, timeout=TIMEOUT_TETE):
    """Amene la tete aux angles demandes sans toucher a la base, et retourne (arrive, pose mesuree)."""

    """
    La pose envoyee reprend les angles de base RELUS, pas
    des valeurs de consigne. Le firmware attend une pose
    complete a six axes : lui renvoyer ce qu'il mesure deja
    revient a ne pas commander la base.

    L'attente ne scrute que J4 et J5. Sur ce bras J3 est
    bloque : une attente portant sur les six axes ne se
    terminerait jamais.
    """

    if not (J4_MIN <= j4 <= J4_MAX):
        print("Consigne J4 hors limites constructeur :", round(j4, 2))
        return (False, None)

    if not (J5_MIN <= j5 <= J5_MAX):
        print("Consigne J5 hors limites constructeur :", round(j5, 2))
        return (False, None)

    pose = [base[0], base[1], base[2], j4, j5, j6]

    try:
        mc.send_angles(pose, int(vitesse))

    except Exception as e:
        print("Envoi de la consigne refuse :", e)
        return (False, None)

    debut = time.time()
    mesure = None

    while time.time() - debut < timeout:

        time.sleep(PERIODE_SCRUTATION)

        mesure = lireAngles(mc)

        if mesure is None:
            continue

        if abs(mesure[3] - j4) <= TOLERANCE_TETE and abs(mesure[4] - j5) <= TOLERANCE_TETE:
            return (True, mesure)

    return (False, mesure)


def approcher(mc, base, j4, j5, j6):
    """Amene la tete a la consigne toujours par le meme cote, et retourne (arrive, pose mesuree)."""

    """
    Le recul est borne aux limites constructeur : pres
    d'une butee, on approche de moins loin plutot que de se
    faire refuser la consigne intermediaire.
    """

    j4_recul = max(J4_MIN, j4 - RECUL_APPROCHE)
    j5_recul = max(J5_MIN, j5 - RECUL_APPROCHE)

    bougerTete(mc, base, j4_recul, j5_recul, j6)

    return bougerTete(mc, base, j4, j5, j6)


def signalerCouleur(mc, couleur):
    """Change la couleur de la LED de la carte Atom, sans consequence si elle ne repond pas."""

    try:
        mc.set_color(couleur[0], couleur[1], couleur[2])

    except Exception:
        # Simple confort visuel : son echec ne doit rien interrompre.
        pass


def allerPositionInitiale(mc, angles=None, vitesse=VITESSE_INIT, timeout=TIMEOUT_INIT):
    """Amene le bras a sa pose de depart et retourne (arrive, pose mesuree)."""

    """
    L'arrivee n'est verifiee que sur J1, J2, J4 et J5. J3
    est bloque sur ce bras : l'attendre ferait echouer
    toutes les mises en position, alors que la calibration
    n'en a pas besoin. La base sera de toute facon relue
    telle qu'elle est, et c'est cette lecture qui fait foi.
    """

    if angles is None:
        angles = ANGLES_POSITION_INITIALE

    print()
    print("Mise en position initiale :", [round(a, 1) for a in angles])
    print("Le bras va traverser son espace de travail.")

    if not lireTexte("Zone degagee ? [O/n] ", "o").lower().startswith("o"):
        print("Mise en position annulee.")
        return (False, lireAngles(mc))

    try:
        mc.send_angles(list(angles), int(vitesse))

    except Exception as e:
        print("Envoi de la pose initiale refuse :", e)
        return (False, None)

    surveilles = [0, 1, 3, 4]

    debut = time.time()
    mesure = None

    while time.time() - debut < timeout:

        time.sleep(PERIODE_SCRUTATION)

        mesure = lireAngles(mc)

        if mesure is None:
            continue

        if all(abs(mesure[i] - angles[i]) <= TOLERANCE_TETE for i in surveilles):
            print("Position initiale atteinte.")
            return (True, mesure)

    print("ATTENTION : la pose initiale n'a pas ete atteinte dans le temps imparti.")

    if mesure is not None:
        ecarts = [round(mesure[i] - angles[i], 1) for i in surveilles]
        print("Ecarts restants sur J1, J2, J4, J5 :", ecarts)

    return (False, mesure)


def relacherTete(mc):
    """Coupe le couple de J4 et J5 pour orienter la tete a la main, et retourne True si c'est fait."""

    """
    C'est la ligne dangereuse du script. release_servo ne
    previent pas et ne retient rien : la tete devient molle
    immediatement. L'appelant doit s'etre assure que
    quelqu'un la tient.
    """

    try:
        for servo in SERVOS_TETE:
            mc.release_servo(servo)

    except Exception as e:
        print("Impossible de relacher la tete :", e)
        return False

    # Laisser le temps aux servos de lacher reellement.
    time.sleep(0.5)

    signalerCouleur(mc, COULEUR_MAIN)

    return True


def verrouillerTete(mc, pose=None):
    """Redonne le couple a J4 et J5 et fige la tete la ou elle est, puis retourne la pose tenue."""

    """
    La pose est lue AVANT de realimenter, puis renvoyee
    comme consigne : realimenter sans consigne laisse
    certains firmwares au repos, et la tete retombe.
    """

    if pose is None:
        pose = lireAngles(mc)

    if pose is None:
        print("Lecture impossible : la tete ne peut pas etre verrouillee proprement.")
        return None

    try:
        for servo in SERVOS_TETE:
            mc.focus_servo(servo)

        time.sleep(0.3)
        mc.send_angles(list(pose), int(VITESSE_TETE))

    except Exception as e:
        print("Impossible de verrouiller la tete :", e)
        return None

    signalerCouleur(mc, COULEUR_VERROUILLE)

    return pose


# =========================================================
# 9. DIALOGUE AVEC L'OPERATEUR
# =========================================================

def lireTexte(message, defaut=""):
    """Lit une reponse au clavier et retourne le texte saisi, ou le defaut si la saisie est vide."""

    try:
        reponse = input(message).strip()

    except EOFError:
        return defaut

    return reponse if reponse else defaut


def lireImpact():
    """Demande les coordonnees du noeud vise en carreaux et retourne (u, v) en centimetres, ou None."""

    print("  Coordonnees du noeud, en CARREAUX (ex : 3 -2).")
    print("  Taper s pour sauter ce point.")

    while True:

        reponse = lireTexte("  carreaux > ")

        if reponse.lower() in ("s", "saut"):
            return None

        morceaux = reponse.replace(",", " ").split()

        if len(morceaux) != 2:
            print("  Il faut deux nombres separes par un espace.")
            continue

        try:
            u = float(morceaux[0]) * COTE_CARREAU
            v = float(morceaux[1]) * COTE_CARREAU

        except ValueError:
            print("  Saisie illisible.")
            continue

        print("  soit u = %.1f cm, v = %.1f cm" % (u, v))
        return (u, v)


def pointageManuel(mc):
    """Relache la tete, attend que l'operateur l'oriente a la main, et retourne (issue, pose)."""

    """
    La tete est relachee puis reverrouillee A CHAQUE
    point. Laisser les servos libres entre deux points
    ferait perdre le pointage des qu'on lache le bras.
    """

    print("  TENIR LA TETE : elle va devenir molle.")

    if not lireTexte("  Prete a etre relachee ? [O/n] ", "o").lower().startswith("o"):
        return ("saut", None)

    if not relacherTete(mc):
        return ("saut", None)

    print("  Tete libre. Orienter le spot sur un noeud du quadrillage.")
    print("  [Entree] valider    s : sauter ce point    f : terminer")

    reponse = lireTexte("  > ").lower()

    # -----------------------------------------------------
    # Releve puis reverrouillage
    # -----------------------------------------------------

    """
    Les angles sont lus AVANT de rendre le couple : c'est
    la pose reellement tenue a la main qui nous interesse.
    Le verrouillage suit immediatement, quelle que soit
    l'issue, pour ne jamais laisser la tete molle.
    """

    pose = lireAngles(mc)

    verrouillerTete(mc, pose)

    if reponse.startswith("f"):
        return ("fin", pose)

    if reponse.startswith("s"):
        return ("saut", pose)

    if pose is None:
        print("  Lecture des angles impossible, point abandonne.")
        return ("saut", None)

    return ("ok", pose)


# =========================================================
# 10. AJUSTEMENT D'UNE SEANCE
# =========================================================

def controlerDomaine(modele, mesures):
    """Previent si des points relevés sortent du domaine ou le modele reste inversible."""

    """
    L'inversion lit J5 par un arc sinus, dont l'image est
    [-90, 90] degres. Un point releve au-dela ne serait pas
    retrouve par anglesPourPoint : mieux vaut le dire tout
    de suite que le decouvrir a la visee.
    """

    limites = [m for m in mesures if abs(m["j5"] + modele["offset_j5"]) > 85.0]

    if limites:
        print()
        print("ATTENTION :", len(limites), "point(s) sont a plus de 85 degres apres correction.")
        print("Le modele ne sait pas les inverser. Reduire la plage de J5.")


def ajusterEtEnregistrer(serie=None, base=None, etiquette_robot="", commentaire=""):
    """Ajuste le modele sur les points de la seance, affiche le bilan et l'enregistre."""

    ajustement = chargerMesures(usage="ajustement", serie=serie)
    validation = chargerMesures(usage="validation", serie=serie)

    print()
    print("=========================================")
    print("AJUSTEMENT DU MODELE")
    print("=========================================")
    print("Points d'ajustement :", len(ajustement))
    print("Points de validation :", len(validation))

    if len(ajustement) < 5:
        print()
        print("Pas assez de points d'ajustement (5 minimum).")
        return None

    modele = ajusterModele(ajustement, etiquette_robot=etiquette_robot,
                           base=base, commentaire=commentaire)

    if modele is None:
        return None

    print()
    print("Decalage estime sur J5 : %+.2f deg" % modele["offset_j5"])

    controlerDomaine(modele, ajustement)

    print()
    afficherResidus("Residus sur les points d'AJUSTEMENT :", modele["residus"])

    # -----------------------------------------------------
    # Validation croisee
    # -----------------------------------------------------

    """
    Les points de validation n'ont servi a rien dans
    l'ajustement. C'est la seule mesure honnete de
    l'erreur : evaluer le modele sur les points qui l'ont
    produit ne mesure que sa propre calibration.
    """

    if validation:
        residus_validation = calculerResidus(modele, validation)
        modele["residus_validation"] = residus_validation
        print()
        afficherResidus("Residus sur les points de VALIDATION :", residus_validation)

        if residus_validation is not None and modele["residus"] is not None:
            if residus_validation["rms_cm"] > 3.0 * modele["residus"]["rms_cm"]:
                print()
                print("ATTENTION : l'erreur en validation est bien pire qu'en ajustement.")
                print("Le modele colle aux points de calibration sans decrire la zone.")
    else:
        print()
        print("Aucun point de validation : l'erreur annoncee n'est pas verifiee.")
        print("Relever 4 ou 5 points hors grille avec l'usage 'validation'.")

    # -----------------------------------------------------
    # Temoin bilineaire
    # -----------------------------------------------------

    temoin = residusBilineaire(ajusterBilineaire(ajustement), ajustement)

    if temoin is not None:
        print()
        print("Pour comparaison, le modele bilineaire de PU_Pointage")
        print("sur les memes points : rms %.2f cm, max %.2f cm" % (temoin["rms_cm"], temoin["max_cm"]))
        modele["temoin_bilineaire"] = temoin

    # -----------------------------------------------------
    # Enregistrement
    # -----------------------------------------------------

    if enregistrerModele(modele):
        print()
        print("Modele enregistre :", FICHIER_MODELE)
        print("Il peut maintenant servir avec calibration_test.py.")

    return modele


# =========================================================
# 11. SEANCE DE RELEVE
# =========================================================

def afficherProtocole():
    """Affiche les consignes de securite et de repartition des points avant la seance."""

    print("=========================================")
    print("CALIBRATION DE LA GRILLE")
    print("=========================================")
    print()
    print("SECURITE : pointeur rouge classe 2 uniquement.")
    print("Le laser de puissance doit etre DEBRANCHE, pas seulement eteint.")
    print()
    print("Deroulement de chaque point :")
    print("  1. la tete est relachee -- LA TENIR avant de valider ;")
    print("  2. vous l'orientez a la main sur un noeud du quadrillage ;")
    print("  3. [Entree] releve les angles et reverrouille la tete ;")
    print("  4. le robot refait l'approche par les moteurs, comme au tir :")
    print("     le spot peut se decaler un peu, c'est normal ;")
    print("  5. vous lisez ou le spot est tombe APRES ce mouvement.")
    print()
    print("Seuls les servos 4 et 5 sont relaches. L'epaule et le coude")
    print("gardent leur couple : le bras ne s'effondre pas.")
    print()
    print("Repartition des points :")
    print("  - une grille reguliere, environ 5 x 4, usage 'ajustement' ;")
    print("  - 4 ou 5 points hors de cette grille, usage 'validation'.")
    print()
    print("Les points d'ajustement doivent inclure les QUATRE COINS de la")
    print("zone utile. Le modele refuse d'extrapoler : ne pointer que le")
    print("centre reviendrait a ne calibrer que le centre.")
    print()
    print("Carreau de %.1f cm. Le noeud (0, 0) est celui que vous choisissez" % COTE_CARREAU)
    print("comme origine : gardez le meme pendant toute la seance.")
    print()


def seanceDeReleve(mc, base, j6, serie, usage):
    """Deroule le pointage des points un par un et retourne le nombre de points enregistres."""

    index = 0
    enregistres = 0

    while True:

        index += 1

        print()
        print("-----------------------------------------")
        print("Point", index, "  (serie '%s', usage '%s')" % (serie, usage))

        # -------------------------------------------------
        # Pointage a la main
        # -------------------------------------------------

        issue, pose = pointageManuel(mc)

        if issue == "fin":
            break

        if issue == "saut":
            index -= 1
            continue

        j4 = pose[3]
        j5 = pose[4]

        # -------------------------------------------------
        # La base a-t-elle bouge ?
        # -------------------------------------------------

        """
        J1 a J3 gardent leur couple, mais une main appuyee
        un peu fort les deplace quand meme. Si la base
        bouge en cours de seance, les points releves avant
        et apres ne decrivent plus la meme geometrie : tout
        le modele est fausse, sans que rien ne le signale.
        """

        ecarts_base = [abs(pose[i] - base[i]) for i in range(3)]

        if max(ecarts_base) > TOLERANCE_BASE:
            print("  ATTENTION : la base a bouge de %.2f deg depuis le debut." % max(ecarts_base))
            print("  Ecarts J1, J2, J3 :", [round(e, 2) for e in ecarts_base])
            print("  Les points deja releves ne decrivent plus la meme geometrie.")

            if not lireTexte("  Garder ce point quand meme ? [o/N] ", "n").lower().startswith("o"):
                index -= 1
                continue

        # -------------------------------------------------
        # Reprise d'approche
        # -------------------------------------------------

        """
        La tete est ramenee sur ces memes angles par les
        moteurs, en arrivant du meme cote qu'au tir. Le jeu
        des reducteurs se replace alors comme il le fera en
        service : le spot peut se decaler un peu, et c'est
        precisement cette position-la qu'il faut relever.

        Sans cette etape, la calibration decrirait une pose
        obtenue a la main, que le bras ne reproduit pas
        quand il y revient tout seul.
        """

        print("  Reprise d'approche...")

        arrive, mesure = approcher(mc, base, j4, j5, j6)

        if mesure is None:
            print("  Lecture impossible, point abandonne.")
            index -= 1
            continue

        if not arrive:
            print("  ATTENTION : la tete n'est pas revenue a la consigne.")

        j4 = mesure[3]
        j5 = mesure[4]

        # -------------------------------------------------
        # Releve de l'impact
        # -------------------------------------------------

        print("  Lire la position du spot MAINTENANT : il a pu se decaler.")

        impact = lireImpact()

        if impact is None:
            index -= 1
            continue

        commentaire = lireTexte("  Commentaire (facultatif) > ")

        if enregistrerPoint(serie, usage, index, (j4, j5), mesure, impact, commentaire):
            enregistres += 1
            print("  Point enregistre : J4 = %+.2f  J5 = %+.2f  ->  u = %.1f  v = %.1f"
                  % (j4, j5, impact[0], impact[1]))

    return enregistres


def main():
    """Deroule la calibration complete et retourne le code de sortie du programme."""

    # -----------------------------------------------------
    # Ajustement seul, sans robot
    # -----------------------------------------------------

    if "--ajuster-seulement" in sys.argv:

        serie = lireTexte("Serie a ajuster (vide = toutes) > ") or None

        return 0 if ajusterEtEnregistrer(serie=serie) is not None else 1

    # -----------------------------------------------------
    # Seance complete
    # -----------------------------------------------------

    afficherProtocole()

    mc = connecterRobot()

    if mc is None:
        return 1

    # -----------------------------------------------------
    # Pose de depart
    # -----------------------------------------------------

    """
    Toute la calibration est attachee a la pose de base.
    On part donc d'une pose connue et reproductible, pour
    qu'une campagne ulterieure puisse etre comparee a
    celle-ci plutot qu'a une position de hasard.
    """

    arrive, base_complete = allerPositionInitiale(mc)

    if base_complete is None:
        return 1

    if not arrive:
        print()
        if not lireTexte("Continuer malgre tout ? [o/N] ", "n").lower().startswith("o"):
            return 1

    j6 = base_complete[5]
    base = base_complete[0:3]

    print()
    print("Base figee a J1 = %+.2f  J2 = %+.2f  J3 = %+.2f" % (base[0], base[1], base[2]))
    print("Ces trois axes ne seront pas commandes. Ne pas toucher au bras")
    print("pendant la seance : toute la calibration est attachee a cette pose.")

    serie = lireTexte("\nNom de la serie > ", datetime.now().strftime("%Y%m%d_%H%M"))
    etiquette = lireTexte("Etiquette du robot (facultatif) > ")

    code_retour = 0

    try:
        if not allumerPointeur(mc):
            print("Le pointeur ne s'allume pas : verifier la broche", BROCHE_LASER)

        print()
        print("=== POINTS D'AJUSTEMENT ===")
        nombre = seanceDeReleve(mc, base, j6, serie, "ajustement")
        print()
        print(nombre, "point(s) d'ajustement enregistres.")

        suite = lireTexte("\nRelever maintenant les points de validation ? [O/n] ", "o")

        if not suite.lower().startswith("n"):
            print()
            print("=== POINTS DE VALIDATION ===")
            print("Choisir des noeuds qui n'ont PAS servi a l'ajustement.")
            nombre = seanceDeReleve(mc, base, j6, serie, "validation")
            print()
            print(nombre, "point(s) de validation enregistres.")

        if ajusterEtEnregistrer(serie=serie, base=base, etiquette_robot=etiquette) is None:
            code_retour = 1

    finally:

        """
        Deux choses doivent arriver quoi qu'il advienne, y
        compris sur interruption au clavier : le pointeur
        s'eteint, et la tete reprend son couple. Une
        interruption pendant que les servos sont relaches
        laisserait sinon la tete molle.

        Les moteurs, eux, restent alimentes : les couper
        ferait retomber le bras sous son propre poids.
        """

        verrouillerTete(mc)
        eteindrePointeur(mc)
        print()
        print("Tete verrouillee, pointeur eteint. Moteurs laisses sous tension.")

    return code_retour


# =========================================================
# 12. EXECUTION AUTONOME
# =========================================================

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
