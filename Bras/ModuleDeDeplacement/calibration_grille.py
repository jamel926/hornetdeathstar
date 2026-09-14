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
Le bras part en position initiale, puis pour chaque point
l'operateur amene le spot sur un noeud du quadrillage avec
les fleches du clavier, et valide par entree.

Tout le deplacement passe par les moteurs, et chaque pas
arrive du meme cote que l'approche du tir. La pose relevee
est donc exactement celle que le bras retrouvera : il n'y a
aucun ecart entre la position pointee et la position
enregistree.

Une touche permet de relacher la tete pour la placer
grossierement a la main, ce qui evite des dizaines d'appuis
pour traverser la zone. Seuls les servos 4 et 5 sont alors
relaches : relacher les six ferait tomber le bras d'un coup
sous son propre poids.

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
Lecture des fleches du clavier. Ces trois modules sont
dans la bibliotheque standard, mais absents de Windows :
sans eux, le pointage retombe sur des commandes tapees.
"""

try:
    import tty
    import termios
    import select
    CLAVIER_DIRECT = True

except ImportError:
    CLAVIER_DIRECT = False

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

"""
Le CARREAU est l'unite de travail du script, de bout en
bout : saisie, modele, residus, affichages. Rien n'est
converti nulle part.

C'est ce que l'operateur lit reellement sur le quadrillage.
Melanger deux unites dans une chaine de mesure finit
toujours par produire un facteur d'echelle silencieux, et
ici il vaudrait 3.

La valeur ci-dessous ne sert qu'a deux choses : relire les
campagnes enregistrees en centimetres, et convertir les
resultats quand on veut les comparer a une longueur
physique.
"""
COTE_CARREAU = 3.0         # centimetres, pour information

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
Pose de depart : celle de PU_PositionInitiale, importee
plus haut. C'est la source de verite de l'equipe, et elle
change (J1 est passe de 160 a 10 degres le 10/09).

Volontairement AUCUNE copie de repli. Une valeur figee ici
serait perimee au premier changement, et un repli silencieux
enverrait le bras 150 degres a cote en ayant l'air de
fonctionner. Si le module manque, le script part de la pose
courante et le dit.
"""

VITESSE_INIT = 100          # int, 1 a 100 : lent, le bras traverse son espace
TIMEOUT_INIT = 25          # secondes

# Retour visuel sur la LED de la carte Atom.
COULEUR_MAIN = (0, 255, 0)        # vert : la tete est libre
COULEUR_VERROUILLE = (255, 80, 0) # orange : la tete est tenue par les moteurs

VITESSE_TETE = 100          # int, 1 a 100 (impose par pymycobot)
TIMEOUT_TETE = 10          # secondes
TOLERANCE_TETE = 2.0       # degres
PERIODE_SCRUTATION = 0.2   # secondes

"""
Fin de mouvement.

TOLERANCE_TETE ne suffit pas a decider qu'un mouvement est
termine : a 30 cm de hauteur, 2 degres valent deja 1 cm, et
le bras est encore en train de bouger quand il passe sous ce
seuil. Relever les angles a cet instant les fait correspondre
a une position que le spot a deja quittee.

On attend donc que les angles ne bougent plus reellement :
plusieurs lectures consecutives identiques, is_moving() a
l'appui quand le firmware le renseigne.
"""
SEUIL_STABILITE = 0.15     # degres entre deux lectures
LECTURES_STABLES = 3       # lectures consecutives sans mouvement
TIMEOUT_STABILITE = 4      # secondes

"""
Pas de deplacement au clavier. Un pas de 0,5 degre vaut
environ 2,6 mm au sol a 30 cm : c'est la finesse utile pour
poser le spot sur un noeud.
"""
PAS_JOG_DEFAUT = 1.0       # degres
PAS_JOG_MIN = 0.1          # degres
PAS_JOG_MAX = 10.0         # degres

"""
Depassement applique quand un pas de jog va dans le sens
oppose au sens d'arrivee.

Le jeu des reducteurs se place du cote du DERNIER mouvement.
En depassant puis en revenant, toute position atteinte au
clavier l'est en arrivant du meme cote qu'au tir, ou
approcher() fait de meme. Sans cela, la position relevee ne
serait pas celle que le bras retrouve quand il y revient
tout seul.
"""
DEPASSEMENT_JOG = 3.0      # degres

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

"""
Rattrapage de l'ecart de repos.

Le bras ne s'arrete pas sur la consigne : il s'arrete la ou
son asservissement se tait, un peu en dessous, la tete tirant
vers le bas. Mesure sur les trois tirs du 11/09 : 0,7 a 1,8
degre de deficit sur J4 ET sur J5, soit 1,5 a 2,3 cm au sol.

Rien ne le signalait, parce que TOLERANCE_TETE vaut 2 degres :
bougerTete() declarait "arrive" et rendait la main.

La correction est directe : relire la pose atteinte, ajouter
l'ecart a la consigne, recommencer. L'ecart de repos etant a
peu pres constant pour une pose donnee, deux passes suffisent.

TOLERANCE_POINTAGE vaut 0,2 degre, soit environ 2 mm au sol :
en dessous, on corrigerait le bruit de lecture des codeurs
(pas de 0,088 degre) sans rien gagner.
"""
TOLERANCE_POINTAGE = 0.2   # degres
CORRECTIONS_MAX = 3        # passes de rattrapage apres l'arrivee

# Limites constructeur (pymycobot/robot_info.py).
J4_MIN, J4_MAX = -150.0, 150.0     # degres
J5_MIN, J5_MAX = -155.0, 160.0     # degres

# Marge acceptee au-dela de la zone reellement pointee.
MARGE_ENVELOPPE = 0.7      # carreaux

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

# Iterations maximales du raffinement final.
RAFFINEMENT_TOURS = 60

"""
Seuils d'interpretation des residus, en carreaux.

Le plancher n'est pas le modele : l'operateur pose le spot
a vue sur un noeud et relit sa position sur le quadrillage.
En dessous d'un tiers de carreau, c'est la mesure qui limite,
pas l'ajustement -- il n'y a plus rien a gagner de ce cote.
Au-dela d'un carreau, quelque chose ne va pas.
"""
RESIDU_BON = 0.35          # carreaux
RESIDU_MAUVAIS = 0.70      # carreaux

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
    "u_carreaux", "v_carreaux"
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
    """Retourne l'ecart quadratique moyen entre impacts predits et mesures, en carreaux."""

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


def residusModele(params, mesures):
    """Retourne les ecarts signes du modele aux mesures, deux par point, ou None."""

    h = list(params[:8]) + [1.0]
    offset = params[8]

    ecarts = []

    for m in mesures:

        evaluation = evaluerHomographie(h, directionFaisceau(m["j4"], m["j5"] + offset))

        if evaluation is None:
            return None

        ecarts.append(evaluation[0] - m["u"])
        ecarts.append(evaluation[1] - m["v"])

    return ecarts


def raffinerModele(params, mesures, tours=RAFFINEMENT_TOURS):
    """Affine les neuf parametres en minimisant l'ecart reel, et retourne les parametres obtenus."""

    """
    L'ajustement DLT ne minimise pas l'ecart en carreaux mais
    une erreur algebrique, celle des produits croises. Les
    deux ne coincident pas : sur une campagne reelle le DLT
    seul laissait 2,4 cm d'ecart moyen la ou le meme modele,
    ajuste sur l'ecart reel, en laisse 0,9.

    Le DLT reste le point de depart -- il donne une solution
    correcte sans rien supposer -- et cette descente de
    Levenberg-Marquardt termine le travail. Jacobien calcule
    par differences finies : neuf parametres, quelques
    dizaines de residus, le cout est negligeable.
    """

    courant = list(params)

    ecarts = residusModele(courant, mesures)

    if ecarts is None:
        return list(params)

    cout = sum(e * e for e in ecarts)
    amorti = 1e-3

    nombre = len(courant)

    for _ in range(tours):

        # -------------------------------------------------
        # Jacobien par differences finies
        # -------------------------------------------------

        jacobien = []

        for i in range(nombre):

            pas = max(1e-7, abs(courant[i]) * 1e-6)

            decale = list(courant)
            decale[i] += pas

            voisins = residusModele(decale, mesures)

            if voisins is None:
                return courant

            jacobien.append([(a - b) / pas for a, b in zip(voisins, ecarts)])

        # -------------------------------------------------
        # Systeme normal amorti
        # -------------------------------------------------

        normale = [[sum(jacobien[i][k] * jacobien[j][k] for k in range(len(ecarts)))
                    for j in range(nombre)] for i in range(nombre)]

        membre = [-sum(jacobien[i][k] * ecarts[k] for k in range(len(ecarts)))
                  for i in range(nombre)]

        for i in range(nombre):
            normale[i][i] *= (1.0 + amorti)

        correction = resoudreSysteme(normale, membre)

        if correction is None:
            break

        essai = [courant[i] + correction[i] for i in range(nombre)]

        ecarts_essai = residusModele(essai, mesures)

        if ecarts_essai is None:
            amorti *= 10.0
            continue

        cout_essai = sum(e * e for e in ecarts_essai)

        # -------------------------------------------------
        # On n'accepte qu'une amelioration
        # -------------------------------------------------

        if cout_essai < cout:
            courant = essai
            ecarts = ecarts_essai

            if cout - cout_essai < 1e-12 * max(cout, 1.0):
                cout = cout_essai
                break

            cout = cout_essai
            amorti = max(1e-9, amorti / 3.0)

        else:
            amorti *= 10.0

            if amorti > 1e8:
                break

    return courant


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
    # Raffinement sur l'ecart reel
    # -----------------------------------------------------

    params = raffinerModele(list(h[:8]) + [offset_j5], mesures)

    h = list(params[:8]) + [1.0]
    offset_j5 = params[8]

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
        "unite": "carreau",
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
    """Retourne (u, v) en carreaux pour un couple d'angles, ou None si le modele n'evalue pas."""

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
        "rms_carreaux": math.sqrt(sum(d ** 2 for d in distances) / nombre),
        "max_carreaux": max(distances),
        "biais_u_carreaux": biais_u,
        "biais_v_carreaux": biais_v,
        "dispersion_carreaux": dispersion,
        "par_point_carreaux": distances,
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
    """Retourne l'ecart quadratique moyen et l'ecart maximum du temoin bilineaire, en carreaux."""

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
        "rms_carreaux": math.sqrt(sum(d ** 2 for d in distances) / len(distances)),
        "max_carreaux": max(distances),
    }


# =========================================================
# 7. LECTURE ET ECRITURE DES FICHIERS
# =========================================================

def enteteCompatible(chemin):
    """Verifie que le fichier existant porte bien les colonnes que l'on s'apprete a ecrire."""

    """
    L'entete n'est ecrit qu'a la creation du fichier. Une
    campagne suivante s'ajoute donc SOUS l'entete deja en
    place, quel qu'il soit -- et personne ne le relit.

    C'est exactement ce qui est arrive a la campagne du
    11/09 : les impacts etaient saisis en carreaux, le
    fichier portait encore des colonnes u_cm / v_cm, et la
    relecture les a divises par trois sans rien signaler.
    Modele trois fois trop petit, zone calibree amputee des
    deux tiers, residus flatteurs d'un facteur trois.

    Une campagne coute une matinee : mieux vaut refuser
    d'ecrire que produire un fichier qui se relit faux.
    """

    try:
        with open(chemin, "r", newline="") as fichier:
            entete = next(csv.reader(fichier), None)

    except Exception as e:
        print("Impossible de relire l'entete du fichier de mesures.")
        print("Erreur :", e)
        return False

    if entete is None:
        return True

    # Les colonnes en trop a la fin (un commentaire libre, par
    # exemple) ne genent pas : seules celles que l'on ecrit comptent.
    if entete[:len(ENTETE_CSV)] == ENTETE_CSV:
        return True

    print()
    print("REFUS D'ECRIRE : le fichier de mesures n'a pas les colonnes attendues.")
    print("  fichier  :", chemin)
    print("  attendu  :", ",".join(ENTETE_CSV))
    print("  trouve   :", ",".join(entete[:len(ENTETE_CSV)]))
    print("Ajouter des points ici produirait un modele faux d'un facteur d'echelle.")
    print("Convertir l'ancien fichier, ou en commencer un nouveau.")

    return False


def enregistrerPoint(serie, usage, index, consigne, mesure, impact, chemin=None):
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

        if not nouveau and not enteteCompatible(chemin):
            return False

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
            ])

        return True

    except Exception as e:
        print("Impossible d'ecrire le point dans le fichier.")
        print("Erreur :", e)
        return False


def lireImpactDuFichier(ligne):
    """Retourne l'impact d'une ligne de mesures en carreaux, ou None si elle n'en porte pas."""

    """
    Les campagnes faites avant le passage au carreau portent
    des colonnes en centimetres. Elles restent exploitables :
    on les convertit a la lecture plutot que de les perdre.
    """

    try:
        if ligne.get("u_carreaux") and ligne.get("v_carreaux"):
            return (float(ligne["u_carreaux"]), float(ligne["v_carreaux"]))

        if ligne.get("u_cm") and ligne.get("v_cm"):
            return (float(ligne["u_cm"]) / COTE_CARREAU,
                    float(ligne["v_cm"]) / COTE_CARREAU)

    except ValueError:
        return None

    return None


def chargerMesures(chemin=None, usage=None, serie=None):
    """Lit le fichier de mesures et retourne la liste des points exploitables."""

    if chemin is None:
        chemin = FICHIER_MESURES

    mesures = []

    try:
        with open(chemin, "r", newline="") as fichier:

            lecteur = csv.DictReader(fichier)

            # La conversion depuis les centimetres est legitime, mais
            # elle doit se voir : c'est elle qui a fait passer une
            # campagne entiere au tiers de son echelle.
            colonnes = lecteur.fieldnames or []

            if "u_carreaux" not in colonnes and "u_cm" in colonnes:
                print("Fichier de mesures en centimetres : conversion en carreaux"
                      " (division par %.0f)." % COTE_CARREAU)

            for ligne in lecteur:

                if usage is not None and ligne.get("usage") != usage:
                    continue

                if serie is not None and ligne.get("serie") != serie:
                    continue

                impact = lireImpactDuFichier(ligne)

                if impact is None:
                    continue

                try:
                    mesures.append({
                        "j4": float(ligne["J4"]),
                        "j5": float(ligne["J5"]),
                        "u": impact[0],
                        "v": impact[1],
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
    print("  ecart moyen (rms) : %.2f carreaux" % residus["rms_carreaux"])
    print("  ecart maximum     : %.2f carreaux" % residus["max_carreaux"])
    print("  biais             : u %+.2f   v %+.2f" % (residus["biais_u_carreaux"], residus["biais_v_carreaux"]))
    print("  dispersion        : %.2f carreaux" % residus["dispersion_carreaux"])


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
    proche = False

    while time.time() - debut < timeout:

        time.sleep(PERIODE_SCRUTATION)

        mesure = lireAngles(mc)

        if mesure is None:
            continue

        if abs(mesure[3] - j4) <= TOLERANCE_TETE and abs(mesure[4] - j5) <= TOLERANCE_TETE:
            proche = True
            break

    if not proche:
        return (False, mesure)

    # -----------------------------------------------------
    # Fin reelle du mouvement
    # -----------------------------------------------------

    """
    Passer sous la tolerance ne veut pas dire etre arrive :
    le bras converge encore. On rend la main seulement quand
    les angles ont cesse de bouger, sinon la pose relevee ne
    correspond deja plus a la position du spot.
    """

    stable = attendreStabilisation(mc)

    return (True, stable if stable is not None else mesure)


def attendreStabilisation(mc, timeout=TIMEOUT_STABILITE):
    """Attend que la tete ait reellement cesse de bouger et retourne la pose stabilisee."""

    debut = time.time()

    precedente = None
    consecutives = 0

    while time.time() - debut < timeout:

        time.sleep(PERIODE_SCRUTATION)

        mesure = lireAngles(mc)

        if mesure is None:
            continue

        # Le firmware sait parfois repondre directement.
        try:
            en_mouvement = mc.is_moving()

        except Exception:
            en_mouvement = 0

        if en_mouvement == 1:
            consecutives = 0
            precedente = mesure
            continue

        if precedente is not None:

            bouge = max(abs(mesure[i] - precedente[i]) for i in (3, 4))

            if bouge <= SEUIL_STABILITE:
                consecutives += 1

                if consecutives >= LECTURES_STABLES:
                    return mesure

            else:
                consecutives = 0

        precedente = mesure

    return precedente


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


def viserAngles(mc, base, j4, j5, j6, tolerance=TOLERANCE_POINTAGE, passes=CORRECTIONS_MAX):
    """Amene la tete aux angles demandes et rattrape l'ecart de repos, puis retourne (vise, pose atteinte)."""

    """
    approcher() envoie une consigne et accepte la pose ou le
    bras s'immobilise, meme si elle est a deux degres. Ici on
    vise l'angle, pas la consigne : tant que la pose relevee
    n'est pas sur l'angle demande, on rallonge la consigne de
    ce qui manque et on y retourne.

    Chaque passe repasse par approcher(), donc par le recul :
    l'arrivee se fait toujours du meme cote, y compris quand
    la correction fait redescendre la consigne. Sans cela le
    rattrapage attaquerait le jeu des reducteurs par l'autre
    face et rendrait la pose non reproductible -- ce que toute
    la calibration cherche justement a eviter.

    La consigne corrigee peut sortir des limites constructeur
    au bord de la course : on la borne, et la fonction rend
    alors False sans insister.
    """

    consigne_j4, consigne_j5 = j4, j5

    arrive, pose = approcher(mc, base, consigne_j4, consigne_j5, j6)

    for _ in range(passes):

        if pose is None:
            return (False, None)

        ecart_j4 = j4 - pose[3]
        ecart_j5 = j5 - pose[4]

        if max(abs(ecart_j4), abs(ecart_j5)) <= tolerance:
            return (True, pose)

        corrigee_j4 = min(max(consigne_j4 + ecart_j4, J4_MIN), J4_MAX)
        corrigee_j5 = min(max(consigne_j5 + ecart_j5, J5_MIN), J5_MAX)

        # Consigne deja en butee : insister ne changerait plus rien.
        if (corrigee_j4, corrigee_j5) == (consigne_j4, consigne_j5):
            return (False, pose)

        consigne_j4, consigne_j5 = corrigee_j4, corrigee_j5

        arrive, pose = approcher(mc, base, consigne_j4, consigne_j5, j6)

    if pose is None:
        return (False, None)

    reste = max(abs(j4 - pose[3]), abs(j5 - pose[4]))

    return (reste <= tolerance, pose)


def bougerTeteDuMemeCote(mc, base, courant_j4, courant_j5, cible_j4, cible_j5, j6):
    """Deplace la tete en arrivant toujours du meme cote, et retourne (arrive, pose mesuree)."""

    """
    Un pas qui va vers les valeurs croissantes arrive deja du
    bon cote : on l'envoie directement. Un pas qui redescend
    depasse la cible puis remonte dessus.

    C'est ce qui permet de relever la pose telle quelle, sans
    reprise d'approche apres coup : la position pointee EST
    celle que le bras retrouvera au tir.
    """

    pre_j4 = cible_j4 - DEPASSEMENT_JOG if cible_j4 < courant_j4 else cible_j4
    pre_j5 = cible_j5 - DEPASSEMENT_JOG if cible_j5 < courant_j5 else cible_j5

    if (pre_j4, pre_j5) != (cible_j4, cible_j5):

        pre_j4 = max(J4_MIN, pre_j4)
        pre_j5 = max(J5_MIN, pre_j5)

        bougerTete(mc, base, pre_j4, pre_j5, j6)

    return bougerTete(mc, base, cible_j4, cible_j5, j6)


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

    if angles is None:
        print()
        print("PU_PositionInitiale est introuvable : la pose de reference est inconnue.")
        print("Le bras NE sera PAS deplace. La seance part de la pose courante,")
        print("qui sera relue telle quelle et enregistree dans le modele.")
        return (False, lireAngles(mc))

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


NOMS_SEQUENCES = {
    "[A": "haut",   "OA": "haut",
    "[B": "bas",    "OB": "bas",
    "[C": "droite", "OC": "droite",
    "[D": "gauche", "OD": "gauche",
}


def nommerTouche(octets):
    """Traduit les octets recus du terminal en nom de touche."""

    """
    Une fleche n'envoie pas un caractere mais une sequence
    d'echappement de trois octets. Elle prend deux formes
    selon le mode du terminal : ESC [ A en mode normal,
    ESC O A en mode application. Les deux circulent, il faut
    accepter les deux.
    """

    if not octets:
        return "inconnu"

    texte = octets.decode("utf-8", "replace")

    if texte[0] == "\x1b":

        if len(texte) == 1:
            return "echap"

        return NOMS_SEQUENCES.get(texte[1:3], "inconnu:" + repr(texte[1:]))

    if texte[0] in ("\r", "\n"):
        return "entree"

    return texte[0].lower()


def ouvrirClavier():
    """Passe le terminal en mode caractere et retourne son reglage precedent, ou None."""

    """
    Le mode est pris UNE fois pour toute la duree du
    pointage, pas a chaque touche.

    tty.setcbreak vide le tampon d'entree par defaut
    (TCSAFLUSH) : appele avant chaque lecture, il jetait
    toutes les touches frappees pendant que le bras bougeait,
    et il devenait impossible d'enchainer les deplacements.
    TCSANOW applique le reglage sans rien jeter, et les appuis
    faits pendant un mouvement sont traites ensuite.
    """

    if not CLAVIER_DIRECT or not sys.stdin.isatty():
        return None

    descripteur = sys.stdin.fileno()

    try:
        ancien_mode = termios.tcgetattr(descripteur)
        tty.setcbreak(descripteur, termios.TCSANOW)

    except Exception as e:
        print("Mode caractere indisponible, les commandes seront a taper :", e)
        return None

    return ancien_mode


def fermerClavier(ancien_mode):
    """Rend au terminal son reglage d'origine."""

    if ancien_mode is None:
        return

    try:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, ancien_mode)

    except Exception:
        pass


NOMS_SEQUENCES = {
    "[A": "haut",   "OA": "haut",
    "[B": "bas",    "OB": "bas",
    "[C": "droite", "OC": "droite",
    "[D": "gauche", "OD": "gauche",
}


def nommerTouche(octets):
    """Traduit les octets recus du terminal en nom de touche."""

    """
    Une fleche n'envoie pas un caractere mais une sequence
    d'echappement de trois octets, et elle prend deux formes
    selon le mode du terminal : ESC [ A en mode normal,
    ESC O A en mode application. Les deux circulent, il faut
    accepter les deux.
    """

    if not octets:
        return "inconnu"

    texte = octets.decode("utf-8", "replace")

    if texte[0] == "\x1b":

        if len(texte) == 1:
            return "echap"

        return NOMS_SEQUENCES.get(texte[1:3], "inconnu:" + repr(texte[1:]))

    if texte[0] in ("\r", "\n"):
        return "entree"

    return texte[0].lower()


def lireTouche(clavier_actif=None):
    """Lit une touche au vol, fleches comprises, et retourne son nom."""

    """
    La lecture se fait par os.read et NON par sys.stdin.read :
    sys.stdin est bufferise, il avale les trois octets d'une
    fleche d'un coup et n'en rend qu'un. Le select qui suit ne
    voyait alors plus rien sur le descripteur, et la fleche
    passait pour un echappement seul.
    """

    if clavier_actif is None:
        return lireTexte("  > ").lower()

    descripteur = sys.stdin.fileno()

    octets = os.read(descripteur, 1)

    if octets == b"\x1b":

        # Le reste de la sequence suit immediatement, ou bien
        # c'est la touche echap toute seule.
        if select.select([descripteur], [], [], 0.05)[0]:
            octets += os.read(descripteur, 2)

    return nommerTouche(octets)


def lireImpact():
    """Demande les coordonnees du noeud vise en carreaux et les retourne, ou None."""

    print("  Coordonnees du noeud, en carreaux (ex : 3 -2).")
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
            return (float(morceaux[0]), float(morceaux[1]))

        except ValueError:
            print("  Saisie illisible.")


def afficherAideClavier(pas):
    """Affiche les commandes du pointage au clavier."""

    print("  fleches gauche/droite : J4 -%.2f / +%.2f deg" % (pas, pas))
    print("  fleches haut/bas      : J5 +%.2f / -%.2f deg" % (pas, pas))
    print("  + / -  : pas plus grand / plus petit      m : bouger la tete a la main")
    print("  entree : valider ce point      s : sauter      f : terminer")


def pointageMainLibre(mc, base, j4, j5, j6):
    """Relache la tete pour un placement grossier a la main, et retourne la pose apres reprise."""

    """
    Traverser 75 cm au clavier prendrait des dizaines
    d'appuis. La main sert a degrossir ; le clavier, a poser
    le spot exactement sur le noeud.
    """

    print("  TENIR LA TETE : elle va devenir molle.")

    if not lireTexte("  Prete a etre relachee ? [O/n] ", "o").lower().startswith("o"):
        return (j4, j5)

    if not relacherTete(mc):
        return (j4, j5)

    print("  Tete libre. La placer approximativement, puis entree.")
    lireTexte("  > ")

    pose = lireAngles(mc)
    verrouillerTete(mc, pose)

    if pose is None:
        return (j4, j5)

    # -----------------------------------------------------
    # Reprise par les moteurs
    # -----------------------------------------------------

    """
    La pose obtenue a la main n'est pas celle ou le bras
    revient tout seul : l'encodeur est cote moteur, il ne voit
    pas le jeu du reducteur. On refait donc l'arrivee par les
    moteurs tout de suite. Le spot bouge un peu ici, et c'est
    normal : l'affinage au clavier suit.
    """

    print("  Reprise par les moteurs, le spot va se decaler un peu...")

    arrive, mesure = approcher(mc, base, pose[3], pose[4], j6)

    if mesure is None:
        return (j4, j5)

    return (mesure[3], mesure[4])


def pointageAuClavier(mc, base, j4, j5, j6):
    """Deplace la tete au clavier jusqu'au noeud vise, et retourne (issue, pose mesuree)."""

    """
    Tout le deplacement passe par les moteurs, et chaque pas
    arrive du meme cote. La pose relevee a la validation est
    donc exactement celle que le bras retrouvera au tir : il
    n'y a plus rien a corriger apres coup.
    """

    pas = PAS_JOG_DEFAUT

    afficherAideClavier(pas)

    clavier = ouvrirClavier()

    try:
        return boucleClavier(mc, base, j4, j5, j6, pas, clavier)

    finally:
        fermerClavier(clavier)


def boucleClavier(mc, base, j4, j5, j6, pas, clavier):
    """Boucle de deplacement au clavier, terminal deja en mode caractere."""

    while True:

        print("  J4 = %+7.2f   J5 = %+7.2f   (pas %.2f)" % (j4, j5, pas))

        touche = lireTouche(clavier)

        # -------------------------------------------------
        # Sortie
        # -------------------------------------------------

        if touche in ("entree", "o", "ok", ""):

            pose = lireAngles(mc)

            if pose is None:
                print("  Lecture des angles impossible.")
                continue

            return ("ok", pose)

        if touche in ("s", "saut"):
            return ("saut", None)

        if touche in ("f", "fin"):
            return ("fin", None)

        if touche in ("?", "h", "aide"):
            afficherAideClavier(pas)
            continue

        # -------------------------------------------------
        # Reglages
        # -------------------------------------------------

        if touche == "+":
            pas = min(PAS_JOG_MAX, pas * 2.0)
            continue

        if touche == "-":
            pas = max(PAS_JOG_MIN, pas / 2.0)
            continue

        if touche == "m":

            # Le mode main pose des questions : il faut rendre
            # le terminal a son fonctionnement normal le temps
            # des saisies, puis le reprendre.
            fermerClavier(clavier)
            j4, j5 = pointageMainLibre(mc, base, j4, j5, j6)
            clavier = ouvrirClavier()

            afficherAideClavier(pas)
            continue

        # -------------------------------------------------
        # Deplacement
        # -------------------------------------------------

        cible_j4 = j4
        cible_j5 = j5

        if touche == "gauche":
            cible_j4 = j4 - pas
        elif touche == "droite":
            cible_j4 = j4 + pas
        elif touche == "haut":
            cible_j5 = j5 + pas
        elif touche == "bas":
            cible_j5 = j5 - pas
        else:
            """
            On affiche ce que le terminal a envoye : une
            fleche non reconnue se diagnostique tout de
            suite, au lieu de passer pour une touche morte.
            """
            print("  Touche non reconnue :", touche, "-- taper ? pour l'aide.")
            continue

        arrive, mesure = bougerTeteDuMemeCote(mc, base, j4, j5, cible_j4, cible_j5, j6)

        if mesure is not None:
            # On repart des angles REELS : sinon l'ecart avec
            # la consigne s'accumule sans que rien ne le dise.
            j4 = mesure[3]
            j5 = mesure[4]

        else:
            print("  ATTENTION : la tete n'a pas suivi la consigne.")


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


def pointsAberrants(modele, mesures, facteur=3.0):
    """Retourne les points dont l'ecart depasse le facteur donne fois l'ecart moyen."""

    """
    Un point tres au-dessus des autres n'est pas du bruit :
    c'est presque toujours une coordonnee mal saisie, ou un
    noeud compte de travers. Le retirer vaut mieux que de
    laisser le modele se deformer pour l'atteindre.
    """

    residus = calculerResidus(modele, mesures)

    if residus is None or residus["rms_carreaux"] <= 0.0:
        return []

    seuil = facteur * residus["rms_carreaux"]

    aberrants = []

    for indice, ecart in enumerate(residus["par_point_carreaux"]):
        if ecart > seuil:
            aberrants.append((indice, ecart, mesures[indice]))

    return aberrants


def diagnostiquer(modele, ajustement, validation):
    """Affiche ce que les residus disent de l'origine de l'erreur."""

    """
    Le but est d'eviter la fausse manoeuvre la plus courante :
    ajouter des points quand ce n'est pas le nombre de points
    qui limite. Le modele a neuf parametres ; vingt points en
    donnent deja quarante equations. Si l'ajustement colle
    mal, en ajouter ne changera rien.
    """

    print()
    print("-----------------------------------------")
    print("DIAGNOSTIC")
    print("-----------------------------------------")

    residus = modele.get("residus")

    if residus is None:
        print("Pas de residus exploitables.")
        return

    ajuste = residus["rms_carreaux"]
    croise = validation and modele.get("residus_validation")
    croise = croise["rms_carreaux"] if croise else None

    # -----------------------------------------------------
    # Points aberrants
    # -----------------------------------------------------

    aberrants = pointsAberrants(modele, ajustement)

    if aberrants:
        print()
        print("%d point(s) tres au-dessus des autres :" % len(aberrants))

        for indice, ecart, point in aberrants:
            print("  point %d : carreau (%.1f, %.1f), ecart %.2f carreaux"
                  % (indice + 1, point["u"], point["v"], ecart))

        print("Verifier ces coordonnees dans le fichier : une saisie de travers")
        print("deforme tout le modele. Les corriger vaut mieux qu'en ajouter.")

    # -----------------------------------------------------
    # Zone couverte
    # -----------------------------------------------------

    """
    Le modele refuse de viser hors de l'enveloppe relevee.
    Afficher cette enveloppe evite la surprise classique :
    des points refuses parce que la calibration ne couvrait
    pas les bords.
    """

    e = modele["enveloppe"]

    largeur = e["u_max"] - e["u_min"]
    hauteur = e["v_max"] - e["v_min"]

    print()
    print("Zone calibree : u de %.0f a %.0f, v de %.0f a %.0f carreaux (%.0f x %.0f)."
          % (e["u_min"], e["u_max"], e["v_min"], e["v_max"], largeur, hauteur))
    print("Viser hors de cette zone sera refuse.")

    # -----------------------------------------------------
    # Interpretation
    # -----------------------------------------------------

    print()

    if ajuste > RESIDU_MAUVAIS:
        print("L'ajustement colle MAL aux points releves (%.2f carreaux)." % ajuste)
        print()
        print("Ajouter des points n'y changera rien : le modele a 9 parametres,")
        print("20 points en donnent deja 40 equations. Ce qui limite est ailleurs.")
        print("Par ordre de probabilite :")
        print("  1. des coordonnees saisies de travers (voir ci-dessus) ;")
        print("  2. la base a bouge pendant la seance -- le script le signale,")
        print("     mais un choc entre deux points passe inapercu ;")
        print("  3. le laser est trop decale de l'axe du poignet pour que")
        print("     l'hypothese du point d'emission fixe tienne sur cette zone.")
        print("     Dans ce cas seul un modele plus riche aiderait.")

    elif croise is not None and croise > 2.0 * max(ajuste, 0.05):
        print("L'ajustement colle bien (%.2f) mais la validation decroche (%.2f)."
              % (ajuste, croise))
        print()
        print("Le modele decrit les points qui l'ont produit et pas la zone.")
        print("C'est le seul cas ou AJOUTER DES POINTS aide vraiment,")
        print("surtout vers les bords et les coins.")

    elif ajuste > RESIDU_BON:
        print("Le modele colle correctement (%.2f carreaux, soit %.1f cm)."
              % (ajuste, ajuste * COTE_CARREAU))

        if croise is not None:
            print("Validation : %.2f carreaux." % croise)

        print()
        print("C'est utilisable, mais il reste de la marge. Les deux causes")
        print("les plus probables a ce niveau sont la derive de la base")
        print("pendant la seance, et la finesse du pointage au bord de la")
        print("zone, ou un pas de 1 degre deplace le spot bien plus qu'au")
        print("centre. Utiliser la touche - pour affiner les points du bord.")

    else:
        print("Le modele colle bien aux mesures (%.2f carreaux, soit %.1f cm)."
              % (ajuste, ajuste * COTE_CARREAU))

        if croise is not None:
            print("La validation confirme (%.2f carreaux)." % croise)

        print()
        print("A ce niveau, c'est la mesure qui limite et non l'ajustement :")
        print("le spot est pose a vue et relu sur le quadrillage. Il n'y a")
        print("plus rien a gagner sur le modele.")
        print()
        print("Si le spot tombe quand meme a cote au moment de viser, ce n'est")
        print("pas le modele : c'est la REPRODUCTIBILITE du bras. Le verifier")
        print("avec calibration_test.py, en visant plusieurs fois le meme point")
        print("depuis des poses differentes et en relevant l'ecart a chaque fois.")

    # -----------------------------------------------------
    # Biais
    # -----------------------------------------------------

    """
    Un decalage constant n'est pas du bruit : il se rattrape
    d'un seul coup, alors qu'une dispersion ne se rattrape
    pas. Le distinguer evite de chercher a ameliorer une
    calibration qui n'a besoin que d'un recalage.
    """

    reference = modele.get("residus_validation") or residus

    biais = math.hypot(reference["biais_u_carreaux"], reference["biais_v_carreaux"])

    if biais > 0.5 * max(reference["dispersion_carreaux"], 1e-6):
        print()
        print("Le modele est DECALE de facon constante : %+.2f en u, %+.2f en v"
              % (reference["biais_u_carreaux"], reference["biais_v_carreaux"]))
        print("(%.2f carreaux au total, pour %.2f de dispersion)."
              % (biais, reference["dispersion_carreaux"]))
        print("Un decalage constant se rattrape ; une dispersion non. Verifier")
        print("que le noeud d'origine est bien le meme qu'a la calibration, et")
        print("que la base n'a pas bouge depuis.")

    print()
    print("Rappel : 1 carreau = %.0f cm." % COTE_CARREAU)


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
            if residus_validation["rms_carreaux"] > 3.0 * modele["residus"]["rms_carreaux"]:
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
        print("sur les memes points : rms %.2f carreaux, max %.2f carreaux" % (temoin["rms_carreaux"], temoin["max_carreaux"]))
        modele["temoin_bilineaire"] = temoin

    # -----------------------------------------------------
    # Enregistrement
    # -----------------------------------------------------

    diagnostiquer(modele, ajustement, validation)

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
    print("  1. les fleches amenent le spot sur un noeud du quadrillage ;")
    print("  2. [Entree] releve les angles et enregistre le point.")
    print()
    print("Le spot ne bouge plus entre le pointage et le releve : chaque")
    print("pas arrive du meme cote que l'approche du tir, la position")
    print("pointee est donc celle que le bras retrouvera.")
    print()
    print("La touche m relache la tete pour la placer a la main quand il")
    print("faut traverser la zone -- LA TENIR avant de valider. Seuls les")
    print("servos 4 et 5 sont relaches, le bras ne s'effondre pas.")
    print()
    print("Repartition des points :")
    print("  - une grille reguliere, environ 5 x 4, usage 'ajustement' ;")
    print("  - 4 ou 5 points hors de cette grille, usage 'validation'.")
    print()
    print("Les points d'ajustement doivent inclure les QUATRE COINS de la")
    print("zone utile. Le modele refuse d'extrapoler : ne pointer que le")
    print("centre reviendrait a ne calibrer que le centre.")
    print()
    print("Tout se compte en CARREAUX (un carreau = %.0f cm)." % COTE_CARREAU)
    print("Le noeud (0, 0) est celui que vous choisissez comme origine :")
    print("gardez le meme pendant toute la seance.")
    print()


def seanceDeReleve(mc, base, j6, serie, usage):
    """Deroule le pointage des points un par un et retourne le nombre de points enregistres."""

    depart = lireAngles(mc)

    j4 = depart[3] if depart else 0.0
    j5 = depart[4] if depart else 0.0

    index = 0
    enregistres = 0

    while True:

        index += 1

        print()
        print("-----------------------------------------")
        print("Point", index, "  (serie '%s', usage '%s')" % (serie, usage))
        print("Amener le spot sur un noeud du quadrillage.")

        issue, pose = pointageAuClavier(mc, base, j4, j5, j6)

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
        J1 a J3 ne sont jamais commandes, mais le mode main
        les deplace si on appuie un peu fort. Si la base bouge
        en cours de seance, les points releves avant et apres
        ne decrivent plus la meme geometrie, et rien dans les
        residus ne le signalerait.
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
        # Releve de l'impact
        # -------------------------------------------------

        """
        Le spot est reste exactement ou il a ete pose : la
        derniere position atteinte l'a ete par les moteurs, en
        arrivant du bon cote. Rien ne bouge entre le pointage
        et le releve.
        """

        impact = lireImpact()

        if impact is None:
            index -= 1
            continue

        if enregistrerPoint(serie, usage, index, (j4, j5), pose, impact):
            enregistres += 1
            print("  Point %d enregistre : J4 = %+.2f  J5 = %+.2f  ->  carreau (%.1f, %.1f)"
                  % (enregistres, j4, j5, impact[0], impact[1]))

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

    if not arrive and ANGLES_POSITION_INITIALE is not None:
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
