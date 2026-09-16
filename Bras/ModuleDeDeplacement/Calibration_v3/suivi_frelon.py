"""
=========================================================
SUIVI DE TRAJECTOIRE RAPIDE - SIMULATION DE VOL DE FRELON
=========================================================

Objectif :
----------

Enchainer une liste de positions le plus vite possible,
pour simuler le suivi d'un frelon en vol par le pointeur.

Ce script NE MODIFIE RIEN. Il ne fait qu'IMPORTER la
chaine de visee deja calibree :

    position (carreaux)
      -> correction apprise (modele_correction_Calib_2)
      -> homographie Calib 2
      -> angles J4 / J5

Ce qui change, c'est uniquement la FACON de commander le
bras une fois les angles connus.


Pourquoi c'est plus rapide que la campagne de calibration
---------------------------------------------------------

viserAngles() est concu pour la PRECISION, pas pour la
vitesse. Par point il fait :

    - un recul de 8 degres, pour arriver toujours du meme
      cote et reprendre le jeu des reducteurs par la meme
      face ;
    - jusqu'a 3 passes de rattrapage a 0,2 degre ;
    - une attente de stabilisation (3 lectures immobiles).

Cela coute plusieurs secondes par point. Un frelon
n'attend pas. Ici on saute les trois :

    1. une seule consigne par point, a vitesse 100 ;
    2. scrutation a 30 ms au lieu de 200 ms ;
    3. on repart des que l'ecart passe sous la tolerance,
       SANS attendre l'arret : la consigne suivante prend
       le relais en plein mouvement, ce qui enchaine les
       segments au lieu de les decomposer en une suite de
       departs et d'arrets.

CONTREPARTIE ASSUMEE :
    la precision baisse. Sans le recul, le jeu n'est plus
    repris du meme cote ; sans stabilisation, la pose est
    lue en vol. Le script MESURE cette degradation au lieu
    de la cacher (colonnes ecart_J4 / ecart_J5 et
    erreur_carreaux du CSV).

    C'est le compromis normal d'un suivi. On ne peut pas
    etre a 0,2 degre ET a 10 Hz avec ce bras : la tolerance
    d'enchainement est donc un REGLAGE, demande au
    lancement.


Ce qui n'est PAS l'optimisation
-------------------------------

Precalculer toute la trajectoire avant de bouger ne sert a
rien : anglesPourPoint() coute 3,9 microsecondes par appel
(mesure sur 10 000 appels). C'est negligeable devant les
~30 ms d'un aller-retour serie.

La conversion se fait donc DANS la boucle. C'est gratuit,
et c'est ce qui permet au meme code de consommer plus tard
un flux camera temps reel, ou les positions ne sont pas
connues d'avance.

Le goulot d'etranglement est le lien serie et la mecanique
du bras, pas le calcul.


Architecture
------------

Trois etages decouples :

    SOURCE de positions -> CONVERSION -> ENVOI + RELEVE

La SOURCE est un simple iterable de couples (x, y) en
carreaux. Brancher la camera consistera a ecrire un
generateur de plus, sans toucher au reste.


Deux modes d'execution
----------------------

MODE 1 - ENCHAINE (waypoints)
    On attend que chaque point soit atteint a la tolerance
    pres, puis on envoie le suivant. Le bras passe par
    tous les points.
    -> mesure la vitesse point a point du bras.

MODE 2 - FLUX (poursuite temps reel)
    On envoie une consigne a cadence fixe, sans jamais
    attendre l'arrivee. Le bras court derriere la cible et
    ne la rattrape pas forcement.
    -> c'est le comportement reel d'un suivi de frelon, et
       ca mesure le RETARD de la tete sur la cible. C'est
       ce mode que la camera utilisera.


Sorties
-------

    resultats/suivi_<trajectoire>_<mode>_<horodatage>.csv
    resultats/suivi_<trajectoire>_<mode>_<horodatage>.png

Toutes les positions sont en CARREAUX.


Dependances
-----------

Uniquement la bibliotheque standard, plus calibration_grille
qui est lui-meme en stdlib pur. matplotlib est OPTIONNEL :
sans lui le script tourne et ecrit son CSV, seule la figure
est sautee. La Jetson n'a pas numpy garanti, le suivi ne
doit pas en dependre.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import json
import math
import random
import time

from datetime import datetime


"""
matplotlib est un confort, pas une dependance.

Sur la Jetson, ni numpy ni matplotlib ne sont garantis. Un
suivi qui refuse de demarrer parce qu'il ne pourra pas
tracer un graphique serait absurde : la donnee qui compte
est le CSV.
"""
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    TRACE_DISPONIBLE = True

except Exception:
    TRACE_DISPONIBLE = False


"""
Un seul module importe, et il est en stdlib pur : pas de
pymycobot a l'import (il n'est charge que dans
connecterRobot), pas de matplotlib, pas de numpy.

On n'importe DELIBEREMENT pas calib2_corrigee_merlin : il
tire matplotlib et PU_PositionInitiale, donc pymycobot, des
l'import. Les quelques fonctions de Calib 2 dont on a
besoin sont donc recopiees plus bas, a l'identique. Elles
tiennent en 50 lignes, et ce script reste utilisable sur une
machine sans robot.
"""
from calibration_grille import (
    DOSSIER_RESULTATS,
    FICHIER_MESURES,
    J4_MIN,
    J4_MAX,
    J5_MIN,
    J5_MAX,
    TOLERANCE_BASE,
    ajusterModele,
    allerPositionInitiale,
    allumerPointeur,
    anglesPourPoint,
    calculerResidus,
    chargerMesures,
    connecterRobot,
    dansEnveloppe,
    eteindrePointeur,
    lireAngles,
    predireImpact,
)


# =========================================================
# 2. PARAMETRES
# =========================================================

SERIE = "Calib 2"


FICHIER_MODELE_CORRECTION = os.path.join(
    DOSSIER_RESULTATS,
    "modele_correction_Calib_2.json",
)


"""
Vitesse envoyee au firmware. L'echelle pymycobot est un
entier de 1 a 100. On ne descend pas en dessous de 100 :
tout l'interet du script est la.
"""
VITESSE_SUIVI = 100


"""
Periode de scrutation des angles.

La calibration scrute a 200 ms : confortable quand on
attend l'immobilite, absurde quand on veut savoir le plus
tot possible qu'on est arrive. A 30 ms on paie une lecture
serie de plus, mais on perd au pire 30 ms par point au lieu
de 200.

Ne pas descendre beaucoup plus bas : chaque lecture est un
aller-retour serie, et saturer le lien ralentit le bras
lui-meme.
"""
PERIODE_LECTURE = 0.03     # secondes


"""
Ecart a partir duquel le point est considere comme atteint
et le suivant est envoye (mode ENCHAINE).

C'est LE parametre du compromis vitesse / precision :

    0,2 deg  -> precision de la calibration, tres lent
    2,0 deg  -> defaut, environ 2 cm au sol
    5,0 deg  -> tres rapide, le bras ne fait que froler
                les points

Le bras n'est pas arrete quand on lache la main : la
consigne suivante prend le relais en plein mouvement.
"""
TOLERANCE_ENCHAINEMENT = 2.0   # degres


"""
Au dela de ce temps, le point est declare manque et on
passe au suivant sans bloquer la trajectoire. Un suivi qui
attend indefiniment un point n'est plus un suivi.
"""
TIMEOUT_POINT = 3.0            # secondes

"""
Le premier point part de la position initiale : le trajet
est long, il a droit a plus de temps.
"""
TIMEOUT_PREMIER_POINT = 8.0    # secondes


"""
Cadence d'envoi en mode FLUX. 10 Hz est un ordre de
grandeur realiste pour une detection camera.
"""
FREQUENCE_FLUX = 10.0      # Hz


"""
SECURITE.

En suivi, le pointeur reste allume pendant les mouvements :
c'est precisement ce qu'on veut montrer. C'est l'inverse du
protocole de calibration, ou le laser ne s'allume qu'a
l'arret pour permettre la mesure du spot.

A ne faire qu'avec le POINTEUR ROUGE CLASSE 2, laser de
puissance DEBRANCHE.
"""
LASER_PENDANT_SUIVI = True


"""
Marge de securite ajoutee au bord de l'enveloppe pour
generer les trajectoires : on genere franchement dedans,
sans profiter de MARGE_ENVELOPPE. Cette marge-la sert a
tolerer un point demande au bord, pas a fabriquer
volontairement des points extrapoles.
"""
MARGE_GENERATION = 0.5     # carreaux


ENTETE_SUIVI = [
    "numero",
    "t_relatif_s",
    "x_demande",
    "y_demande",
    "x_corrige",
    "y_corrige",
    "J4_consigne",
    "J5_consigne",
    "J4_atteint",
    "J5_atteint",
    "ecart_J4",
    "ecart_J5",
    "x_atteint",
    "y_atteint",
    "erreur_carreaux",
    "duree_s",
    "amplitude_deg",
    "vitesse_deg_s",
    "atteint",
]


# =========================================================
# 3. CALIB 2 - RECOPIE DES FONCTIONS NECESSAIRES
# =========================================================

"""
Les quatre fonctions qui suivent sont reprises telles
quelles de calib2_corrigee_merlin.py.

Elles sont recopiees et non importees pour une seule
raison : importer ce module tirerait matplotlib et
pymycobot des le chargement. Le comportement est
identique ; si la calibration change, c'est ici qu'il
faudra reporter.
"""


def chargerModeleCorrection():
    """Charge le modele polynomial de correction de Calib 2, ou None s'il est absent."""

    print()
    print("Modele de correction :", FICHIER_MODELE_CORRECTION)

    if not os.path.exists(FICHIER_MODELE_CORRECTION):
        print("  ABSENT : le suivi tournera SANS correction apprise.")
        return None

    try:
        with open(FICHIER_MODELE_CORRECTION, "r") as fichier:
            modele = json.load(fichier)

    except Exception as e:
        print("  ERREUR de lecture :", e)
        return None

    serie_modele = (modele.get("serie") or "").strip()

    if serie_modele != SERIE:
        print("  REFUS : modele de la serie '%s', attendu '%s'." % (serie_modele, SERIE))
        return None

    print("  Charge (degre %s)." % modele.get("degre"))

    return modele


def chargerBaseSerie():
    """Retourne J1/J2/J3 releves pendant Calib 2, ou None."""

    try:
        with open(FICHIER_MESURES, "r", newline="") as fichier:

            for ligne in csv.DictReader(fichier):

                if (ligne.get("serie") or "").strip() != SERIE:
                    continue

                try:
                    return [
                        float(ligne["J1"]),
                        float(ligne["J2"]),
                        float(ligne["J3"]),
                    ]

                except (ValueError, KeyError):
                    continue

    except Exception as e:
        print("Impossible de lire la base de la calibration :", e)

    return None


def construireModeleCalib2():
    """Reconstruit le modele geometrique de Calib 2 depuis les mesures, ou None."""

    ajustement = chargerMesures(usage="ajustement", serie=SERIE)
    validation = chargerMesures(usage="validation", serie=SERIE)

    if len(ajustement) < 5:
        print()
        print("ERREUR : Calib 2 ne contient que %d point(s) d'ajustement, il en faut 5."
              % len(ajustement))
        return None

    modele = ajusterModele(ajustement, base=chargerBaseSerie())

    if modele is None:
        print("ERREUR : impossible de reconstruire Calib 2.")
        return None

    modele["serie"] = SERIE

    if validation:
        modele["residus_validation"] = calculerResidus(modele, validation)

    return modele


def predireCorrection(modele, x, y):
    """Retourne l'erreur (dx, dy) predite par le modele appris pour la cible (x, y)."""

    if modele is None:
        return (0.0, 0.0)

    centre_x = float(modele["centre_x"])
    centre_y = float(modele["centre_y"])

    echelle_x = float(modele["echelle_x"])
    echelle_y = float(modele["echelle_y"])

    if abs(echelle_x) < 1e-12:
        echelle_x = 1.0

    if abs(echelle_y) < 1e-12:
        echelle_y = 1.0

    x_normalise = (x - centre_x) / echelle_x
    y_normalise = (y - centre_y) / echelle_y

    valeurs = [
        (x_normalise ** int(terme[0])) * (y_normalise ** int(terme[1]))
        for terme in modele["termes"]
    ]

    erreur_x = sum(
        float(coefficient) * valeur
        for coefficient, valeur in zip(modele["coefficients_erreur_x"], valeurs)
    )

    erreur_y = sum(
        float(coefficient) * valeur
        for coefficient, valeur in zip(modele["coefficients_erreur_y"], valeurs)
    )

    return (float(erreur_x), float(erreur_y))


def verifierBase(mc, modele, tolerance=TOLERANCE_BASE):
    """Refuse de tirer si J1/J2/J3 ont bouge depuis la calibration."""

    """
    Le modele decrit la geometrie du bras DANS UNE POSE DE
    BASE donnee. Si la base a bouge, il ne decrit plus
    rien, et le faisceau part ailleurs en ayant l'air de
    fonctionner.
    """

    reference = modele.get("base")

    if not reference:
        print("ATTENTION : pas de pose de base dans le modele, verification impossible.")
        return True

    mesure = lireAngles(mc)

    if mesure is None:
        print("Lecture des angles impossible.")
        return False

    ecarts = [abs(mesure[i] - reference[i]) for i in range(3)]

    print()
    print("Base de calibration :", [round(a, 2) for a in reference])
    print("Base courante       :", [round(mesure[i], 2) for i in range(3)])
    print("Ecarts              :", [round(e, 2) for e in ecarts])

    if max(ecarts) > tolerance:
        print()
        print("REFUS : la base a bouge de %.2f deg depuis la calibration." % max(ecarts))
        print("Le modele ne decrit plus la geometrie actuelle.")
        print("Remettre le bras dans sa pose de calibration, ou refaire une campagne.")
        return False

    print("Base conforme.")

    return True


# =========================================================
# 4. ENVELOPPE
# =========================================================

def bornesUtiles(modele, marge=MARGE_GENERATION):
    """Retourne (x_min, x_max, y_min, y_max) ou il est sur de generer des points."""

    e = modele["enveloppe"]

    return (
        e["u_min"] + marge,
        e["u_max"] - marge,
        e["v_min"] + marge,
        e["v_max"] - marge,
    )


def serrerDansEnveloppe(modele, x, y, marge=MARGE_GENERATION):
    """Ramene un point dans l'enveloppe au lieu de le rejeter."""

    """
    Rejeter un point d'une trajectoire y ferait un trou :
    le bras sauterait par-dessus et la mesure de suivi ne
    voudrait plus rien dire. On rabat sur le bord, la
    trajectoire reste continue.
    """

    x_min, x_max, y_min, y_max = bornesUtiles(modele, marge)

    return (
        min(max(x, x_min), x_max),
        min(max(y, y_min), y_max),
    )


# =========================================================
# 5. SOURCES DE POSITIONS
# =========================================================

"""
Une source est un ITERABLE de couples (x, y) en carreaux.

Rien d'autre. C'est ce qui permet a une liste finie et a un
flux camera infini d'etre consommes par exactement le meme
code : la boucle itere, elle n'indexe pas.
"""


def genererVolFrelon(modele, nombre, pas_max=2.0, inertie=0.80, graine=None):
    """Genere une trajectoire continue qui ressemble a un vol d'insecte."""

    """
    Un frelon ne se teleporte pas : sa position a l'instant
    n+1 depend de celle a l'instant n. Tirer des points
    independants au hasard -- ce que fait la campagne de
    calibration, et c'est correct POUR ELLE -- donnerait une
    suite de grands sauts, donc une mesure de la vitesse de
    traversee, pas une mesure de suivi.

    On integre donc une vitesse bruitee :

        v(n+1) = inertie * v(n) + (1 - inertie) * bruit
        p(n+1) = p(n) + v(n+1)

    inertie proche de 1  -> vol lisse, planant
    inertie proche de 0  -> vol nerveux, saccade

    Aux bords, la vitesse est reflechie : le frelon rebondit
    dans la zone au lieu d'y rester colle.
    """

    """
    GAIN. Sans lui, pas_max serait un mensonge.

    Le filtre du dessus est un AR(1) : si on lui injecte un
    bruit d'amplitude a, sa sortie ne monte qu'a

        a * racine((1 - inertie) / (1 + inertie))

    soit un TIERS de a pour inertie = 0,80. Mesure a l'appui :
    en demandant 2,0 carreaux de pas, on obtenait 0,72.

    On compense donc l'attenuation du filtre. Le bridage
    plus bas garde le dernier mot : le pas ne DEPASSE pas
    pas_max, mais il l'approche vraiment.
    """
    gain = math.sqrt((1.0 + inertie) / (1.0 - inertie)) if inertie < 1.0 else 1.0

    if graine is not None:
        random.seed(graine)

    x_min, x_max, y_min, y_max = bornesUtiles(modele)

    x = (x_min + x_max) / 2.0
    y = (y_min + y_max) / 2.0

    vx = 0.0
    vy = 0.0

    for _ in range(nombre):

        secousse = (1.0 - inertie) * gain

        vx = inertie * vx + secousse * random.uniform(-pas_max, pas_max)
        vy = inertie * vy + secousse * random.uniform(-pas_max, pas_max)

        # Bride la vitesse pour garder des pas comparables.
        norme = math.hypot(vx, vy)

        if norme > pas_max:
            vx = vx * pas_max / norme
            vy = vy * pas_max / norme

        x = x + vx
        y = y + vy

        if x < x_min or x > x_max:
            vx = -vx
            x = min(max(x, x_min), x_max)

        if y < y_min or y > y_max:
            vy = -vy
            y = min(max(y, y_min), y_max)

        yield (x, y)


def genererFigure(modele, forme, nombre):
    """Genere une trajectoire geometrique, reproductible d'un essai a l'autre."""

    """
    Le vol de frelon est aleatoire, donc impossible a
    comparer d'un reglage a l'autre. Ces figures servent de
    banc de mesure : meme trajectoire, on change un
    parametre, on lit la difference.
    """

    x_min, x_max, y_min, y_max = bornesUtiles(modele)

    centre_x = (x_min + x_max) / 2.0
    centre_y = (y_min + y_max) / 2.0

    rayon_x = (x_max - x_min) / 2.0
    rayon_y = (y_max - y_min) / 2.0

    if forme == "cercle":

        for i in range(nombre):
            angle = 2.0 * math.pi * i / nombre
            yield (
                centre_x + rayon_x * math.cos(angle),
                centre_y + rayon_y * math.sin(angle),
            )

        return

    if forme == "carre":

        sommets = [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
        ]

        par_cote = max(1, nombre // 4)

        for i in range(4):

            depart = sommets[i]
            arrivee = sommets[(i + 1) % 4]

            for k in range(par_cote):
                t = k / float(par_cote)
                yield (
                    depart[0] + t * (arrivee[0] - depart[0]),
                    depart[1] + t * (arrivee[1] - depart[1]),
                )

        return

    if forme == "zigzag":

        """
        Le pire cas pour le bras : chaque point inverse le
        sens de J4. C'est la figure qui fait le plus
        ressortir le jeu des reducteurs, donc celle qui
        montre le mieux ce que coute l'abandon du recul.
        """

        for i in range(nombre):
            t = i / float(max(1, nombre - 1))
            yield (
                x_min if i % 2 == 0 else x_max,
                y_min + t * (y_max - y_min),
            )

        return

    print("Forme inconnue :", forme)


def chargerTrajectoireFichier(chemin):
    """Lit une trajectoire dans un CSV (colonnes x,y) ou un JSON (liste de paires)."""

    if not os.path.exists(chemin):
        print("Fichier introuvable :", chemin)
        return []

    positions = []

    # -----------------------------------------------------
    # JSON
    # -----------------------------------------------------

    if chemin.lower().endswith(".json"):

        try:
            with open(chemin, "r", encoding="utf-8") as f:
                contenu = json.load(f)

        except Exception as e:
            print("Lecture JSON impossible :", e)
            return []

        for element in contenu:

            try:
                if isinstance(element, dict):
                    positions.append((float(element["x"]), float(element["y"])))
                else:
                    positions.append((float(element[0]), float(element[1])))

            except Exception:
                continue

        return positions

    # -----------------------------------------------------
    # CSV avec entete x,y
    # -----------------------------------------------------

    try:
        with open(chemin, "r", encoding="utf-8", newline="") as f:

            lecteur = csv.DictReader(f)
            colonnes = lecteur.fieldnames or []

            if "x" in colonnes and "y" in colonnes:

                for ligne in lecteur:
                    try:
                        positions.append((float(ligne["x"]), float(ligne["y"])))
                    except (TypeError, ValueError):
                        continue

                return positions

    except Exception as e:
        print("Lecture CSV impossible :", e)
        return []

    # -----------------------------------------------------
    # CSV sans entete exploitable : deux premieres colonnes
    # -----------------------------------------------------

    try:
        with open(chemin, "r", encoding="utf-8", newline="") as f:

            for ligne in csv.reader(f):

                if len(ligne) < 2:
                    continue

                try:
                    positions.append((float(ligne[0]), float(ligne[1])))
                except ValueError:
                    continue

    except Exception as e:
        print("Lecture CSV impossible :", e)
        return []

    return positions


"""
-----------------------------------------------------------
A ECRIRE AU MOMENT DU BRANCHEMENT DE LA CAMERA
-----------------------------------------------------------

    def sourceCamera():
        while True:
            detection = ...          # lecture du flux
            x, y = ...               # conversion en CARREAUX
            yield (x, y)

Aucun autre etage du script ne change : le mode FLUX
consomme deja une source infinie.

ATTENTION, le maillon manquant n'est pas ici mais dans la
conversion :

  - la camera sort des PIXELS (camera/ReconnaissanceObjet/
    ModuleAI.py), puis des CENTIMETRES relatifs au coin
    rouge haut-gauche (camera/ModuleCam/Calibrage.py,
    CoordenneesAuRobo) ;
  - le bras travaille en CARREAUX.

    >>> UN CARREAU VAUT 3,0 cm. <<<

    Tranche par Thomas le 14/09/2026. C'est la valeur de
    calibration_grille.py et de "cote_carreau": 3.0 dans
    modele_grille.json, donc celle sur laquelle toute la
    calibration a ete faite.

    PU_Pointage.py declare COTE_CARREAU = 2.5 : cette
    valeur est FAUSSE. Ne pas s'en servir, et ne pas la
    recopier. Prendre 2,5 au lieu de 3,0 ferait pointer le
    laser 20 % a cote, et rien dans le code ne le dirait --
    c'est exactement le facteur d'echelle silencieux contre
    lequel l'en-tete de calibration_grille.py met en garde.

La conversion est donc :

    x_carreaux = x_cm_camera / 3.0

Reste a fixer l'origine et l'orientation du repere camera
par rapport au repere (u, v) du quadrillage. Aucun code ne
le fait aujourd'hui, et ca ne se devine pas : il faudra le
mesurer une fois, sur la zone reelle.
-----------------------------------------------------------
"""


# =========================================================
# 6. CONVERSION POSITION -> CONSIGNE ANGULAIRE
# =========================================================

def convertirPosition(contexte, x_brut, y_brut):
    """Transforme une position en carreaux en consigne angulaire, ou None si impossible."""

    """
    Cout mesure : environ 4 microsecondes. C'est pour cela
    qu'elle est appelee DANS la boucle plutot que precalculee
    sur toute la trajectoire : ca ne coute rien, et ca rend
    le flux temps reel possible sans changer une ligne.
    """

    modele = contexte["modele"]
    correction = contexte["correction"]

    x, y = serrerDansEnveloppe(modele, x_brut, y_brut)

    erreur_x, erreur_y = predireCorrection(correction, x, y)

    x_corrige = x - erreur_x
    y_corrige = y - erreur_y

    # La correction peut pousser la cible dehors : on la
    # rabat plutot que de perdre le point.
    if not dansEnveloppe(modele, x_corrige, y_corrige):
        x_corrige, y_corrige = serrerDansEnveloppe(modele, x_corrige, y_corrige, 0.0)

    resultat = anglesPourPoint(modele, x_corrige, y_corrige)

    if resultat is None:
        return None

    j4, j5 = resultat

    if not (J4_MIN <= j4 <= J4_MAX and J5_MIN <= j5 <= J5_MAX):
        return None

    return {
        "x": x,
        "y": y,
        "x_corrige": x_corrige,
        "y_corrige": y_corrige,
        "j4": j4,
        "j5": j5,
    }


def amplitude(consigne_a, consigne_b):
    """Ecart angulaire entre deux consignes, au sens du plus grand axe."""

    """
    J4 et J5 bougent en parallele : le mouvement dure le
    temps du plus lent, donc du plus grand ecart. Prendre la
    somme ou la norme surestimerait la course reelle.
    """

    return max(
        abs(consigne_b["j4"] - consigne_a["j4"]),
        abs(consigne_b["j5"] - consigne_a["j5"]),
    )


# =========================================================
# 7. EXECUTION - MODE ENCHAINE
# =========================================================

def suivreEnchaine(mc, base, j6, source, contexte,
                   tolerance=TOLERANCE_ENCHAINEMENT, total=None):
    """Enchaine les positions en attendant chaque point a la tolerance pres."""

    """
    Boucle volontairement nue :

        convertir -> send_angles -> lire -> comparer -> suivant

    Pas d'appel a is_moving() : c'est un aller-retour serie
    de plus par scrutation, pour une information qu'on
    deduit deja de l'ecart angulaire.

    Pas d'appel a attendreStabilisation() : on veut repartir
    AVANT l'arret, c'est ce qui enchaine les segments.
    """

    releves = []

    depart_global = time.perf_counter()

    precedente = None
    numero = 0

    for x_brut, y_brut in source:

        numero += 1

        consigne = convertirPosition(contexte, x_brut, y_brut)

        if consigne is None:
            print("  %4d  position inatteignable, ignoree" % numero)
            continue

        pose = [base[0], base[1], base[2], consigne["j4"], consigne["j5"], j6]

        limite = TIMEOUT_PREMIER_POINT if precedente is None else TIMEOUT_POINT

        t0 = time.perf_counter()

        try:
            mc.send_angles(pose, VITESSE_SUIVI)

        except Exception as e:
            print("  %4d  consigne refusee : %s" % (numero, e))
            continue

        mesure = None
        atteint = False

        while time.perf_counter() - t0 < limite:

            time.sleep(PERIODE_LECTURE)

            lecture = lireAngles(mc)

            if lecture is None:
                continue

            mesure = lecture

            ecart = max(
                abs(lecture[3] - consigne["j4"]),
                abs(lecture[4] - consigne["j5"]),
            )

            if ecart <= tolerance:
                atteint = True
                break

        t1 = time.perf_counter()

        course = amplitude(precedente, consigne) if precedente is not None else float("nan")

        releves.append(construireReleve(
            numero,
            t1 - depart_global,
            consigne,
            mesure,
            t1 - t0,
            course,
            atteint,
        ))

        precedente = consigne

        print("  %4d%s  %.3f s  %s" % (
            numero,
            ("/%d" % total) if total else "",
            t1 - t0,
            "ok" if atteint else "MANQUE",
        ))

    return releves


# =========================================================
# 8. EXECUTION - MODE FLUX
# =========================================================

def suivreFlux(mc, base, j6, source, contexte,
               frequence=FREQUENCE_FLUX, total=None):
    """Envoie les consignes a cadence fixe sans jamais attendre l'arrivee."""

    """
    C'est le mode qui represente vraiment le suivi : la
    camera sort une position a cadence fixe, on la pousse au
    bras, et tant pis si le bras n'y est pas encore. La
    consigne suivante ecrase la precedente en plein vol.

    Ce qui est mesure ici n'est plus un temps de deplacement
    -- il n'y a plus d'arrivee -- mais le RETARD : l'ecart
    entre la position du bras et la consigne en cours au
    moment du tick.

    Le rythme est tenu sur une HORLOGE ABSOLUE et non par un
    sleep constant : sinon le temps de la conversion et de
    la lecture serie s'ajouterait a chaque tour et la
    cadence deriverait silencieusement.
    """

    periode = 1.0 / frequence

    releves = []

    depart_global = time.perf_counter()

    precedente = None
    numero = 0

    for x_brut, y_brut in source:

        numero += 1

        echeance = depart_global + numero * periode

        t0 = time.perf_counter()

        consigne = convertirPosition(contexte, x_brut, y_brut)

        if consigne is None:
            print("  %4d  position inatteignable, ignoree" % numero)
            continue

        pose = [base[0], base[1], base[2], consigne["j4"], consigne["j5"], j6]

        try:
            mc.send_angles(pose, VITESSE_SUIVI)

        except Exception as e:
            print("  %4d  consigne refusee : %s" % (numero, e))
            continue

        mesure = lireAngles(mc)

        ecart = float("nan")

        if mesure is not None:
            ecart = max(
                abs(mesure[3] - consigne["j4"]),
                abs(mesure[4] - consigne["j5"]),
            )

        course = amplitude(precedente, consigne) if precedente is not None else float("nan")

        t1 = time.perf_counter()

        releves.append(construireReleve(
            numero,
            t1 - depart_global,
            consigne,
            mesure,
            t1 - t0,
            course,
            ecart <= TOLERANCE_ENCHAINEMENT,
            mesurer_vitesse=False,
        ))

        precedente = consigne

        if total and numero % 10 == 0:
            print("  %4d/%d  retard %.2f deg" % (numero, total, ecart))

        # Tenue de la cadence. Un retard n'est jamais rattrape
        # en accelerant : il est signale.
        reste = echeance - time.perf_counter()

        if reste > 0:
            time.sleep(reste)

        elif numero > 1:
            print("  tick %d en retard de %.3f s : cadence trop haute" % (numero, -reste))

    return releves


# =========================================================
# 9. RELEVES ET ENREGISTREMENT
# =========================================================

def construireReleve(numero, t_relatif, consigne, mesure, duree, course, atteint,
                     mesurer_vitesse=True):
    """Assemble une ligne de resultat a partir de la consigne et de la pose lue."""

    """
    mesurer_vitesse est a False en mode FLUX.

    La duree y est le temps du TICK (envoyer + lire), pas la
    duree d'un mouvement : il n'y a pas d'arrivee a
    chronometrer. Diviser l'amplitude commandee par ce
    temps-la donnait 16 000 deg/s, un chiffre qui ne decrit
    rien. On laisse la case vide plutot que de la remplir
    avec du faux.
    """

    j4_atteint = mesure[3] if mesure is not None else float("nan")
    j5_atteint = mesure[4] if mesure is not None else float("nan")

    vitesse = float("nan")

    if mesurer_vitesse and duree > 0 and course == course:
        vitesse = course / duree

    return {
        "numero": numero,
        "t_relatif_s": t_relatif,
        "x_demande": consigne["x"],
        "y_demande": consigne["y"],
        "x_corrige": consigne["x_corrige"],
        "y_corrige": consigne["y_corrige"],
        "J4_consigne": consigne["j4"],
        "J5_consigne": consigne["j5"],
        "J4_atteint": j4_atteint,
        "J5_atteint": j5_atteint,
        "ecart_J4": j4_atteint - consigne["j4"],
        "ecart_J5": j5_atteint - consigne["j5"],
        "duree_s": duree,
        "amplitude_deg": course,
        "vitesse_deg_s": vitesse,
        "atteint": atteint,
    }


def completerImpacts(modele, releves):
    """Ajoute la position reellement pointee, en carreaux, deduite des angles atteints."""

    """
    L'ecart angulaire ne parle pas : 2 degres, c'est peu ou
    beaucoup selon la distance. On repasse les angles LUS
    dans le modele direct pour obtenir le point vise au sol,
    et l'erreur devient lisible en carreaux -- la meme unite
    que le reste du projet.

    ATTENTION : c'est l'erreur du MODELE, pas une mesure du
    spot. Elle dit ou le bras pointe d'apres sa pose, pas ou
    la tache rouge est reellement tombee. Pour ca, il faut
    la campagne de calibration et l'oeil de l'operateur.
    """

    for releve in releves:

        j4 = releve["J4_atteint"]
        j5 = releve["J5_atteint"]

        impact = None

        if j4 == j4 and j5 == j5:
            impact = predireImpact(modele, j4, j5)

        if impact is None:
            releve["x_atteint"] = float("nan")
            releve["y_atteint"] = float("nan")
            releve["erreur_carreaux"] = float("nan")
            continue

        releve["x_atteint"] = impact[0]
        releve["y_atteint"] = impact[1]

        releve["erreur_carreaux"] = math.hypot(
            impact[0] - releve["x_demande"],
            impact[1] - releve["y_demande"],
        )

    return releves


def enregistrerReleves(releves, chemin):
    """Ecrit le CSV de la session de suivi."""

    os.makedirs(DOSSIER_RESULTATS, exist_ok=True)

    try:
        with open(chemin, "w", encoding="utf-8", newline="") as f:

            ecrivain = csv.writer(f)
            ecrivain.writerow(ENTETE_SUIVI)

            for releve in releves:
                ecrivain.writerow([releve.get(colonne, "") for colonne in ENTETE_SUIVI])

    except Exception as e:
        print("Ecriture du CSV impossible :", e)
        return False

    return True


# =========================================================
# 10. BILAN
# =========================================================

def propres(valeurs):
    """Retire les NaN d'une liste."""

    return [v for v in valeurs if v == v]


def moyenne(valeurs):
    """Moyenne en ignorant les NaN, ou NaN si rien d'exploitable."""

    utiles = propres(valeurs)

    if not utiles:
        return float("nan")

    return sum(utiles) / len(utiles)


def maximum(valeurs):
    """Maximum en ignorant les NaN, ou NaN si rien d'exploitable."""

    utiles = propres(valeurs)

    if not utiles:
        return float("nan")

    return max(utiles)


def afficherBilan(releves, mode, tolerance):
    """Resume chiffre de la session, adapte au mode."""

    """
    Les deux modes ne mesurent PAS la meme chose, donc ils
    n'affichent pas les memes chiffres.

    ENCHAINE a une arrivee par point : le temps de
    deplacement et la vitesse angulaire ont un sens.

    FLUX n'en a pas : la consigne suivante arrive avant. Ce
    qui a un sens la, c'est le RETARD de la tete sur la
    cible au moment du tick. Afficher un "temps par point"
    de 0,000 s serait une reponse a une question que
    personne n'a posee.
    """

    if not releves:
        print("Aucun releve.")
        return

    total = releves[-1]["t_relatif_s"]

    erreurs = [r["erreur_carreaux"] for r in releves]
    courses = [r["amplitude_deg"] for r in releves]

    dans_tolerance = sum(1 for r in releves if r["atteint"])

    print()
    print("=========================================")
    print("BILAN -", mode.upper())
    print("=========================================")
    print()

    print("Points               : %d" % len(releves))
    print("Duree totale         : %.2f s" % total)

    if total > 0:
        print("Cadence effective    : %.2f points/s" % (len(releves) / total))

    print()
    print("Amplitude par saut   : moy %.2f deg   max %.2f deg" % (
        moyenne(courses),
        maximum(courses),
    ))

    if mode == "flux":

        # Retard de la tete sur la cible, au sens du plus
        # grand axe : c'est ce que le mode flux mesure.
        retards = [
            max(abs(r["ecart_J4"]), abs(r["ecart_J5"]))
            for r in releves
        ]

        print()
        print("RETARD sur la cible  : moy %.2f deg   max %.2f deg" % (
            moyenne(retards),
            maximum(retards),
        ))

        print("Ticks a moins de %.1f deg : %d  (%.0f %%)" % (
            tolerance,
            dans_tolerance,
            100.0 * dans_tolerance / len(releves),
        ))

        print()
        print("Si le retard ne redescend jamais, la cadence est")
        print("trop haute pour ce bras : baisser la frequence.")

    else:

        durees = [r["duree_s"] for r in releves]
        vitesses = [r["vitesse_deg_s"] for r in releves]

        print()
        print("Temps par point      : moy %.3f s   min %.3f s   max %.3f s" % (
            moyenne(durees),
            min(durees),
            max(durees),
        ))

        print("Vitesse angulaire    : moy %.1f deg/s   max %.1f deg/s" % (
            moyenne(vitesses),
            maximum(vitesses),
        ))

        print("Points dans %.1f deg  : %d  (%.0f %%)" % (
            tolerance,
            dans_tolerance,
            100.0 * dans_tolerance / len(releves),
        ))

    print()
    print("Erreur de pointage (deduite du MODELE, pas une mesure du spot) :")
    print("  moyenne %.2f carreaux   max %.2f carreaux" % (
        moyenne(erreurs),
        maximum(erreurs),
    ))


def tracerSuivi(releves, chemin, mode):
    """Trace la trajectoire demandee, la trajectoire suivie et l'erreur au cours du temps."""

    if not TRACE_DISPONIBLE:
        print("matplotlib absent : figure non tracee (le CSV, lui, est ecrit).")
        return False

    if not releves:
        return False

    x_demande = [r["x_demande"] for r in releves]
    y_demande = [r["y_demande"] for r in releves]

    x_atteint = [r["x_atteint"] for r in releves]
    y_atteint = [r["y_atteint"] for r in releves]

    temps = [r["t_relatif_s"] for r in releves]
    erreurs = [r["erreur_carreaux"] for r in releves]

    figure, axes = plt.subplots(1, 2, figsize=(13, 5.5))

    axes[0].plot(x_demande, y_demande, "-o", markersize=3, linewidth=1.0,
                 label="cible (frelon)")

    axes[0].plot(x_atteint, y_atteint, "-o", markersize=3, linewidth=1.0,
                 label="pointeur (pose lue)")

    axes[0].set_xlabel("X (carreaux)")
    axes[0].set_ylabel("Y (carreaux)")
    axes[0].set_title("Trajectoire - mode %s" % mode)
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    axes[0].set_aspect("equal", adjustable="datalim")

    axes[1].plot(temps, erreurs, "-", linewidth=1.2)
    axes[1].set_xlabel("temps (s)")
    axes[1].set_ylabel("erreur (carreaux)")
    axes[1].set_title("Ecart pointeur / cible")
    axes[1].grid(True, alpha=0.3)

    figure.tight_layout()

    try:
        figure.savefig(chemin, dpi=140)

    except Exception as e:
        print("Enregistrement de la figure impossible :", e)
        plt.close(figure)
        return False

    plt.close(figure)

    return True


# =========================================================
# 11. INTERFACE
# =========================================================

def demanderEntier(message, defaut):
    """Demande un entier positif, avec valeur par defaut."""

    while True:

        reponse = input("%s [%d] > " % (message, defaut)).strip()

        if reponse == "":
            return defaut

        try:
            valeur = int(reponse)

        except ValueError:
            print("  Saisir un nombre entier.")
            continue

        if valeur > 0:
            return valeur

        print("  Le nombre doit etre superieur a 0.")


def demanderFlottant(message, defaut):
    """Demande un nombre reel positif, avec valeur par defaut."""

    while True:

        reponse = input("%s [%.2f] > " % (message, defaut)).strip()

        if reponse == "":
            return defaut

        try:
            valeur = float(reponse.replace(",", "."))

        except ValueError:
            print("  Saisir un nombre.")
            continue

        if valeur > 0:
            return valeur

        print("  Le nombre doit etre superieur a 0.")


def choisirTrajectoire(modele):
    """Menu de choix de la source de positions. Retourne (positions, nom)."""

    """
    Les positions sont materialisees en liste ICI, et pas
    plus tard : on veut pouvoir les compter, les verifier et
    les afficher avant que le bras ne bouge. Une source
    camera, elle, restera un generateur.
    """

    print()
    print("=========================================")
    print("TRAJECTOIRE A SUIVRE")
    print("=========================================")
    print()
    print("  1 - Vol de frelon simule (aleatoire, continu)")
    print("  2 - Cercle")
    print("  3 - Carre")
    print("  4 - Zigzag (pire cas : inverse J4 a chaque point)")
    print("  5 - Fichier CSV ou JSON")
    print()

    choix = input("Choix [1] > ").strip() or "1"

    if choix == "5":

        chemin = input("Chemin du fichier > ").strip()

        return chargerTrajectoireFichier(chemin), "fichier"

    nombre = demanderEntier("Nombre de positions", 40)

    if choix == "2":
        return list(genererFigure(modele, "cercle", nombre)), "cercle"

    if choix == "3":
        return list(genererFigure(modele, "carre", nombre)), "carre"

    if choix == "4":
        return list(genererFigure(modele, "zigzag", nombre)), "zigzag"

    pas = demanderFlottant("Pas maximum par point (carreaux)", 2.0)

    return list(genererVolFrelon(modele, nombre, pas_max=pas)), "frelon"


def choisirMode():
    """Menu de choix du mode d'execution. Retourne (mode, reglage)."""

    print()
    print("=========================================")
    print("MODE D'EXECUTION")
    print("=========================================")
    print()
    print("  1 - ENCHAINE : attendre chaque point a la")
    print("      tolerance pres, puis envoyer le suivant.")
    print("      -> mesure la vitesse point a point.")
    print()
    print("  2 - FLUX : envoyer a cadence fixe sans jamais")
    print("      attendre l'arrivee.")
    print("      -> mesure le retard du bras sur la cible.")
    print("      -> c'est le mode de la camera.")
    print()

    choix = input("Choix [1] > ").strip() or "1"

    if choix == "2":
        return "flux", demanderFlottant("Cadence (Hz)", FREQUENCE_FLUX)

    print()
    print("Tolerance : 0,2 deg = precision calibration (lent)")
    print("            2,0 deg = defaut, environ 2 cm au sol")
    print("            5,0 deg = tres rapide, le bras frole les points")

    return "enchaine", demanderFlottant(
        "Tolerance d'enchainement (degres)",
        TOLERANCE_ENCHAINEMENT,
    )


# =========================================================
# 12. PROGRAMME PRINCIPAL
# =========================================================

def main():

    print("=========================================")
    print("SUIVI DE TRAJECTOIRE - SIMULATION FRELON")
    print("=========================================")
    print()
    print("SECURITE : pointeur rouge classe 2, laser de")
    print("puissance DEBRANCHE. Contrairement aux scripts de")
    print("calibration, le pointeur reste ALLUME pendant les")
    print("mouvements.")
    print()

    # -----------------------------------------------------
    # Calibration
    # -----------------------------------------------------

    print("Reconstruction de Calib 2...")

    modele = construireModeleCalib2()

    if modele is None:
        return 1

    print()
    print("Calib 2 chargee : %d points d'ajustement" % modele.get("nb_points", 0))

    print("Zone calibree : x %.1f..%.1f, y %.1f..%.1f carreaux" % (
        modele["enveloppe"]["u_min"],
        modele["enveloppe"]["u_max"],
        modele["enveloppe"]["v_min"],
        modele["enveloppe"]["v_max"],
    ))

    correction = chargerModeleCorrection()

    contexte = {
        "modele": modele,
        "correction": correction,
    }

    # -----------------------------------------------------
    # Trajectoire et mode
    # -----------------------------------------------------

    positions, nom_trajectoire = choisirTrajectoire(modele)

    if not positions:
        print("Aucune position a suivre.")
        return 1

    print()
    print("%d position(s) dans la trajectoire." % len(positions))

    mode, reglage = choisirMode()

    # -----------------------------------------------------
    # Verification a blanc : toutes les positions
    # donnent-elles des angles valides ? Se fait AVANT la
    # connexion, donc sans robot et sans risque.
    # -----------------------------------------------------

    invalides = sum(
        1 for x, y in positions
        if convertirPosition(contexte, x, y) is None
    )

    if invalides:
        print()
        print("ATTENTION : %d position(s) sur %d ne donnent pas d'angles valides."
              % (invalides, len(positions)))
        print("Elles seront ignorees pendant le suivi.")

    # -----------------------------------------------------
    # Robot
    # -----------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        return 1

    allerPositionInitiale(mc)

    if not verifierBase(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    print()
    print("Base verrouillee :", [round(a, 2) for a in base])

    print()
    input("Entree pour lancer le suivi (Ctrl+C pour arreter) > ")

    # -----------------------------------------------------
    # Suivi
    # -----------------------------------------------------

    releves = []

    try:

        if LASER_PENDANT_SUIVI:
            allumerPointeur(mc)

        print()
        print("Suivi en cours...")
        print()

        if mode == "flux":
            releves = suivreFlux(
                mc, base, j6,
                iter(positions), contexte,
                frequence=reglage,
                total=len(positions),
            )

        else:
            releves = suivreEnchaine(
                mc, base, j6,
                iter(positions), contexte,
                tolerance=reglage,
                total=len(positions),
            )

    except KeyboardInterrupt:
        print()
        print("Suivi interrompu par l'operateur.")

    finally:

        # Le pointeur s'eteint quoi qu'il arrive, y compris
        # sur Ctrl+C et sur exception.
        eteindrePointeur(mc)

        print()
        print("Pointeur eteint.")

    if not releves:
        print("Aucun releve a enregistrer.")
        return 1

    # -----------------------------------------------------
    # Resultats
    # -----------------------------------------------------

    completerImpacts(modele, releves)

    horodatage = datetime.now().strftime("%Y%m%d_%H%M%S")

    base_nom = "suivi_%s_%s_%s" % (nom_trajectoire, mode, horodatage)

    chemin_csv = os.path.join(DOSSIER_RESULTATS, base_nom + ".csv")
    chemin_png = os.path.join(DOSSIER_RESULTATS, base_nom + ".png")

    tolerance_bilan = reglage if mode == "enchaine" else TOLERANCE_ENCHAINEMENT

    afficherBilan(releves, mode, tolerance_bilan)

    print()

    if enregistrerReleves(releves, chemin_csv):
        print("CSV    :", chemin_csv)

    if tracerSuivi(releves, chemin_png, mode):
        print("Figure :", chemin_png)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
