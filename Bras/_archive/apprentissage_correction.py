"""
=========================================================
APPRENTISSAGE D'UNE CORRECTION DE POINTAGE
=========================================================

Objectif :
---------
Le robot vise lui-meme une serie de cases, l'operateur dit
ou le spot est REELLEMENT tombe, et le script apprend la
correction a appliquer pour que la prochaine visee tombe
juste.

Ce que ce script mesure, et que la calibration ne mesure pas :
------------------------------------------------------------
calibration_grille.py releve des points POINTES A LA MAIN :
la pose enregistree est celle ou le spot etait pose. Elle
mesure donc le modele, et seulement lui.

Ici c'est l'inverse : on demande une case, le robot y va
tout seul, et on regarde ou ca tombe. L'ecart contient TOUTE
la chaine -- modele, ecart de repos du bras, jeu des
reducteurs, deformation du quadrillage, position du guide de
lecture. C'est la seule mesure qui corresponde a ce que fait
le demonstrateur le jour J.

Ce que la correction peut rattraper, et ce qu'elle ne peut pas :
--------------------------------------------------------------
Une erreur REPRODUCTIBLE se rattrape : si la meme case rate
toujours du meme cote, il suffit de viser a cote. Une erreur
DISPERSEE ne se rattrape pas : si la meme case tombe tantot
a gauche tantot a droite, aucune correction ne le corrige,
et une carte apprise sur ces points apprend du bruit.

D'ou les deux garde-fous du script :

  1. Quelques cases sont visees DEUX FOIS, au debut et a la
     fin du balayage. L'ecart entre les deux visites donne le
     plancher de bruit. Une correction ne peut pas descendre
     en dessous.

  2. Les corrections candidates sont departagees en
     VALIDATION CROISEE (leave-one-out) : chaque point est
     predit par une correction ajustee SANS lui. Une carte
     qui apprend le bruit se trahit la, et le script garde
     alors "aucune correction" en le disant.

Les corrections candidates, de la plus sobre a la plus riche :

  aucune       -- temoin, l'erreur telle qu'elle est
  constante    -- 2 parametres, un simple decalage
  affine       -- 6 parametres, decalage + echelle + rotation
                  + cisaillement du quadrillage
  quadratique  -- 12 parametres, deformation douce
  carte        -- les points eux-memes, interpoles par
                  ponderation inverse de la distance

Une correction n'est adoptee que si elle fait au moins
GAIN_MINIMAL de mieux que le temoin EN VALIDATION CROISEE.

Deroulement d'une seance :
-------------------------
  1. choix de la calibration de depart et verification de la base ;
  2. balayage : le robot vise, l'operateur saisit l'impact reel ;
  3. ajustement, comparaison des candidates, verdict ;
  4. verification facultative : quelques tirs avec la
     correction appliquee, pour mesurer le gain REEL.

Utilisation :
------------
    python3 apprentissage_correction.py
    python3 apprentissage_correction.py --ajuster-seulement   # sans robot

Les mesures sont ecrites au fur et a mesure dans
resultats/balayage_correction.csv : une seance interrompue
n'est pas perdue, et --ajuster-seulement les rejoue.

ATTENTION :
----------
Pointeur rouge classe 2 uniquement. Le laser de puissance
reste DEBRANCHE, pas seulement eteint.
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import os
import csv
import sys
import math
from datetime import datetime

"""
La couche robot, le modele et le choix de la campagne
existent deja. Les deux modules importes gardent leur code
executable sous if __name__ == "__main__" : ces imports
n'ouvrent aucun port serie et ne declenchent aucun mouvement.
"""

from calibration_grille import (
    COTE_CARREAU,
    DOSSIER_RESULTATS,
    allumerPointeur,
    anglesPourPoint,
    connecterRobot,
    dansEnveloppe,
    enteteCompatible,
    eteindrePointeur,
    lireAngles,
    lireTexte,
    moindresCarres,
    viserAngles,
)

from calibration_test import (
    choisirCalibration,
    construireModeleSerie,
    verifierBase,
)


# =========================================================
# 2. PARAMETRES
# =========================================================

FICHIER_BALAYAGE = os.path.join(DOSSIER_RESULTATS, "balayage_correction.csv")
FICHIER_CORRECTION = os.path.join(DOSSIER_RESULTATS, "correction_pointage.json")

ENTETE_BALAYAGE = [
    "horodatage", "serie", "usage", "index",
    "u_demande_carreaux", "v_demande_carreaux",
    "j4_consigne", "j5_consigne", "J4", "J5",
    "u_reel_carreaux", "v_reel_carreaux",
    "erreur_u_carreaux", "erreur_v_carreaux",
]

"""
Pas du balayage.

Une correction ne peut decrire que ce qu'elle a vu : avec un
pas de 6 carreaux sur 24 x 27, cela fait 5 x 5 = 25 cases,
soit une bonne demi-heure a la main. Descendre a 4 donne 7 x 8
= 56 cases et plus d'une heure, pour un gain qui n'existe que
si l'erreur est structuree A CETTE ECHELLE -- ce que le
verdict du script dit apres coup.
"""
PAS_BALAYAGE = 6.0          # carreaux

"""
Cases revisitees en fin de balayage pour mesurer le bruit.

Elles sont choisies dans le balayage lui-meme, donc elles ne
coutent que quelques tirs, et l'ecart entre les deux visites
contient tout ce qui n'est pas reproductible : jeu du bras,
lecture du quadrillage, derive lente de la base pendant la
seance. C'est le plancher que la correction ne franchira pas.
"""
CASES_CONTROLE = 4

"""
Gain minimal exige en validation croisee pour adopter une
correction plutot que le temoin.

Une correction qui gagne 3 % ne gagne rien : elle deplace le
probleme et ajoute un fichier de plus a maintenir. On veut un
gain franc ou rien.
"""
GAIN_MINIMAL = 0.15         # 15 %

"""
Avance exigee d'une correction riche sur une correction
sobre pour lui etre preferee.

Le chiffre de validation croisee est lui-meme bruite : il
est estime sur une trentaine de points, donc a quelque
chose comme 1/sqrt(2n), soit 13 %, pres. Departager deux
candidates a 2 % d'ecart, c'est tirer a pile ou face -- et
le perdant du tirage est toujours la correction la plus
simple, celle qui se relit et qui ne part pas en vrille
hors de la zone mesuree.
"""
MARGE_SOBRIETE = 0.10       # 10 %

"""
Distance de lissage de la carte interpolee.

Une interpolation qui passe exactement par chaque point
mesure reproduit aussi son bruit. En lissant sur l'ordre du
pas de balayage, la carte decrit la tendance locale au lieu
de chaque point. La validation croisee tranche ensuite.
"""
LISSAGE_CARTE = PAS_BALAYAGE

# Nombre de tirs proposes par defaut pour la verification finale.
TIRS_VERIFICATION = 6


# =========================================================
# 3. LES CASES A BALAYER
# =========================================================

def casesDuBalayage(modele, pas=PAS_BALAYAGE):
    """Retourne la liste des cases a viser, en serpentin, dans la zone calibree."""

    """
    Le serpentin evite de traverser toute la zone a chaque
    changement de ligne : c'est du temps operateur, et une
    occasion de moins de bousculer le bras.

    Les cases sont arrondies au noeud entier : l'operateur
    lit sur le quadrillage, une cible a 6,4 carreaux ne se
    verifie pas.
    """

    enveloppe = modele["enveloppe"]

    cases = []
    ligne = 0

    v = enveloppe["v_min"]

    while v <= enveloppe["v_max"] + 1e-9:

        colonne = []
        u = enveloppe["u_min"]

        while u <= enveloppe["u_max"] + 1e-9:
            colonne.append((round(u), round(v)))
            u += pas

        # Une ligne sur deux est parcourue a l'envers.
        if ligne % 2:
            colonne.reverse()

        cases.extend(colonne)

        ligne += 1
        v += pas

    return [c for c in cases if dansEnveloppe(modele, c[0], c[1])]


def casesDeControle(cases, nombre=CASES_CONTROLE):
    """Retourne les cases a revisiter en fin de balayage pour mesurer le bruit."""

    """
    Etalees sur le parcours, pour que l'ecart entre les deux
    visites ne soit pas celui d'un seul coin de la zone.
    """

    if not cases or nombre <= 0:
        return []

    nombre = min(nombre, len(cases))

    pas = max(1, len(cases) // nombre)

    return [cases[min(i * pas, len(cases) - 1)] for i in range(nombre)]


# =========================================================
# 4. ENREGISTREMENT DES MESURES
# =========================================================

def enregistrerMesure(serie, usage, index, demande, angles, reel, chemin=None):
    """Ajoute une ligne au fichier de balayage et retourne True si l'ecriture a reussi."""

    if chemin is None:
        chemin = FICHIER_BALAYAGE

    try:
        os.makedirs(os.path.dirname(chemin), exist_ok=True)

        nouveau = not os.path.exists(chemin)

        if not nouveau and not enteteCompatible(chemin, ENTETE_BALAYAGE):
            return False

        with open(chemin, "a", newline="") as fichier:

            redacteur = csv.writer(fichier)

            if nouveau:
                redacteur.writerow(ENTETE_BALAYAGE)

            erreur = ("", "") if reel is None else (
                round(reel[0] - demande[0], 3), round(reel[1] - demande[1], 3))

            redacteur.writerow([
                datetime.now().isoformat(timespec="seconds"),
                serie, usage, index,
                round(demande[0], 3), round(demande[1], 3),
                round(angles[0], 3), round(angles[1], 3),
                round(angles[2], 3), round(angles[3], 3),
                "" if reel is None else round(reel[0], 3),
                "" if reel is None else round(reel[1], 3),
                erreur[0], erreur[1],
            ])

        return True

    except Exception as e:
        print("Impossible d'ecrire la mesure dans le fichier.")
        print("Erreur :", e)
        return False


def chargerMesuresBalayage(chemin=None, serie=None, usage=None):
    """Lit le fichier de balayage et retourne les points exploitables."""

    if chemin is None:
        chemin = FICHIER_BALAYAGE

    points = []

    try:
        with open(chemin, "r", newline="") as fichier:

            for ligne in csv.DictReader(fichier):

                if serie is not None and (ligne.get("serie") or "").strip() != serie:
                    continue

                if usage is not None and (ligne.get("usage") or "").strip() != usage:
                    continue

                try:
                    if not ligne.get("u_reel_carreaux"):
                        continue

                    points.append({
                        "u": float(ligne["u_demande_carreaux"]),
                        "v": float(ligne["v_demande_carreaux"]),
                        "eu": float(ligne["erreur_u_carreaux"]),
                        "ev": float(ligne["erreur_v_carreaux"]),
                        "horodatage": ligne.get("horodatage", ""),
                    })

                except (ValueError, KeyError):
                    continue

    except FileNotFoundError:
        print("Fichier de balayage introuvable :", chemin)
        return []

    except Exception as e:
        print("Impossible de lire le fichier de balayage.")
        print("Erreur :", e)
        return []

    return points


def listerSeriesBalayage(chemin=None):
    """Retourne les noms de series presentes dans le fichier de balayage, dans l'ordre d'apparition."""

    if chemin is None:
        chemin = FICHIER_BALAYAGE

    series = []

    try:
        with open(chemin, "r", newline="") as fichier:

            for ligne in csv.DictReader(fichier):

                nom = (ligne.get("serie") or "").strip()

                if nom and nom not in series:
                    series.append(nom)

    except Exception:
        return []

    return series


# =========================================================
# 5. LE BRUIT DE LA MESURE
# =========================================================

def planchierDeBruit(points):
    """Estime l'ecart type d'une mesure a partir des cases visees plusieurs fois, ou None."""

    """
    Deux visites de la meme case donnent deux erreurs qui ne
    different que par ce qui n'est pas reproductible. La
    difference de deux tirages de meme ecart type vaut
    sqrt(2) fois cet ecart type : on divise donc par sqrt(2).

    C'est le chiffre le plus important de la seance. Une
    correction qui annonce mieux que ce plancher ne mesure
    plus rien de reel.
    """

    visites = {}

    for point in points:
        visites.setdefault((point["u"], point["v"]), []).append(point)

    ecarts = []

    for cle, liste in visites.items():

        if len(liste) < 2:
            continue

        for i in range(len(liste)):
            for j in range(i + 1, len(liste)):
                ecarts.append(math.hypot(liste[i]["eu"] - liste[j]["eu"],
                                         liste[i]["ev"] - liste[j]["ev"]))

    if not ecarts:
        return None

    quadratique = math.sqrt(sum(e * e for e in ecarts) / len(ecarts))

    return {
        "paires": len(ecarts),
        "ecart_rms_entre_visites": quadratique,
        "bruit_par_mesure": quadratique / math.sqrt(2.0),
        "pire": max(ecarts),
    }


# =========================================================
# 6. LES CORRECTIONS CANDIDATES
# =========================================================

def termes(u, v, degre):
    """Retourne les termes du polynome en (u, v) jusqu'au degre demande."""

    if degre <= 0:
        return [1.0]

    if degre == 1:
        return [1.0, u, v]

    return [1.0, u, v, u * u, u * v, v * v]


def ajusterPolynome(points, degre):
    """Ajuste un polynome sur les erreurs et retourne la correction, ou None si impossible."""

    """
    Les deux composantes de l'erreur sont ajustees
    separement : rien ne dit qu'elles se deforment de la
    meme facon, et les equations normales sont les memes.
    """

    nb_termes = len(termes(0.0, 0.0, degre))

    # Autant d'equations que d'inconnues ne mesure plus rien.
    if len(points) < nb_termes + 1:
        return None

    lignes = [termes(p["u"], p["v"], degre) for p in points]

    cu = moindresCarres(lignes, [p["eu"] for p in points])
    cv = moindresCarres(lignes, [p["ev"] for p in points])

    if cu is None or cv is None:
        return None

    return {"type": "polynome", "degre": degre, "cu": cu, "cv": cv}


def ajusterCarte(points, lissage=LISSAGE_CARTE):
    """Retourne une correction qui interpole les points mesures, ou None s'il n'y en a pas assez."""

    if len(points) < 3:
        return None

    return {
        "type": "carte",
        "lissage": lissage,
        "points": [{"u": p["u"], "v": p["v"], "eu": p["eu"], "ev": p["ev"]} for p in points],
    }


def evaluerCorrection(correction, u, v):
    """Retourne l'erreur (eu, ev) que la correction prevoit a la case demandee."""

    if correction is None:
        return (0.0, 0.0)

    if correction["type"] == "polynome":

        base = termes(u, v, correction["degre"])

        return (sum(c * t for c, t in zip(correction["cu"], base)),
                sum(c * t for c, t in zip(correction["cv"], base)))

    if correction["type"] == "carte":

        """
        Ponderation inverse de la distance, adoucie par le
        lissage. Le terme en lissage^2 evite la division par
        zero sur un point mesure, et empeche la carte de
        passer exactement par lui -- donc d'en reproduire le
        bruit.
        """

        lissage = correction.get("lissage", LISSAGE_CARTE)

        poids_total = 0.0
        somme_u = 0.0
        somme_v = 0.0

        for point in correction["points"]:

            distance2 = (u - point["u"]) ** 2 + (v - point["v"]) ** 2

            poids = 1.0 / (distance2 + lissage * lissage)

            poids_total += poids
            somme_u += poids * point["eu"]
            somme_v += poids * point["ev"]

        if poids_total == 0.0:
            return (0.0, 0.0)

        return (somme_u / poids_total, somme_v / poids_total)

    return (0.0, 0.0)


# =========================================================
# 7. DEPARTAGER LES CANDIDATES
# =========================================================

def rmsResiduel(points, correction):
    """Retourne l'ecart quadratique moyen qui resterait apres application de la correction."""

    if not points:
        return None

    total = 0.0

    for p in points:

        prevu = evaluerCorrection(correction, p["u"], p["v"])

        total += (p["eu"] - prevu[0]) ** 2 + (p["ev"] - prevu[1]) ** 2

    return math.sqrt(total / len(points))


def rmsValidationCroisee(points, ajuster):
    """Retourne le rms predit sur des points JAMAIS vus par l'ajustement, ou None."""

    """
    Chaque point est predit par une correction ajustee sur
    tous les autres. C'est la seule facon de savoir si une
    correction a appris la deformation ou le bruit : une
    carte qui colle parfaitement aux points qui l'ont
    produite peut tres bien etre pire qu'aucune correction
    sur le point d'a cote.
    """

    if len(points) < 4:
        return None

    total = 0.0
    comptes = 0

    for i in range(len(points)):

        autres = points[:i] + points[i + 1:]

        correction = ajuster(autres)

        if correction is None:
            return None

        prevu = evaluerCorrection(correction, points[i]["u"], points[i]["v"])

        total += (points[i]["eu"] - prevu[0]) ** 2 + (points[i]["ev"] - prevu[1]) ** 2
        comptes += 1

    if comptes == 0:
        return None

    return math.sqrt(total / comptes)


def candidates():
    """Retourne les corrections candidates, de la plus sobre a la plus riche."""

    return [
        ("aucune", lambda pts: None),
        ("constante", lambda pts: ajusterPolynome(pts, 0)),
        ("affine", lambda pts: ajusterPolynome(pts, 1)),
        ("quadratique", lambda pts: ajusterPolynome(pts, 2)),
        ("carte", lambda pts: ajusterCarte(pts)),
    ]


def choisirCorrection(points, gain_minimal=GAIN_MINIMAL):
    """Compare les candidates en validation croisee et retourne (nom, correction, tableau)."""

    tableau = []

    for nom, ajuster in candidates():

        """
        Le temoin n'a aucun parametre a ajuster : il n'y a
        donc rien a valider sur des points laisses de cote,
        et sa validation croisee est l'erreur telle quelle.
        Le faire passer par rmsValidationCroisee() reviendrait
        a confondre "pas de correction" et "ajustement
        impossible", qui se signalent tous deux par None.
        """

        if nom == "aucune":

            brut = rmsResiduel(points, None)

            tableau.append({"nom": nom, "ajuste": brut, "croisee": brut, "raison": ""})
            continue

        correction = ajuster(points)

        if correction is None:
            tableau.append({"nom": nom, "ajuste": None, "croisee": None,
                            "raison": "pas assez de points"})
            continue

        tableau.append({
            "nom": nom,
            "ajuste": rmsResiduel(points, correction),
            "croisee": rmsValidationCroisee(points, ajuster),
            "raison": "",
        })

    temoin = next((l["croisee"] for l in tableau if l["nom"] == "aucune"), None)

    if temoin is None:
        return ("aucune", None, tableau)

    """
    A egalite de validation croisee, la correction la plus
    sobre gagne : elle a moins de facons de se tromper hors
    des points mesures, et elle se relit.
    """

    meilleure = None

    for ligne in tableau:

        if ligne["nom"] == "aucune" or ligne["croisee"] is None:
            continue

        if ligne["croisee"] > temoin * (1.0 - gain_minimal):
            continue

        if meilleure is None or ligne["croisee"] < meilleure["croisee"] * (1.0 - MARGE_SOBRIETE):
            meilleure = ligne

    if meilleure is None:
        return ("aucune", None, tableau)

    ajuster = dict(candidates())[meilleure["nom"]]

    return (meilleure["nom"], ajuster(points), tableau)


# =========================================================
# 8. APPLIQUER LA CORRECTION
# =========================================================

def cibleCorrigee(correction, u, v, tours=3):
    """Retourne la case a demander au modele pour que le spot tombe sur (u, v)."""

    """
    La correction dit l'erreur commise QUAND ON DEMANDE une
    case. Pour atteindre une case, il faut donc demander
    celle dont l'erreur ramene dessus : x tel que
    x + erreur(x) = cible. L'erreur variant lentement, trois
    tours de point fixe suffisent largement.

    La case demandee peut sortir de quelques dixiemes de la
    zone calibree ; la marge d'enveloppe du modele l'absorbe.
    """

    if correction is None:
        return (u, v)

    x, y = u, v

    for _ in range(tours):

        erreur = evaluerCorrection(correction, x, y)

        x = u - erreur[0]
        y = v - erreur[1]

    return (x, y)


def enregistrerCorrection(correction, infos, chemin=None):
    """Ecrit la correction au format JSON et retourne True si l'ecriture a reussi."""

    import json

    if chemin is None:
        chemin = FICHIER_CORRECTION

    if correction is None:
        contenu = dict(infos)
        contenu["type"] = "aucune"

    else:
        contenu = dict(correction)
        contenu.update(infos)

    try:
        os.makedirs(os.path.dirname(chemin), exist_ok=True)

        with open(chemin, "w") as fichier:
            json.dump(contenu, fichier, indent=2, ensure_ascii=False)

        return True

    except Exception as e:
        print("Impossible d'enregistrer la correction.")
        print("Erreur :", e)
        return False


def chargerCorrection(chemin=None):
    """Relit la correction enregistree, ou None s'il n'y en a pas d'utilisable."""

    import json

    if chemin is None:
        chemin = FICHIER_CORRECTION

    if not os.path.isfile(chemin):
        return None

    try:
        with open(chemin) as fichier:
            contenu = json.load(fichier)

    except Exception as e:
        print("Correction illisible :", e)
        return None

    if contenu.get("type") in ("polynome", "carte"):
        return contenu

    return None


# =========================================================
# 9. LA SEANCE
# =========================================================

def lireImpact(demande):
    """Demande ou le spot est tombe et retourne (u, v), None pour sauter, ou "fin"."""

    print("  Ou le spot est-il tombe ? deux nombres en carreaux (les demis comptent),")
    print("  s pour sauter cette case, f pour terminer le balayage.")

    while True:

        reponse = lireTexte("  impact reel > ")

        if reponse.lower() in ("f", "fin"):
            return "fin"

        if reponse.lower() in ("s", "saut", ""):
            return None

        morceaux = reponse.replace(",", " ").split()

        if len(morceaux) != 2:
            print("  Il faut deux nombres separes par un espace.")
            continue

        try:
            reel = (float(morceaux[0]), float(morceaux[1]))

        except ValueError:
            print("  Saisie illisible.")
            continue

        ecart = math.hypot(reel[0] - demande[0], reel[1] - demande[1])

        print("  Erreur : %+.1f / %+.1f carreau, soit %.2f carreaux (%.1f cm)"
              % (reel[0] - demande[0], reel[1] - demande[1], ecart, ecart * COTE_CARREAU))

        return reel


def tirer(mc, base, modele, j6, demande, correction=None):
    """Vise la case demandee et retourne (angles envoyes et atteints, avertissement)."""

    """
    La correction, quand il y en a une, deplace la CIBLE
    donnee au modele. Le modele et la couche robot ne sont
    pas touches : on peut donc mesurer avec et sans en
    changeant un seul argument.
    """

    cible = cibleCorrigee(correction, demande[0], demande[1])

    resultat = anglesPourPoint(modele, cible[0], cible[1])

    if resultat is None:
        return (None, "case hors du domaine angulaire du modele")

    j4, j5 = resultat

    vise, mesure = viserAngles(mc, base, j4, j5, j6)

    if mesure is None:
        return (None, "mouvement echoue")

    avertissement = ""

    if not vise:
        avertissement = ("la tete se repose a %+.2f / %+.2f deg de l'angle demande"
                         % (mesure[3] - j4, mesure[4] - j5))

    return ((j4, j5, mesure[3], mesure[4]), avertissement)


def balayer(mc, base, modele, j6, serie, cases, usage="apprentissage", correction=None):
    """Vise chaque case, releve l'impact saisi par l'operateur, et retourne le nombre de mesures."""

    mesures = 0

    for index, demande in enumerate(cases, 1):

        print()
        print("-----------------------------------------")
        print("Case %d / %d   ->   (%g, %g) carreaux"
              % (index, len(cases), demande[0], demande[1]))

        angles, avertissement = tirer(mc, base, modele, j6, demande, correction)

        if angles is None:
            print("  Case ignoree :", avertissement)
            continue

        print("  Angles : consigne J4 %+.2f / J5 %+.2f  ->  atteint %+.2f / %+.2f"
              % angles)

        if avertissement:
            print("  ATTENTION :", avertissement)

        reel = lireImpact(demande)

        if reel == "fin":
            print()
            print("Balayage interrompu par l'operateur.")
            break

        enregistrerMesure(serie, usage, index, demande, angles, reel)

        if reel is not None:
            mesures += 1

    return mesures


# =========================================================
# 10. LE BILAN
# =========================================================

def afficherCarteDesErreurs(points):
    """Affiche les erreurs mesurees case par case, en carreaux."""

    """
    Le verdict du script est statistique ; l'oeil, lui, voit
    tout de suite si les fleches d'un meme coin partent dans
    le meme sens. Si la carte est visiblement organisee alors
    que le verdict dit "aucune", c'est que le balayage est
    trop grossier pour la structure en question : refaire une
    seance avec PAS_BALAYAGE plus petit.
    """

    lignes = {}

    for point in points:
        lignes.setdefault(point["v"], []).append(point)

    print()
    print("Erreur mesuree, en carreaux (u puis v). Une case vue deux fois est moyennee.")
    print()

    for v in sorted(lignes):

        # Plusieurs visites de la meme case : on les moyenne.
        cases = {}

        for point in lignes[v]:
            cases.setdefault(point["u"], []).append(point)

        texte = []

        for u in sorted(cases):

            liste = cases[u]

            eu = sum(p["eu"] for p in liste) / len(liste)
            ev = sum(p["ev"] for p in liste) / len(liste)

            texte.append("%2g:%+.1f/%+.1f" % (u, eu, ev))

        print("  v=%-3g " % v + "  ".join(texte))


def afficherBilan(points, bruit, tableau, nom_retenu):
    """Affiche ce que les mesures disent, et ce que la correction peut ou ne peut pas y faire."""

    print()
    print("=========================================")
    print("BILAN DU BALAYAGE")
    print("=========================================")
    print("Cases mesurees :", len(points))

    brut = rmsResiduel(points, None)

    moyen_u = sum(p["eu"] for p in points) / len(points)
    moyen_v = sum(p["ev"] for p in points) / len(points)

    print()
    print("Erreur actuelle : rms %.2f carreaux (%.1f cm), biais %+.2f / %+.2f carreau"
          % (brut, brut * COTE_CARREAU, moyen_u, moyen_v))

    afficherCarteDesErreurs(points)

    # -----------------------------------------------------
    # Le plancher
    # -----------------------------------------------------

    if bruit is None:
        print()
        print("Aucune case n'a ete visee deux fois : le plancher de bruit est INCONNU.")
        print("Le verdict ci-dessous repose alors sur la seule validation croisee.")

    else:
        print()
        print("Plancher de bruit : %.2f carreaux (%.1f cm) par mesure, sur %d paire(s)."
              % (bruit["bruit_par_mesure"], bruit["bruit_par_mesure"] * COTE_CARREAU,
                 bruit["paires"]))
        print("Aucune correction ne peut descendre en dessous : c'est ce que la meme")
        print("case donne de different quand on la vise deux fois.")

        if bruit["bruit_par_mesure"] > 0.7 * brut:
            print()
            print("CE PLANCHER EXPLIQUE DEJA L'ESSENTIEL DE L'ERREUR.")
            print("Ce qui reste a corriger est mince : l'erreur n'est pas reproductible,")
            print("elle change d'un tir a l'autre. Chercher dans la mecanique et dans la")
            print("lecture du quadrillage, pas dans une carte de correction.")

    # -----------------------------------------------------
    # Les candidates
    # -----------------------------------------------------

    print()
    print("Correction        sur ses points   en validation croisee")
    print("-----------------------------------------------------------")

    for ligne in tableau:

        if ligne["ajuste"] is None and ligne["raison"]:
            print("  %-14s  %s" % (ligne["nom"], ligne["raison"]))
            continue

        croisee = "     --" if ligne["croisee"] is None else "%.2f carr" % ligne["croisee"]

        marque = "  <= retenue" if ligne["nom"] == nom_retenu else ""

        print("  %-14s  %.2f carr        %s%s"
              % (ligne["nom"], ligne["ajuste"], croisee, marque))

    print()
    print("La colonne qui compte est la validation croisee : chaque point y est predit")
    print("par une correction ajustee SANS lui. Une correction qui colle a ses propres")
    print("points mais decroche en validation croisee a appris le bruit.")

    # -----------------------------------------------------
    # Verdict
    # -----------------------------------------------------

    temoin = next((l["croisee"] for l in tableau if l["nom"] == "aucune"), None)
    retenue = next((l["croisee"] for l in tableau if l["nom"] == nom_retenu), None)

    print()

    if nom_retenu == "aucune":
        print("VERDICT : aucune correction retenue.")
        print()
        print("Aucune candidate ne gagne les %.0f %% exiges en validation croisee."
              % (GAIN_MINIMAL * 100))
        print("L'erreur mesuree n'est pas une deformation apprenable : c'est de la")
        print("dispersion. Une carte ajustee dessus rendrait le pointage MOINS bon")
        print("sur les cases qu'elle n'a pas vues.")
        print()
        print("Pistes, dans l'ordre : la case de depart du balayage est-elle bien la")
        print("meme qu'a la calibration ; la base a-t-elle bouge ; le quadrillage")
        print("a-t-il glisse ; l'impact est-il lu au meme endroit du spot a chaque fois.")

    else:
        gain = 100.0 * (1.0 - retenue / temoin) if temoin else 0.0

        print("VERDICT : correction '%s' retenue, %.0f %% de mieux en validation croisee."
              % (nom_retenu, gain))
        print("Erreur attendue apres correction : %.2f carreaux (%.1f cm)."
              % (retenue, retenue * COTE_CARREAU))

        # Franchement sous le plancher, pas a 1 % pres : l'estimation
        # du plancher a elle-meme son incertitude, sur si peu de paires.
        if bruit is not None and retenue < 0.8 * bruit["bruit_par_mesure"]:
            print()
            print("ATTENTION : cette valeur est SOUS le plancher de bruit. Elle est trop")
            print("belle pour etre vraie -- trop peu de paires de controle, sans doute.")

        print()
        print("Ce chiffre est une prevision. La phase de verification la confronte a")
        print("des tirs reels : c'est elle qui decide.")


# =========================================================
# 11. AJUSTEMENT SUR UNE SERIE
# =========================================================

def ajusterEtEnregistrer(serie, modele_serie=""):
    """Ajuste la correction sur les points d'une serie, affiche le bilan, l'enregistre et la retourne."""

    points = chargerMesuresBalayage(serie=serie, usage="apprentissage")

    if len(points) < 6:
        print()
        print("Seulement %d case(s) mesuree(s) dans la serie '%s' : il en faut au moins 6."
              % (len(points), serie))
        return None

    bruit = planchierDeBruit(points)

    nom, correction, tableau = choisirCorrection(points)

    afficherBilan(points, bruit, tableau, nom)

    retenue = next((l["croisee"] for l in tableau if l["nom"] == nom), None)

    infos = {
        "date": datetime.now().isoformat(timespec="seconds"),
        "serie": serie,
        "calibration": modele_serie,
        "nb_points": len(points),
        "rms_avant_carreaux": rmsResiduel(points, None),
        "rms_attendu_carreaux": retenue,
        "bruit_par_mesure_carreaux": None if bruit is None else bruit["bruit_par_mesure"],
        "cote_carreau": COTE_CARREAU,
    }

    if enregistrerCorrection(correction, infos):
        print()
        print("Correction enregistree :", FICHIER_CORRECTION)

        if correction is None:
            print("Elle porte le type 'aucune' : les autres scripts l'ignoreront.")

    return correction


# =========================================================
# 12. PROGRAMME
# =========================================================

def sansRobot():
    """Rejoue l'ajustement sur une serie deja mesuree, sans toucher au robot."""

    series = listerSeriesBalayage()

    if not series:
        print("Aucune serie dans", FICHIER_BALAYAGE)
        return 1

    print()
    print("Series disponibles :")

    for indice, nom in enumerate(series, 1):
        points = chargerMesuresBalayage(serie=nom, usage="apprentissage")
        print("  %d. %-28s %d case(s) mesuree(s)" % (indice, nom, len(points)))

    choix = lireTexte("Numero de la serie > ", "1")

    try:
        serie = series[int(choix) - 1]

    except (ValueError, IndexError):
        print("Choix invalide.")
        return 1

    ajusterEtEnregistrer(serie)

    return 0


def main():
    """Balaye la grille, apprend la correction, puis propose de la verifier."""

    print("=========================================")
    print("APPRENTISSAGE D'UNE CORRECTION DE POINTAGE")
    print("=========================================")
    print()
    print("SECURITE : pointeur rouge classe 2, laser de puissance DEBRANCHE.")

    # ---------------------------------------------------------
    # Calibration de depart, avant tout mouvement.
    # ---------------------------------------------------------

    serie_modele = choisirCalibration()

    if serie_modele is None:
        print("Aucune calibration selectionnee.")
        return 0

    modele = construireModeleSerie(serie_modele)

    if modele is None:
        return 1

    cases = casesDuBalayage(modele)
    controles = casesDeControle(cases)

    print()
    print("Calibration de depart :", serie_modele)
    print("Zone : u %.0f..%.0f, v %.0f..%.0f carreaux"
          % (modele["enveloppe"]["u_min"], modele["enveloppe"]["u_max"],
             modele["enveloppe"]["v_min"], modele["enveloppe"]["v_max"]))
    print("Balayage : %d cases au pas de %g carreaux, plus %d case(s) revisitee(s)"
          % (len(cases), PAS_BALAYAGE, len(controles)))
    print("Compter environ %d minutes." % max(1, (len(cases) + len(controles)) * 3 // 2))

    defaut = "Correction " + datetime.now().strftime("%d-%m %Hh%M")
    serie = lireTexte("Nom de cette seance > ", defaut).strip() or defaut

    # ---------------------------------------------------------
    # Robot.
    # ---------------------------------------------------------

    mc = connecterRobot()

    if mc is None:
        return 1

    if not verifierBase(mc, modele):
        return 1

    angles = lireAngles(mc)

    if angles is None:
        print("Lecture des angles impossible.")
        return 1

    base = angles[0:3]
    j6 = angles[5]

    try:
        if not allumerPointeur(mc):
            print("Le pointeur ne s'allume pas.")

        # -----------------------------------------------------
        # Phase 1 : le balayage
        # -----------------------------------------------------

        print()
        print("=========================================")
        print("BALAYAGE")
        print("=========================================")
        print("Pour chaque case, le robot vise et vous saisissez ou le spot est tombe.")
        print("Lire au demi-carreau : au noeud entier, la lecture vaut %g cm et masque"
              % COTE_CARREAU)
        print("exactement ce que l'on cherche a mesurer.")

        mesures = balayer(mc, base, modele, j6, serie, cases)

        # -----------------------------------------------------
        # Phase 2 : les cases de controle
        # -----------------------------------------------------

        if mesures and controles:

            print()
            print("=========================================")
            print("CASES DE CONTROLE")
            print("=========================================")
            print("Les memes cases qu'au debut, revisitees : l'ecart entre les deux")
            print("visites donne le plancher que la correction ne franchira pas.")

            mesures += balayer(mc, base, modele, j6, serie, controles)

        if not mesures:
            print()
            print("Aucune mesure relevee.")
            return 1

        # -----------------------------------------------------
        # Phase 3 : l'ajustement
        # -----------------------------------------------------

        correction = ajusterEtEnregistrer(serie, modele_serie=serie_modele)

        # -----------------------------------------------------
        # Phase 4 : la verification
        # -----------------------------------------------------

        if correction is None:
            return 0

        print()
        reponse = lireTexte("Verifier la correction sur quelques tirs ? (O/n) > ", "o")

        if reponse.lower().startswith("n"):
            print("Verification sautee. Le gain annonce reste une prevision.")
            return 0

        """
        Les cases de verification sont prises entre les
        noeuds du balayage : une correction se juge la ou
        elle n'a pas appris, pas sur ses propres points.
        """
        decalage = PAS_BALAYAGE / 2.0

        cases_verif = [(u + decalage, v + decalage) for (u, v) in cases]
        cases_verif = [c for c in cases_verif if dansEnveloppe(modele, c[0], c[1])]

        pas = max(1, len(cases_verif) // TIRS_VERIFICATION)
        cases_verif = cases_verif[::pas][:TIRS_VERIFICATION]

        print()
        print("=========================================")
        print("VERIFICATION")
        print("=========================================")
        print("%d tirs AVEC la correction, sur des cases jamais apprises."
              % len(cases_verif))

        balayer(mc, base, modele, j6, serie, cases_verif,
                usage="verification", correction=correction)

        points = chargerMesuresBalayage(serie=serie, usage="verification")

        if points:

            reste = rmsResiduel(points, None)

            print()
            print("Erreur mesuree APRES correction : %.2f carreaux (%.1f cm) sur %d tirs."
                  % (reste, reste * COTE_CARREAU, len(points)))
            print("A comparer a l'erreur d'avant, en tete du bilan. Si le chiffre n'a pas")
            print("bouge, la correction n'a rien appris d'utile : la supprimer.")

    finally:

        eteindrePointeur(mc)
        print()
        print("Pointeur eteint. Moteurs laisses sous tension.")

    return 0


# =========================================================
# 13. EXECUTION AUTONOME
# =========================================================

if __name__ == "__main__":

    try:
        if "--ajuster-seulement" in sys.argv:
            code_retour = sansRobot()

        else:
            code_retour = main()

    except KeyboardInterrupt:
        print()
        print("Programme interrompu par l'operateur.")
        code_retour = 1

    print()
    print("Programme termine.")
