"""
=========================================================
TESTS UNITAIRES - APPRENTISSAGE DE LA CORRECTION
=========================================================

Tourne sans robot : pymycobot est remplace par un faux
module, et aucun test n'ouvre de port serie.

Le test central n'est pas celui de l'ajustement : c'est
celui du REFUS. Une correction qui apprend le bruit et
l'annonce comme un gain est plus nuisible que pas de
correction du tout, parce qu'elle est invisible. On
verifie donc d'abord que le script refuse de corriger du
bruit pur, et seulement ensuite qu'il retrouve les
deformations qu'on lui donne.

Lancement, depuis Bras/ModuleDeDeplacement/ :

    python3 -m unittest test_unitaires.tests_unitaire_correction -v
"""


# =========================================================
# 1. IMPORTS
# =========================================================

import unittest
from unittest.mock import MagicMock
import sys
import os
import math
import random
import json
import shutil
import tempfile

## Remplacement de pymycobot par un faux module (absent hors Jetson)
faux_pymycobot = MagicMock()
sys.modules["pymycobot"] = faux_pymycobot
sys.modules["pymycobot.mycobot280"] = faux_pymycobot.mycobot280

## importation du module a tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
import apprentissage_correction as correction


# =========================================================
# 2. JEUX DE POINTS SYNTHETIQUES
# =========================================================

def casesTest(pas=6):
    """Retourne les cases d'un balayage de 5 x 5 sur la zone reelle."""

    return [(float(u), float(v))
            for v in range(0, 28, pas)
            for u in range(0, 25, pas)]


def pointsSynthetiques(deformation, bruit=0.0, graine=1, repetitions=4):
    """Fabrique des mesures a partir d'une deformation connue, avec des cases revisitees."""

    """
    Les cases repetees sont indispensables : sans elles, le
    plancher de bruit est inconnu et la moitie du diagnostic
    ne s'exprime pas.
    """

    alea = random.Random(graine)

    cases = casesTest()

    if repetitions > 0:
        cases = cases + cases[::max(1, len(cases) // repetitions)]

    points = []

    for (u, v) in cases:

        eu, ev = deformation(u, v)

        points.append({
            "u": u,
            "v": v,
            "eu": eu + alea.gauss(0.0, bruit),
            "ev": ev + alea.gauss(0.0, bruit),
        })

    return points


BIAIS = lambda u, v: (0.8, -0.5)
AFFINE = lambda u, v: (0.03 * v + 0.3, -0.04 * u - 0.2)
QUADRATIQUE = lambda u, v: (0.004 * (u - 12.0) ** 2 - 0.4, 0.03 * v - 0.4)
RIEN = lambda u, v: (0.0, 0.0)


# =========================================================
# 3. LE REFUS
# =========================================================

class TestRefusDApprendreLeBruit(unittest.TestCase):
    """Sur une erreur non reproductible, le script doit ne RIEN retenir et le dire."""

    def test_bruit_pur_ne_donne_aucune_correction(self):
        """Des erreurs tirees au hasard ne doivent produire aucune correction."""

        for graine in range(5):

            points = pointsSynthetiques(RIEN, bruit=0.5, graine=graine)

            nom, corrigee, tableau = correction.choisirCorrection(points)

            self.assertEqual(nom, "aucune", "graine %d" % graine)
            self.assertIsNone(corrigee)

    def test_une_deformation_noyee_dans_le_bruit_est_refusee(self):
        """Une structure reelle mais plus petite que le bruit ne doit pas etre retenue."""

        points = pointsSynthetiques(QUADRATIQUE, bruit=1.2, graine=4)

        nom, corrigee, tableau = correction.choisirCorrection(points)

        self.assertEqual(nom, "aucune")

    def test_le_temoin_est_bien_l_erreur_telle_quelle(self):
        """La ligne 'aucune' doit porter l'erreur brute, pas None."""

        points = pointsSynthetiques(BIAIS, bruit=0.1)

        nom, corrigee, tableau = correction.choisirCorrection(points)

        temoin = next(l for l in tableau if l["nom"] == "aucune")

        self.assertIsNotNone(temoin["croisee"])
        self.assertAlmostEqual(temoin["croisee"],
                               correction.rmsResiduel(points, None), places=9)


# =========================================================
# 4. CE QUI DOIT ETRE RETROUVE
# =========================================================

class TestDeformationsRetrouvees(unittest.TestCase):
    """Une deformation reproductible doit etre retrouvee, et par la candidate la plus sobre."""

    def test_biais_constant(self):
        """Un decalage constant doit donner la correction constante."""

        points = pointsSynthetiques(BIAIS, bruit=0.05)

        nom, corrigee, tableau = correction.choisirCorrection(points)

        self.assertEqual(nom, "constante")

        erreur = correction.evaluerCorrection(corrigee, 10.0, 10.0)

        self.assertAlmostEqual(erreur[0], 0.8, places=1)
        self.assertAlmostEqual(erreur[1], -0.5, places=1)

    def test_deformation_affine(self):
        """Une grille tournee et mise a l'echelle doit donner la correction affine."""

        points = pointsSynthetiques(AFFINE, bruit=0.05)

        nom, corrigee, tableau = correction.choisirCorrection(points)

        self.assertEqual(nom, "affine")

    def test_deformation_quadratique(self):
        """Une deformation courbe ne doit pas se contenter d'une correction affine."""

        points = pointsSynthetiques(QUADRATIQUE, bruit=0.05)

        nom, corrigee, tableau = correction.choisirCorrection(points)

        self.assertIn(nom, ("quadratique", "carte"))

        rang = {l["nom"]: l["croisee"] for l in tableau}

        self.assertLess(rang[nom], rang["affine"])

    def test_ajustement_exact_sans_bruit(self):
        """Sans bruit, le polynome du bon degre doit reproduire la deformation exactement."""

        points = pointsSynthetiques(QUADRATIQUE, bruit=0.0)

        corrigee = correction.ajusterPolynome(points, 2)

        self.assertLess(correction.rmsResiduel(points, corrigee), 1e-9)


# =========================================================
# 5. APPLICATION
# =========================================================

class TestApplication(unittest.TestCase):
    """Corriger, c'est demander la case dont l'erreur ramene sur la bonne."""

    def test_la_cible_corrigee_annule_la_deformation(self):
        """Apres correction, le spot doit tomber sur la case demandee, y compris hors des points appris."""

        points = pointsSynthetiques(AFFINE, bruit=0.0)

        corrigee = correction.ajusterPolynome(points, 1)

        for (u, v) in [(3.0, 3.0), (9.5, 17.0), (21.0, 9.0), (15.0, 23.5)]:

            demande = correction.cibleCorrigee(corrigee, u, v)

            # Ou le spot tombe reellement quand on demande cette case.
            eu, ev = AFFINE(demande[0], demande[1])

            self.assertAlmostEqual(demande[0] + eu, u, places=3)
            self.assertAlmostEqual(demande[1] + ev, v, places=3)

    def test_sans_correction_la_cible_est_inchangee(self):
        """Une correction absente ne doit rien deplacer du tout."""

        self.assertEqual(correction.cibleCorrigee(None, 7.0, 11.0), (7.0, 11.0))

    def test_la_carte_reste_dans_la_plage_mesuree(self):
        """L'interpolation ne doit jamais sortir de l'intervalle des erreurs mesurees."""

        """
        Une ponderation par des poids positifs est une
        moyenne : elle ne peut pas depasser ses extremes.
        C'est ce qui rend la carte sure a extrapoler, la ou
        un polynome de degre 2 diverge hors de la zone.
        """

        points = pointsSynthetiques(QUADRATIQUE, bruit=0.3)

        carte = correction.ajusterCarte(points)

        mini = min(p["eu"] for p in points)
        maxi = max(p["eu"] for p in points)

        for (u, v) in [(-5.0, -5.0), (12.0, 14.0), (40.0, 40.0), (0.0, 27.0)]:

            eu, ev = correction.evaluerCorrection(carte, u, v)

            self.assertGreaterEqual(eu, mini - 1e-9)
            self.assertLessEqual(eu, maxi + 1e-9)


# =========================================================
# 6. LE PLANCHER DE BRUIT
# =========================================================

class TestPlancherDeBruit(unittest.TestCase):
    """Sans cases revisitees, on ne sait pas ce que vaut une mesure."""

    def test_retrouve_l_ecart_type_injecte(self):
        """Le plancher estime doit retrouver le bruit reellement injecte."""

        sigma = 0.4

        points = pointsSynthetiques(RIEN, bruit=sigma, graine=11, repetitions=25)

        bruit = correction.planchierDeBruit(points)

        self.assertIsNotNone(bruit)
        self.assertGreater(bruit["paires"], 10)

        # Peu de paires : on verifie l'ordre de grandeur, pas la troisieme decimale.
        self.assertLess(abs(bruit["bruit_par_mesure"] - sigma), 0.5 * sigma)

    def test_sans_case_repetee_le_plancher_est_inconnu(self):
        """Aucune case vue deux fois doit donner None, et non un chiffre invente."""

        points = pointsSynthetiques(RIEN, bruit=0.3, repetitions=0)

        self.assertIsNone(correction.planchierDeBruit(points))


# =========================================================
# 7. FICHIERS
# =========================================================

class TestFichiers(unittest.TestCase):
    """Une seance interrompue ne doit rien perdre, et rien ecrire sous un entete etranger."""

    def setUp(self):
        self.dossier = tempfile.mkdtemp()
        self.chemin = os.path.join(self.dossier, "balayage.csv")

    def tearDown(self):
        shutil.rmtree(self.dossier, ignore_errors=True)

    def test_aller_retour_par_le_fichier(self):
        """Les erreurs relues doivent etre celles qui ont ete ecrites."""

        correction.enregistrerMesure("s1", "apprentissage", 1, (6.0, 12.0),
                                     (10.0, 5.0, 9.8, 4.9), (6.5, 11.0),
                                     chemin=self.chemin)

        points = correction.chargerMesuresBalayage(chemin=self.chemin, serie="s1")

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0]["eu"], 0.5)
        self.assertAlmostEqual(points[0]["ev"], -1.0)

    def test_case_sautee_non_relue(self):
        """Une case sans impact saisi ne doit pas entrer dans l'ajustement."""

        correction.enregistrerMesure("s1", "apprentissage", 1, (6.0, 12.0),
                                     (10.0, 5.0, 9.8, 4.9), None, chemin=self.chemin)

        self.assertEqual(correction.chargerMesuresBalayage(chemin=self.chemin), [])

    def test_usages_separes(self):
        """L'apprentissage et la verification ne doivent jamais etre melanges."""

        correction.enregistrerMesure("s1", "apprentissage", 1, (0.0, 0.0),
                                     (1.0, 1.0, 1.0, 1.0), (1.0, 0.0), chemin=self.chemin)
        correction.enregistrerMesure("s1", "verification", 2, (3.0, 3.0),
                                     (1.0, 1.0, 1.0, 1.0), (3.0, 3.0), chemin=self.chemin)

        appris = correction.chargerMesuresBalayage(chemin=self.chemin, usage="apprentissage")
        verifie = correction.chargerMesuresBalayage(chemin=self.chemin, usage="verification")

        self.assertEqual(len(appris), 1)
        self.assertEqual(len(verifie), 1)

    def test_refus_d_ecrire_sous_un_entete_etranger(self):
        """Le piege des unites ne doit pas pouvoir se reproduire sur ce fichier."""

        with open(self.chemin, "w") as fichier:
            fichier.write("horodatage,serie,usage,index,u_demande_cm,v_demande_cm\n")

        ecrit = correction.enregistrerMesure("s1", "apprentissage", 1, (0.0, 0.0),
                                             (1.0, 1.0, 1.0, 1.0), (1.0, 0.0),
                                             chemin=self.chemin)

        self.assertFalse(ecrit)

    def test_aller_retour_de_la_correction(self):
        """La correction relue doit predire exactement ce que predisait celle qui a ete ecrite."""

        chemin = os.path.join(self.dossier, "correction.json")

        points = pointsSynthetiques(QUADRATIQUE, bruit=0.05)
        avant = correction.ajusterPolynome(points, 2)

        correction.enregistrerCorrection(avant, {"serie": "s1"}, chemin=chemin)
        apres = correction.chargerCorrection(chemin)

        self.assertIsNotNone(apres)

        for (u, v) in [(0.0, 0.0), (12.0, 14.0), (24.0, 27.0)]:
            self.assertEqual(correction.evaluerCorrection(avant, u, v),
                             correction.evaluerCorrection(apres, u, v))

    def test_une_correction_refusee_est_ignoree_a_la_relecture(self):
        """Le fichier ecrit quand rien n'est retenu ne doit pas etre pris pour une correction."""

        chemin = os.path.join(self.dossier, "correction.json")

        correction.enregistrerCorrection(None, {"serie": "s1"}, chemin=chemin)

        self.assertTrue(os.path.isfile(chemin))
        self.assertIsNone(correction.chargerCorrection(chemin))


if __name__ == "__main__":
    unittest.main(verbosity=2)
