"""
=========================================================
TESTS UNITAIRES - PU_Tete ET DERIVE DE LA BASE
=========================================================

Ces tests s'executent sans robot et sans la bibliotheque
pymycobot : le module pymycobot est remplace par un faux
module avant l'import, et le robot par un faux objet dont
J3 ne repond plus.

Lancement :
    python3 -m unittest discover -s test_unitaires -v
"""

import unittest
from unittest.mock import MagicMock
import json
import shutil
import sys
import os
import tempfile


## Remplacement de pymycobot par un faux module (absent hors Jetson)
faux_pymycobot = MagicMock()
sys.modules["pymycobot"] = faux_pymycobot
sys.modules["pymycobot.mycobot280"] = faux_pymycobot.mycobot280


## importation des modules a tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
import PU_Tete
import test_derive_base as derive


class FauxBras:
    """Bras dont J3 est bloque a -124 degres : il ignore toute consigne sur cet axe."""

    J3_BLOQUE = -124.0

    def __init__(self):
        self.pose = [2.0, 48.0, self.J3_BLOQUE, 75.0, 1.0, 1.0]

    def is_moving(self):
        return 0

    def stop(self):
        pass

    def send_angles(self, angles, vitesse):
        self.pose = list(angles)
        self.pose[2] = self.J3_BLOQUE
        return 1

    def get_angles(self):
        return list(self.pose)


class TestDecoupage(unittest.TestCase):
    """Tests de la separation base / tete."""

    def test_construire_pose(self):
        """La pose complete reprend la base puis la tete, dans l'ordre."""
        self.assertEqual(
            PU_Tete.construirePose([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        )

    def test_ecart_tete_ignore_la_base(self):
        """Un ecart enorme sur la base ne compte pas dans l'ecart de tete."""
        mesure = [0.0, 0.0, -124.0, 10.0, 0.0, 0.0]
        consigne = [0.0, 0.0, -144.0, 10.5, 0.0, 0.0]
        self.assertAlmostEqual(
            PU_Tete.ecartPartie(mesure, consigne, PU_Tete.INDICES_TETE), 0.5
        )

    def test_ecart_base_vu_separement(self):
        """Le meme ecart est bien visible quand on regarde la base."""
        mesure = [0.0, 0.0, -124.0, 10.0, 0.0, 0.0]
        consigne = [0.0, 0.0, -144.0, 10.5, 0.0, 0.0]
        self.assertAlmostEqual(
            PU_Tete.ecartPartie(mesure, consigne, PU_Tete.INDICES_BASE), 20.0
        )

    def test_lecture_par_partie(self):
        """lireBase et lireTete decoupent bien la pose lue."""
        bras = FauxBras()
        self.assertEqual(PU_Tete.lireBase(bras), [2.0, 48.0, -124.0])
        self.assertEqual(PU_Tete.lireTete(bras), [75.0, 1.0, 1.0])


class TestBougerTete(unittest.TestCase):
    """Tests du deplacement de tete sur un bras dont un axe de base est mort."""

    def setUp(self):
        """Bras a J3 bloque, et reference correspondant a sa pose reelle."""
        self.bras = FauxBras()
        self.reference = {
            "robot": "banc_essai",
            "date": "2026-09-08T10:00:00",
            "base": [2.0, 48.0, -124.0],
            "tete": [75.0, 1.0, 1.0],
        }

    def test_tete_atteinte_malgre_j3_mort(self):
        """La tete arrive a sa consigne alors que J3 n'a pas bouge."""
        arrive, mesure = PU_Tete.bougerTete(self.bras, [90.0, 20.0, 1.0], timeout=2)
        self.assertTrue(arrive)
        self.assertAlmostEqual(mesure[3], 90.0)
        self.assertAlmostEqual(mesure[2], FauxBras.J3_BLOQUE)

    def test_consigne_j3_prise_sur_la_mesure(self):
        """La consigne envoyee reprend le J3 mesure, jamais la valeur nominale du projet."""
        PU_Tete.bougerTete(self.bras, [90.0, 20.0, 1.0], timeout=2)
        self.assertAlmostEqual(self.bras.pose[2], FauxBras.J3_BLOQUE)
        self.assertNotAlmostEqual(self.bras.pose[2], PU_Tete.ANGLES_POSITION_INITIALE[2])

    def test_consigne_hors_limites_refusee(self):
        """Une consigne de tete hors limites est refusee, le bras ne bouge pas."""
        pose_avant = list(self.bras.pose)
        arrive, mesure = PU_Tete.bougerTete(self.bras, [200.0, 0.0, 0.0], timeout=2)
        self.assertFalse(arrive)
        self.assertIsNone(mesure)
        self.assertEqual(self.bras.pose, pose_avant)

    def test_base_conforme_a_sa_reference(self):
        """La base d'un bras qui n'a pas glisse est declaree conforme."""
        conforme, ecarts, base = PU_Tete.verifierBase(
            self.bras, reference=self.reference
        )
        self.assertTrue(conforme)
        self.assertEqual(base, [2.0, 48.0, -124.0])

    def test_base_hors_reference_detectee(self):
        """Une base qui a glisse de 5 degres est declaree non conforme."""
        self.bras.pose[2] = -119.0
        self.bras.J3_BLOQUE = -119.0
        conforme, ecarts, _ = PU_Tete.verifierBase(
            self.bras, reference=self.reference
        )
        self.assertFalse(conforme)
        self.assertAlmostEqual(ecarts[2], 5.0)


class TestReference(unittest.TestCase):
    """Tests de l'enregistrement de la reference de base."""

    def setUp(self):
        """Ecrit la reference dans un dossier temporaire."""
        self.dossier = tempfile.mkdtemp()
        self.fichier_initial = PU_Tete.FICHIER_REFERENCE
        self.dossier_initial = PU_Tete.DOSSIER_RESULTATS
        PU_Tete.DOSSIER_RESULTATS = self.dossier
        PU_Tete.FICHIER_REFERENCE = os.path.join(self.dossier, "reference_base.json")

    def tearDown(self):
        """Restaure les chemins du module."""
        PU_Tete.FICHIER_REFERENCE = self.fichier_initial
        PU_Tete.DOSSIER_RESULTATS = self.dossier_initial
        shutil.rmtree(self.dossier, ignore_errors=True)

    def test_reference_ecrite_et_relue(self):
        """La reference enregistree se relit avec son etiquette de robot."""
        PU_Tete.enregistrerReference(FauxBras(), "banc_essai", "J3 place a la main")
        relue = PU_Tete.chargerReference()

        self.assertEqual(relue["robot"], "banc_essai")
        self.assertEqual(relue["base"], [2.0, 48.0, -124.0])

    def test_absence_de_reference(self):
        """Sans fichier, le chargement retourne None au lieu de lever."""
        self.assertIsNone(PU_Tete.chargerReference())

    def test_verification_sans_reference(self):
        """La base est declaree non conforme quand aucune reference n'existe."""
        conforme, ecarts, base = PU_Tete.verifierBase(FauxBras())
        self.assertFalse(conforme)
        self.assertIsNone(ecarts)


class TestAnalyseDerive(unittest.TestCase):
    """Tests de la lecture de la derive de la base."""

    def test_base_stable(self):
        """Une base qui ne bouge pas est declaree stable."""
        historique = [[0.0, 0.0, 0.05]] * 9
        resume = derive.afficherRapport(historique, 9)
        self.assertEqual(resume["verdict"], "base stable")

    def test_tassement_initial(self):
        """Une base qui bouge puis se fige est distinguee du glissement."""
        historique = [[0.0, 0.0, 0.0], [0.0, 0.0, 1.5], [0.0, 0.0, 1.6],
                      [0.0, 0.0, 1.6], [0.0, 0.0, 1.6], [0.0, 0.0, 1.6]]
        resume = derive.afficherRapport(historique, 6)
        self.assertEqual(resume["verdict"], "tassement initial")

    def test_glissement_continu(self):
        """Une base qui derive encore a la fin du test est refusee."""
        historique = [[0.0, 0.0, float(i) * 0.5] for i in range(9)]
        resume = derive.afficherRapport(historique, 9)
        self.assertEqual(resume["verdict"], "glissement continu")

    def test_derive_signee(self):
        """La derive garde son signe, un glissement dans un sens n'annule pas l'autre."""
        derives = derive.derivesDepuisReference([2.0, 48.0, -119.0], [2.0, 48.0, -124.0])
        self.assertAlmostEqual(derives[2], 5.0)

    def test_poses_hors_limites_ecartees(self):
        """Une pose de tete qui sortirait des limites n'est pas parcourue."""
        poses = [[75.0, 1.0, 1.0], [200.0, 1.0, 1.0]]
        valides = derive.filtrerPosesValides(poses, [2.0, 48.0, -124.0])
        self.assertEqual(len(valides), 1)


if __name__ == "__main__":
    unittest.main()
