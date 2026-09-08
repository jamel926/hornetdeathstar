"""
=========================================================
TESTS UNITAIRES - REPETABILITE ET DIAGNOSTIC J3
=========================================================

Ces tests s'executent sans robot et sans la bibliotheque
pymycobot : le module pymycobot est remplace par un faux
module avant l'import des scripts testes.

Seules les fonctions de calcul sont testees ici. Tout ce
qui parle au robot demande du materiel et se verifie au
labo.

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
import test_repetabilite_initiale as repetabilite
import test_diagnostic_J3 as diagnostic
import PU_Journal


class TestStatistiques(unittest.TestCase):
    """Tests des calculs de moyenne, d'etendue et de biais."""

    def setUp(self):
        """Trois poses ou seul J3 varie, de 2 degres au total."""
        self.poses = [
            [0.0, 10.0, -100.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, -101.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, -102.0, 0.0, 0.0, 0.0],
        ]

    def test_moyenne(self):
        """La moyenne de J3 sur les trois poses vaut -101."""
        moyennes = repetabilite.moyenneParArticulation(self.poses)
        self.assertAlmostEqual(moyennes[2], -101.0)

    def test_etendue_axe_variable(self):
        """L'etendue de J3 vaut 2 degres."""
        etendues = repetabilite.etendueParArticulation(self.poses)
        self.assertAlmostEqual(etendues[2], 2.0)

    def test_etendue_axe_fixe(self):
        """Un axe qui ne bouge pas a une etendue nulle."""
        etendues = repetabilite.etendueParArticulation(self.poses)
        self.assertAlmostEqual(etendues[1], 0.0)

    def test_biais_signe(self):
        """Le biais est signe : une mesure sous la consigne donne un biais negatif."""
        cible = [0.0, 10.0, -80.0, 0.0, 0.0, 0.0]
        biais = repetabilite.biaisParArticulation(self.poses, cible)
        self.assertAlmostEqual(biais[2], -21.0)

    def test_pose_unique(self):
        """Une seule pose donne une etendue nulle partout."""
        etendues = repetabilite.etendueParArticulation([self.poses[0]])
        self.assertEqual(etendues, [0.0] * 6)


class TestIndiceMaximum(unittest.TestCase):
    """Tests de la designation de l'articulation la plus fautive."""

    def test_valeur_absolue(self):
        """Un ecart negatif de grande amplitude l'emporte sur un petit ecart positif."""
        self.assertEqual(repetabilite.indiceMaximum([0.5, -20.0, 1.0]), 1)

    def test_premier_en_cas_egalite(self):
        """En cas d'egalite, le premier indice est retenu."""
        self.assertEqual(repetabilite.indiceMaximum([3.0, 3.0]), 0)


class TestPoseDetour(unittest.TestCase):
    """Tests de la pose de detour du mode automatique."""

    def test_detour_valide(self):
        """Le detour calcule depuis la pose initiale du projet respecte les limites."""
        pose = repetabilite.calculerPoseDetour()
        self.assertIsNotNone(pose)
        self.assertTrue(repetabilite.verifierAngles(pose))

    def test_detour_applique_le_decalage(self):
        """Le detour est bien la pose de depart decalee terme a terme."""
        depart = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        decalage = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        self.assertEqual(
            repetabilite.calculerPoseDetour(depart, decalage),
            decalage
        )

    def test_detour_hors_limites_refuse(self):
        """Un detour qui sortirait des limites est refuse, pas rabote."""
        depart = [0.0, 130.0, 0.0, 0.0, 0.0, 0.0]
        decalage = [0.0, 30.0, 0.0, 0.0, 0.0, 0.0]
        self.assertIsNone(repetabilite.calculerPoseDetour(depart, decalage))


class TestAnalyseBalayage(unittest.TestCase):
    """Tests de la lecture automatique du balayage de J3."""

    def test_decalage_constant(self):
        """Un ecart identique sur toute la plage est vu comme un decalage, sans saturation."""
        resultats = [(-60.0, -40.0), (-80.0, -60.0), (-100.0, -80.0), (-120.0, -100.0)]
        ecart_moyen, variation, sature = diagnostic.analyserBalayage(resultats)
        self.assertAlmostEqual(ecart_moyen, 20.0)
        self.assertAlmostEqual(variation, 0.0)
        self.assertFalse(sature)

    def test_saturation(self):
        """Une mesure qui cesse de suivre la consigne est vue comme une butee."""
        resultats = [(-60.0, -60.0), (-80.0, -80.0), (-100.0, -100.0),
                     (-120.0, -102.0), (-140.0, -103.0)]
        _, _, sature = diagnostic.analyserBalayage(resultats)
        self.assertTrue(sature)

    def test_suivi_fidele(self):
        """Un bras qui suit sa consigne ne declenche ni saturation ni ecart."""
        resultats = [(-60.0, -60.0), (-80.0, -80.0), (-100.0, -100.0), (-120.0, -120.0)]
        ecart_moyen, variation, sature = diagnostic.analyserBalayage(resultats)
        self.assertAlmostEqual(ecart_moyen, 0.0)
        self.assertAlmostEqual(variation, 0.0)
        self.assertFalse(sature)


class TestPoseAvecJ3(unittest.TestCase):
    """Tests de la construction des consignes du diagnostic."""

    def test_remplace_j3(self):
        """Seul J3 est remplace quand J2 n'est pas donne."""
        pose = diagnostic.poseAvecJ3(-130.0)
        self.assertAlmostEqual(pose[2], -130.0)
        self.assertEqual(pose[:2], list(diagnostic.POSE_ESSAI[:2]))

    def test_remplace_j2_et_j3(self):
        """J2 et J3 sont remplaces quand les deux sont donnes."""
        pose = diagnostic.poseAvecJ3(-130.0, 48.0)
        self.assertAlmostEqual(pose[1], 48.0)
        self.assertAlmostEqual(pose[2], -130.0)

    def test_ne_modifie_pas_la_pose_dessai(self):
        """La constante POSE_ESSAI n'est pas modifiee par les appels successifs."""
        avant = list(diagnostic.POSE_ESSAI)
        diagnostic.poseAvecJ3(-130.0, 48.0)
        self.assertEqual(list(diagnostic.POSE_ESSAI), avant)


class TestJournal(unittest.TestCase):
    """Tests du journal de session transmis pour diagnostic."""

    def setUp(self):
        """Ecrit les journaux dans un dossier temporaire, pas dans resultats/."""
        self.dossier = tempfile.mkdtemp()
        self.dossier_initial = PU_Journal.DOSSIER_RESULTATS
        PU_Journal.DOSSIER_RESULTATS = self.dossier

    def tearDown(self):
        """Restaure le dossier de resultats du module."""
        PU_Journal.DOSSIER_RESULTATS = self.dossier_initial
        shutil.rmtree(self.dossier, ignore_errors=True)

    def test_cles_du_journal(self):
        """Le journal porte bien une entree constantes distincte."""
        journal = PU_Journal.ouvrirJournal("essai", None, {"TOLERANCE": 2.0})
        self.assertEqual(journal["constantes"]["TOLERANCE"], 2.0)
        self.assertIn("environnement", journal)
        self.assertIn("evenements", journal)

    def test_ecart_calcule(self):
        """L'ecart est calcule article par articulation a partir de la consigne et de la mesure."""
        journal = PU_Journal.ouvrirJournal("essai")
        PU_Journal.ajouterMesure(
            journal, "balayage", "J3",
            [0.0, 0.0, -144.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -124.0, 0.0, 0.0, 0.0]
        )
        self.assertAlmostEqual(journal["evenements"][0]["ecart"][2], 20.0)

    def test_lectures_brutes_conservees(self):
        """Les lectures individuelles sont gardees, pas seulement leur moyenne."""
        journal = PU_Journal.ouvrirJournal("essai")
        lectures = [[0.0] * 6, [0.1] * 6]
        PU_Journal.ajouterMesure(journal, "essai", "1", None, [0.05] * 6, lectures)
        self.assertEqual(len(journal["evenements"][0]["lectures_brutes"]), 2)

    def test_mesure_sans_consigne(self):
        """Une lecture sans consigne est acceptee et ne porte pas d'ecart."""
        journal = PU_Journal.ouvrirJournal("essai")
        PU_Journal.ajouterMesure(journal, "communication", "repos", None, [0.0] * 6)
        self.assertNotIn("ecart", journal["evenements"][0])

    def test_observation_vide_ignoree(self):
        """Une observation vide n'encombre pas le journal."""
        journal = PU_Journal.ouvrirJournal("essai")
        PU_Journal.ajouterObservation(journal, "   ")
        PU_Journal.ajouterObservation(journal, "le bras claque en fin de course")
        self.assertEqual(len(journal["observations"]), 1)

    def test_fichier_relisible(self):
        """Le journal ferme est un JSON relisible contenant le resume."""
        journal = PU_Journal.ouvrirJournal("essai")
        chemin = PU_Journal.fermerJournal(journal, {"e1_valide": False})

        self.assertIsNotNone(chemin)

        with open(chemin) as f:
            relu = json.load(f)

        self.assertEqual(relu["resume"]["e1_valide"], False)
        self.assertIsNotNone(relu["fin"])

    def test_journal_absent_tolere(self):
        """Les fonctions d'ecriture acceptent un journal None sans lever."""
        PU_Journal.ajouterMesure(None, "essai", "1", None, [0.0] * 6)
        PU_Journal.ajouterObservation(None, "texte")
        self.assertIsNone(PU_Journal.fermerJournal(None))


class TestRapport(unittest.TestCase):
    """Tests du resume produit par le rapport de repetabilite."""

    def test_biais_repere_sans_dispersion(self):
        """Un bras decale mais regulier est declare repetable et non juste."""
        cible = [0.0, 0.0, -144.0, 0.0, 0.0, 0.0]
        poses = [
            [0.0, 0.0, -124.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -124.3, 0.0, 0.0, 0.0],
            [0.0, 0.0, -123.8, 0.0, 0.0, 0.0],
        ]
        resume = repetabilite.afficherRapport(poses, cible)

        self.assertTrue(resume["repetable"])
        self.assertFalse(resume["juste"])
        self.assertEqual(resume["articulation_biais_max"], "J3")
        self.assertAlmostEqual(resume["biais_max"], 19.967, places=2)


if __name__ == "__main__":
    unittest.main()
