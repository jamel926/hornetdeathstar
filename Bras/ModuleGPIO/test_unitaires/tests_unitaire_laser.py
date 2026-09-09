"""
=========================================================
TESTS UNITAIRES - PU_Laser (COMMANDE DU LASER SUR G33)
=========================================================

Ces tests s'executent sans robot, sans laser et sans la
bibliotheque pymycobot : le robot est remplace par un faux
objet qui enregistre tout ce qu'on lui ecrit.

Ce qu'ils verifient est exactement ce qu'on ne peut pas
verifier a l'oeil sur le banc sans allumer le laser :
qu'aucun chemin de code n'allume sans initialisation ni
armement, et que l'extinction a bien lieu meme quand le
tir se passe mal.

Lancement :
    python3 -m unittest discover -s test_unitaires -v
"""

import unittest
from unittest.mock import MagicMock
import os
import sys


## Remplacement de pymycobot par un faux module (absent hors Jetson)
## PU_Laser importe connecterRobot depuis PU_PositionInitiale, qui lui
## importe pymycobot : sans ce faux module, les tests ne tourneraient
## que sur le robot.
faux_pymycobot = MagicMock()
sys.modules["pymycobot"] = faux_pymycobot
sys.modules["pymycobot.mycobot280"] = faux_pymycobot.mycobot280


## importation du module a tester
dossier_racine = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(dossier_racine)
import Bras.ModuleDeDeplacement.PU_Laser as PU_Laser


class FauxBras:
    """Faux robot qui note les appels GPIO au lieu de les envoyer sur le port serie."""

    def __init__(self, echoue_sur=None):
        self.ecritures = []          # liste de (broche, niveau)
        self.modes = []              # liste de (broche, mode)
        self.echoue_sur = echoue_sur # nom de methode qui doit lever une exception

    def set_pin_mode(self, broche, mode):
        if self.echoue_sur == "set_pin_mode":
            raise RuntimeError("faux echec de set_pin_mode")
        self.modes.append((broche, mode))
        return 1

    def set_digital_output(self, broche, niveau):
        if self.echoue_sur == "set_digital_output":
            raise RuntimeError("faux echec de set_digital_output")
        self.ecritures.append((broche, niveau))
        return 1

    def dernierNiveau(self):
        """Retourne le dernier niveau ecrit sur la broche, ou None si rien n'a ete ecrit."""
        if not self.ecritures:
            return None
        return self.ecritures[-1][1]


class BaseLaser(unittest.TestCase):
    """Remet le module dans son etat de depart et neutralise les attentes avant chaque test."""

    def setUp(self):

        PU_Laser._broche_configuree = False
        PU_Laser._laser_allume = False
        PU_Laser._laser_arme = False
        PU_Laser._instant_allumage = None

        """
        Les time.sleep() du module servent a laisser le MCU
        respirer. En test ils ne feraient que ralentir, on
        les remplace par une fonction qui note la duree
        demandee : c'est aussi comme ca qu'on verifie le
        bornage de la duree de tir.
        """

        self.durees_attendues = []
        self.vrai_sleep = PU_Laser.time.sleep
        PU_Laser.time.sleep = self.durees_attendues.append

        self.bras = FauxBras()

    def tearDown(self):
        PU_Laser.time.sleep = self.vrai_sleep

    def initialiser(self):
        """Raccourci : initialise et arme le laser sur le faux bras."""
        PU_Laser.initialiserLaser(self.bras)
        PU_Laser.armerLaser()


class TestImportSansEffet(BaseLaser):

    def test_import_n_ecrit_rien(self):
        """Importer le module ne doit configurer ni ecrire aucune broche."""
        self.assertEqual(self.bras.ecritures, [])
        self.assertEqual(self.bras.modes, [])

    def test_module_n_ouvre_pas_le_port(self):
        """Le module ne doit exposer aucun objet robot cree a l'import."""
        self.assertFalse(hasattr(PU_Laser, "mc"))

    def test_connecterRobot_est_visible(self):
        """connecterRobot doit exister au niveau du module, pas seulement dans une fonction."""
        self.assertTrue(callable(PU_Laser.connecterRobot))


class TestInitialisation(BaseLaser):

    def test_initialisation_met_la_broche_en_sortie(self):
        """initialiserLaser configure G33 en mode sortie."""
        PU_Laser.initialiserLaser(self.bras)
        self.assertIn((PU_Laser.BROCHE_LASER, PU_Laser.MODE_SORTIE), self.bras.modes)

    def test_initialisation_eteint_le_laser(self):
        """initialiserLaser laisse la broche au niveau eteint."""
        PU_Laser.initialiserLaser(self.bras)
        self.assertEqual(self.bras.dernierNiveau(), PU_Laser.NIVEAU_ETEINT)

    def test_initialisation_echoue_sans_robot(self):
        """Sans robot connecte, l'initialisation echoue au lieu de faire semblant."""
        self.assertFalse(PU_Laser.initialiserLaser(None))

    def test_initialisation_echoue_si_le_mode_refuse(self):
        """Si set_pin_mode leve, la broche n'est pas declaree prete."""
        bras = FauxBras(echoue_sur="set_pin_mode")
        self.assertFalse(PU_Laser.initialiserLaser(bras))
        self.assertFalse(PU_Laser.etatLaser()["configuree"])


class TestRefusDAllumage(BaseLaser):

    def test_refus_sans_initialisation(self):
        """Allumer sans initialiserLaser est refuse : ecrire sur une broche en entree ne fait rien."""
        PU_Laser.armerLaser()
        self.assertFalse(PU_Laser.allumerLaser(self.bras))
        self.assertEqual(self.bras.ecritures, [])

    def test_refus_sans_armement(self):
        """Allumer sans armerLaser est refuse."""
        PU_Laser.initialiserLaser(self.bras)
        self.assertFalse(PU_Laser.allumerLaser(self.bras))
        self.assertEqual(self.bras.dernierNiveau(), PU_Laser.NIVEAU_ETEINT)

    def test_desarmement_eteint(self):
        """Desarmer alors que le laser est allume l'eteint."""
        self.initialiser()
        PU_Laser.allumerLaser(self.bras)
        PU_Laser.desarmerLaser(self.bras)
        self.assertEqual(self.bras.dernierNiveau(), PU_Laser.NIVEAU_ETEINT)
        self.assertFalse(PU_Laser.etatLaser()["allume"])


class TestAllumageExtinction(BaseLaser):

    def test_allumage_ecrit_le_bon_niveau_sur_la_bonne_broche(self):
        """Allumer ecrit NIVEAU_ALLUME sur G33."""
        self.initialiser()
        self.assertTrue(PU_Laser.allumerLaser(self.bras))
        self.assertEqual(self.bras.ecritures[-1], (PU_Laser.BROCHE_LASER, PU_Laser.NIVEAU_ALLUME))

    def test_extinction_ecrit_le_niveau_eteint(self):
        """Eteindre ecrit NIVEAU_ETEINT sur G33."""
        self.initialiser()
        PU_Laser.allumerLaser(self.bras)
        self.assertTrue(PU_Laser.eteindreLaser(self.bras))
        self.assertEqual(self.bras.ecritures[-1], (PU_Laser.BROCHE_LASER, PU_Laser.NIVEAU_ETEINT))

    def test_extinction_toujours_autorisee(self):
        """Eteindre fonctionne meme sans initialisation ni armement : une extinction ne se refuse pas."""
        self.assertTrue(PU_Laser.eteindreLaser(self.bras))
        self.assertEqual(self.bras.dernierNiveau(), PU_Laser.NIVEAU_ETEINT)

    def test_les_deux_niveaux_sont_distincts(self):
        """Allume et eteint ne peuvent pas etre le meme niveau logique."""
        self.assertNotEqual(PU_Laser.NIVEAU_ALLUME, PU_Laser.NIVEAU_ETEINT)


class TestTir(BaseLaser):

    def test_tir_eteint_a_la_fin(self):
        """Un tir se termine toujours par une extinction."""
        self.initialiser()
        self.assertTrue(PU_Laser.tirer(self.bras, 1.0))
        self.assertEqual(self.bras.ecritures[-1], (PU_Laser.BROCHE_LASER, PU_Laser.NIVEAU_ETEINT))

    def test_duree_bornee(self):
        """Une duree superieure au maximum est ramenee au maximum, pas refusee."""
        self.initialiser()
        PU_Laser.tirer(self.bras, PU_Laser.DUREE_TIR_MAX * 10)
        self.assertIn(PU_Laser.DUREE_TIR_MAX, self.durees_attendues)
        self.assertNotIn(PU_Laser.DUREE_TIR_MAX * 10, self.durees_attendues)

    def test_duree_negative_refusee(self):
        """Une duree nulle ou negative n'allume rien."""
        self.initialiser()
        self.assertFalse(PU_Laser.tirer(self.bras, 0))
        self.assertEqual(self.bras.dernierNiveau(), PU_Laser.NIVEAU_ETEINT)

    def test_extinction_meme_si_l_attente_est_interrompue(self):
        """Une interruption pendant le tir n'empeche pas l'extinction."""

        self.initialiser()

        def sleepInterrompu(duree):
            raise KeyboardInterrupt()

        PU_Laser.time.sleep = sleepInterrompu

        with self.assertRaises(KeyboardInterrupt):
            PU_Laser.tirer(self.bras, 1.0)

        self.assertEqual(self.bras.ecritures[-1], (PU_Laser.BROCHE_LASER, PU_Laser.NIVEAU_ETEINT))

    def test_tir_refuse_sans_armement(self):
        """Tirer sans armement n'envoie aucune commande d'allumage."""
        PU_Laser.initialiserLaser(self.bras)
        avant = len(self.bras.ecritures)
        self.assertFalse(PU_Laser.tirer(self.bras, 1.0))
        self.assertEqual(len(self.bras.ecritures), avant)

    def test_extinction_meme_si_l_allumage_est_interrompu(self):
        """Une interruption entre l'ecriture d'allumage et le retour de allumerLaser passe quand meme par l'extinction."""

        self.initialiser()

        def sleepInterrompu(duree):
            raise KeyboardInterrupt()

        PU_Laser.time.sleep = sleepInterrompu

        with self.assertRaises(KeyboardInterrupt):
            PU_Laser.tirer(self.bras, 1.0)

        self.assertEqual(self.bras.ecritures[-1], (PU_Laser.BROCHE_LASER, PU_Laser.NIVEAU_ETEINT))


class TestEcritureBasNiveau(BaseLaser):

    def test_ecriture_sans_robot(self):
        """Ecrire sans robot connecte retourne False au lieu de lever."""
        self.assertFalse(PU_Laser.ecrireBroche(None, 1))

    def test_niveau_invalide(self):
        """Un niveau autre que 0 ou 1 est refuse avant l'envoi."""
        self.assertFalse(PU_Laser.ecrireBroche(self.bras, 2))
        self.assertEqual(self.bras.ecritures, [])

    def test_echec_de_la_liaison(self):
        """Une exception de pymycobot est rattrapee et signalee par False."""
        bras = FauxBras(echoue_sur="set_digital_output")
        self.assertFalse(PU_Laser.ecrireBroche(bras, 1))


class TestEtat(BaseLaser):

    def test_etat_apres_allumage(self):
        """etatLaser reflete la derniere commande envoyee."""
        self.initialiser()
        PU_Laser.allumerLaser(self.bras)
        etat = PU_Laser.etatLaser()
        self.assertTrue(etat["allume"])
        self.assertTrue(etat["armee"])
        self.assertTrue(etat["configuree"])
        self.assertEqual(etat["broche"], PU_Laser.BROCHE_LASER)

    def test_broche_conforme_a_pymycobot(self):
        """La broche doit faire partie de celles que pymycobot accepte pour la tete du bras."""
        self.assertIn(PU_Laser.BROCHE_LASER, [19, 22, 23, 33, 39])


if __name__ == "__main__":
    unittest.main(verbosity=2)
