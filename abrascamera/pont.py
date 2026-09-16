"""Contrat d'echange entre le pole image et le module bras."""

"""
===========================================================
LE PONT ENTRE LES DEUX POLES
===========================================================

Deux programmes tournent en parallele et ne se connaissent
pas :

    camera_frelon.py  ECRIT  une ligne par detection
    bras_frelon.py    LIT    la derniere ligne ecrite

Ce fichier est le seul endroit ou le format est defini. Si
le format change, il change ici et nulle part ailleurs.


Pourquoi un fichier et pas un socket :
-------------------------------------
Les deux programmes tournent sur la meme machine (l'ordi du
robot). Un fichier se lit avec `tail -f` pendant que ca
tourne, se rejoue apres coup, et permet de tester chaque
pole tout seul : on ecrit une ligne a la main et le bras
pointe. Un socket n'offre rien de tout ca.


Le format
---------
    horodatage,x_pct,y_pct,confiance
    1789458231.482,43.7,61.2,0.91

    horodatage  time.time(), epoch absolu, en secondes
    x_pct       0 a 100, proportion de la grille
    y_pct       0 a 100, proportion de la grille
    confiance   0.0 a 1.0, sortie du modele YOLO


ATTENTION : l'horodatage est ABSOLU
-----------------------------------
main.py ecrit `time.perf_counter() - start_time`, c'est a
dire un temps relatif a SON lancement. Cette valeur n'a
aucun sens pour un autre processus : elle repart de zero a
chaque demarrage de la camera, et deux processus lances a
des moments differents n'ont pas la meme origine.

Ici on ecrit time.time(). C'est ce qui permet au bras de
repondre a la seule question qui compte : est-ce que cette
position date de moins d'une seconde ?


ATTENTION : le fichier est ouvert en "a", pas en "w"
----------------------------------------------------
main.py l'ouvre en "w", donc il tronque le fichier a chaque
demarrage. Si le bras lisait a cet instant, il verrait le
fichier retrecir sous lui.

On ouvre en "a" et c'est le lanceur qui archive l'ancien
fichier. Le lecteur ci-dessous sait quand meme gerer une
troncature, parce qu'un jour quelqu'un lancera main.py par
erreur pendant que le bras tourne.
"""


import csv
import os
import time


# =========================================================
# 1. LE FORMAT
# =========================================================


DOSSIER = os.path.dirname(os.path.abspath(__file__))


CHEMIN_POSITIONS = os.path.join(DOSSIER, "positions_frelon.csv")


ENTETE = ["horodatage", "x_pct", "y_pct", "confiance"]


"""
Au dela de cet age, la position n'est plus une cible mais un
souvenir. Le bras s'immobilise et eteint le pointeur.

Une seconde, c'est dix fois la periode du mode flux (10 Hz).
Assez pour encaisser quelques images sans detection, trop
court pour continuer a viser un frelon parti depuis
longtemps.
"""
AGE_MAX = 1.0


# =========================================================
# 2. COTE CAMERA : ECRIRE
# =========================================================


def ouvrirJournal(chemin=CHEMIN_POSITIONS):
    """Ouvre le fichier de positions en ajout et retourne (fichier, writer)."""

    """
    L'entete n'est ecrite que si le fichier est vide ou
    absent : en mode ajout, la reecrire a chaque lancement
    planterait une ligne de texte au milieu des donnees, et
    le lecteur la prendrait pour une position illisible.
    """

    fichier_neuf = (
        not os.path.exists(chemin)
        or os.path.getsize(chemin) == 0
    )

    fichier = open(chemin, "a", newline="")

    writer = csv.writer(fichier)

    if fichier_neuf:
        writer.writerow(ENTETE)
        fichier.flush()

    return fichier, writer


def ecrireDetection(fichier, writer, x_pct, y_pct, confiance):
    """Ajoute une detection au journal et la rend lisible immediatement."""

    """
    Le flush est ce qui fait fonctionner tout le pont. Sans
    lui, Python garde les lignes dans son tampon et le bras
    ne voit rien pendant plusieurs secondes.
    """

    writer.writerow([
        "%.3f" % time.time(),
        "%.2f" % x_pct,
        "%.2f" % y_pct,
        "%.3f" % confiance,
    ])

    fichier.flush()


# =========================================================
# 3. COTE BRAS : LIRE LA DERNIERE POSITION
# =========================================================


class LecteurDernierePosition(object):
    """Suit le fichier de positions et retourne la derniere detection ecrite."""

    """
    Pourquoi ne pas relire tout le fichier a chaque fois :
    ------------------------------------------------------
    A 10 Hz pendant une demonstration de dix minutes, le
    fichier fait quelques milliers de lignes. Les relire
    toutes, dix fois par seconde, marche encore -- puis un
    jour ca ne marche plus, et personne ne comprend pourquoi
    le bras a ralenti.

    On garde donc un offset et on ne lit que ce qui est
    arrive depuis le dernier appel.

    chargerTrajectoireFichier() de suivi_frelon.py lit le
    fichier entier une fois et le ferme : c'est le bon outil
    pour rejouer une trajectoire enregistree, pas pour
    suivre un flux.
    """

    def __init__(self, chemin=CHEMIN_POSITIONS):

        self._chemin = chemin

        self._offset = 0

        self._reste = ""

        self._derniere = None

        self._modifie = None

    def _redemarrerSiTronque(self):
        """Repart du debut du fichier si celui-ci a ete recree sous nos pieds."""

        """
        Le cas a couvrir : quelqu'un relance main.py, qui
        ouvre le journal en mode "w" et le vide.

        Le test evident -- fichier plus court que notre
        offset -- ne suffit pas. Si la ligne reecrite fait
        exactement la meme longueur que celle qu'on avait
        deja lue, la taille finale est identique, l'offset
        tombe pile sur la fin du fichier, et on ne lit plus
        jamais rien. Le bras continuerait a viser la
        derniere position d'avant la troncature, sans aucun
        message.

        D'ou le second test : une taille inchangee alors que
        le fichier vient d'etre modifie ne peut pas arriver
        en fonctionnement normal, ou chaque ecriture
        l'allonge. C'est donc qu'il a ete recree.
        """

        try:
            etat = os.stat(self._chemin)

        except OSError:
            return

        recree = (
            etat.st_size < self._offset
            or (
                etat.st_size == self._offset
                and self._modifie is not None
                and etat.st_mtime != self._modifie
            )
        )

        if recree:
            self._offset = 0
            self._reste = ""

        self._modifie = etat.st_mtime

    def lire(self):
        """Retourne la derniere position connue (x_pct, y_pct, confiance, age) ou None."""

        self._redemarrerSiTronque()

        try:
            with open(self._chemin, "r") as fichier:
                fichier.seek(self._offset)
                bloc = fichier.read()
                self._offset = fichier.tell()

        except (OSError, IOError):
            return self._derniereAvecAge()

        if bloc:
            self._absorber(bloc)

        return self._derniereAvecAge()

    def _absorber(self, bloc):
        """Analyse les octets recus et retient la derniere ligne complete."""

        """
        Le tampon _reste porte la fin de ligne coupee entre
        deux lectures. La camera fait un flush par ligne,
        donc le cas est rare -- mais rare n'est pas jamais,
        et une ligne coupee en deux donne une position
        fausse, pas une erreur.
        """

        bloc = self._reste + bloc

        lignes = bloc.split("\n")

        self._reste = lignes.pop()

        for ligne in reversed(lignes):

            position = self._analyser(ligne)

            if position is not None:
                self._derniere = position
                return

    def _analyser(self, ligne):
        """Convertit une ligne du journal en (horodatage, x, y, confiance), ou None."""

        champs = ligne.strip().split(",")

        if len(champs) < 4:
            return None

        try:
            return (
                float(champs[0]),
                float(champs[1]),
                float(champs[2]),
                float(champs[3]),
            )

        except ValueError:
            # L'entete tombe ici, et c'est voulu.
            return None

    def _derniereAvecAge(self):
        """Ajoute l'age a la derniere position retenue."""

        if self._derniere is None:
            return None

        horodatage, x, y, confiance = self._derniere

        """
        L'horodatage est ecrit arrondi a la milliseconde, il
        peut donc etre imperceptiblement dans le futur. On
        rabat sur zero : un age negatif n'a pas de sens et
        piegerait le premier qui testerait son signe.
        """

        return (x, y, confiance, max(0.0, time.time() - horodatage))


# =========================================================
# 4. VERIFICATION RAPIDE
# =========================================================


def main():
    """Ecrit trois positions et verifie qu'on les relit correctement."""

    import tempfile

    chemin = os.path.join(tempfile.mkdtemp(), "test_pont.csv")

    print("Fichier de test :", chemin)
    print()

    fichier, writer = ouvrirJournal(chemin)

    lecteur = LecteurDernierePosition(chemin)

    print("Avant toute ecriture :", lecteur.lire())

    for x, y in [(10.0, 20.0), (50.0, 50.0), (90.5, 12.25)]:

        ecrireDetection(fichier, writer, x, y, 0.9)

        resultat = lecteur.lire()

        print("Ecrit (%5.2f, %5.2f)  ->  relu %s" % (
            x, y,
            "(%.2f, %.2f) conf %.2f age %.3f s" % resultat,
        ))

    fichier.close()

    print()
    print("Relecture sans nouvelle ecriture :", lecteur.lire() is not None)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
