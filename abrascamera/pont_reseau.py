"""Contrat d'echange entre le PC (camera) et la Jetson (bras), par le reseau."""

"""
===========================================================
LE PONT RESEAU ENTRE LES DEUX POLES
===========================================================

Meme role que pont.py, mais quand la camera et le bras ne
tournent plus sur la meme machine :

    PC      camera_frelon_reseau.py  ENVOIE  une position par detection
    Jetson  bras_frelon_reseau.py    RECOIT  la derniere position

Les deux machines sont reliees par un cable reseau direct.
pont.py reste valable quand tout tourne sur la Jetson : ce
fichier ne le remplace pas, il s'ajoute a cote.


Pourquoi UDP et pas TCP :
-------------------------
Le bras n'a besoin que de la DERNIERE position. Une position
perdue n'a aucune importance : la suivante arrive quelques
dizaines de millisecondes plus tard. TCP, lui, retransmet
les paquets perdus et bloque les suivants en attendant --
on recevrait en retard des positions deja perimees.

Avec UDP, si le cable est debranche, la camera continue
d'envoyer dans le vide et le bras ne recoit plus rien : il
s'immobilise au bout de pont.AGE_MAX et eteint le pointeur.
Aucun des deux programmes ne plante.


Le format
---------
Une position = un datagramme UDP, en texte :

    x_pct,y_pct,confiance
    43.70,61.20,0.910

Memes unites que pont.py. Pas d'horodatage : voir ci-dessous.


ATTENTION : l'age est mesure a la RECEPTION
-------------------------------------------
pont.py ecrit time.time() de la camera et le bras le compare
a SON time.time(). Sur une seule machine, c'est la meme
horloge. Sur deux machines, rien ne garantit que les
horloges du PC et de la Jetson soient d'accord : une seconde
d'ecart suffit pour que le bras juge TOUTES les positions
perimees, ou vise une position vieille sans le savoir.

On ne transmet donc pas d'horodatage. Le bras date lui-meme
chaque position au moment ou il la lit, avec sa propre
horloge. L'erreur commise est au plus une periode de la
boucle du bras (0,1 s a 10 Hz), bien sous AGE_MAX (1 s).
"""


import socket
import time

import pont


# =========================================================
# 1. LE FORMAT ET LE RESEAU
# =========================================================


"""
Adresses du cable direct PC <-> Jetson. Elles sont choisies
hors de 192.168.1.x pour ne jamais entrer en conflit avec le
reseau Wi-Fi d'une des deux machines.

Le PC distribue l'adresse de la Jetson (connexion partagee,
voir README_reseau.md). Elle n'est donc pas fixee par nous :
10.10.0.103 est celle obtenue au premier branchement, et le
serveur DHCP la redonne en principe a la meme carte. Si elle
change, la passer avec --ip-bras.
"""
IP_PC = "10.10.0.1"

IP_JETSON = "10.10.0.103"


"""
Port UDP d'ecoute du bras. Quelconque au-dessus de 1024,
il doit seulement etre le meme des deux cotes.
"""
PORT = 5005


"""
Taille maximale d'un datagramme lu. Une position fait une
vingtaine d'octets : 512 laisse une marge large.
"""
TAILLE_MAX = 512


AGE_MAX = pont.AGE_MAX


# =========================================================
# 2. COTE CAMERA (PC) : ENVOYER
# =========================================================


class EmetteurPositions(object):
    """Envoie chaque detection au bras, sans attendre de reponse."""

    def __init__(self, ip=IP_JETSON, port=PORT):

        self._destination = (ip, port)

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.erreurs = 0

    def envoyer(self, x_pct, y_pct, confiance):
        """Envoie une position. Ne leve jamais d'exception."""

        """
        Si le cable est debranche, sendto() peut echouer
        (reseau injoignable). On compte l'erreur et on
        continue : la camera ne doit pas s'arreter parce
        que le bras est absent.
        """

        message = "%.2f,%.2f,%.3f" % (x_pct, y_pct, confiance)

        try:
            self._socket.sendto(message.encode("ascii"), self._destination)

        except OSError:
            self.erreurs += 1

    def fermer(self):
        """Libere le socket."""

        self._socket.close()


# =========================================================
# 3. COTE BRAS (JETSON) : RECEVOIR LA DERNIERE POSITION
# =========================================================


class RecepteurDernierePosition(object):
    """Ecoute le reseau et retourne la derniere position recue."""

    """
    S'utilise exactement comme pont.LecteurDernierePosition :
    lire() retourne (x_pct, y_pct, confiance, age) ou None.
    C'est ce qui permet a bras_frelon_reseau.py de reutiliser
    la boucle de pointage de bras_frelon.py sans la recopier.

    Le socket est NON BLOQUANT. La camera envoie plus vite
    que le bras ne lit (jusqu'a 30 images par seconde contre
    10 consignes) : a chaque lire(), on vide tout ce qui est
    arrive et on ne garde que le dernier. Sans ca, les
    positions s'empileraient dans le tampon du systeme et le
    bras viserait avec un retard qui grandit sans cesse.
    """

    def __init__(self, port=PORT, ip_ecoute="0.0.0.0"):

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._socket.bind((ip_ecoute, port))

        self._socket.setblocking(False)

        self._derniere = None

        self._reception = None

        self.recues = 0

        self.illisibles = 0

    def lire(self):
        """Retourne la derniere position connue (x_pct, y_pct, confiance, age) ou None."""

        while True:

            try:
                donnees, _ = self._socket.recvfrom(TAILLE_MAX)

            except BlockingIOError:
                break

            except OSError:
                break

            position = self._analyser(donnees)

            if position is None:
                self.illisibles += 1
                continue

            self._derniere = position

            self._reception = time.monotonic()

            self.recues += 1

        if self._derniere is None:
            return None

        x, y, confiance = self._derniere

        """
        time.monotonic() et pas time.time() : l'horloge
        systeme peut etre recalee pendant la seance (NTP, ou
        la Jetson qui retrouve l'heure en se connectant), et
        un saut d'horloge fausserait l'age.
        """
        age = time.monotonic() - self._reception

        return (x, y, confiance, age)

    def oublier(self):
        """Jette tout ce qui a ete recu jusqu'ici, y compris la derniere position."""

        """
        A appeler juste avant de commencer a pointer. Un
        simple lire() ne suffit pas : il viderait le tampon
        mais daterait la derniere position de MAINTENANT,
        puisque l'age est mesure a la lecture. Une position
        vieille de 30 secondes passerait pour fraiche.
        """

        self.lire()

        self._derniere = None

        self._reception = None

    def _analyser(self, donnees):
        """Convertit un datagramme en (x, y, confiance), ou None."""

        try:
            champs = donnees.decode("ascii").strip().split(",")

        except UnicodeDecodeError:
            return None

        if len(champs) != 3:
            return None

        try:
            return (float(champs[0]), float(champs[1]), float(champs[2]))

        except ValueError:
            return None

    def fermer(self):
        """Libere le socket."""

        self._socket.close()


# =========================================================
# 4. VERIFICATION
# =========================================================


def main():
    """Sans argument : test local. --ecouter : affiche ce qui arrive du PC."""

    """
    Deux usages :

    1. Sur une seule machine, sans reseau :
           python abrascamera/pont_reseau.py

    2. Sur la Jetson, pour verifier que le cable marche
       AVANT de lancer le bras :
           python abrascamera/pont_reseau.py --ecouter
       puis lancer camera_frelon_reseau.py sur le PC.
    """

    import sys

    if "--ecouter" in sys.argv:
        return ecouter()

    recepteur = RecepteurDernierePosition(PORT, "127.0.0.1")

    emetteur = EmetteurPositions("127.0.0.1", PORT)

    print("Avant tout envoi :", recepteur.lire())

    for x, y in [(10.0, 20.0), (50.0, 50.0), (90.5, 12.25)]:

        emetteur.envoyer(x, y, 0.9)

        time.sleep(0.01)

        print("Envoye (%5.2f, %5.2f)  ->  recu %s" % (
            x, y,
            "(%.2f, %.2f) conf %.2f age %.3f s" % recepteur.lire(),
        ))

    """
    Trois envois d'affilee sans lecture : on doit relire le
    DERNIER, pas le premier.
    """
    for x in (1.0, 2.0, 3.0):
        emetteur.envoyer(x, x, 0.5)

    time.sleep(0.01)

    print("Rafale 1, 2, 3        ->  recu x = %.2f (attendu 3.00)" % recepteur.lire()[0])

    time.sleep(0.2)

    print("Sans envoi, age       ->  %.2f s (doit grandir)" % recepteur.lire()[3])

    emetteur.fermer()

    recepteur.fermer()

    return 0


def ecouter():
    """Affiche les positions recues, deux fois par seconde."""

    recepteur = RecepteurDernierePosition(PORT)

    print("Ecoute sur le port UDP %d. Ctrl+C pour arreter." % PORT)

    try:
        while True:

            time.sleep(0.5)

            position = recepteur.lire()

            if position is None:
                print("  rien recu pour l'instant")
                continue

            print("  %6.2f %%  %6.2f %%   confiance %.2f   age %.2f s   (%d recues)" % (
                position + (recepteur.recues,)
            ))

    except KeyboardInterrupt:
        print()

    recepteur.fermer()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
