from pymycobot.mycobot import MyCobot
import threading
import time
from math import sqrt

stop_event = threading.Event()    # Arrêt déclenché par le blocage du robot
alerte_event = threading.Event()  # La mise en mouvement du robot déclenche une alerte

mc = MyCobot('/dev/ttyTHS1', 1000000)

### Configuration de la sécurité (à modifier selon les besoins) ###
DISTANCE_SEUIL = 5.0  # Seuil de distance pour déclencher l'alerte


def arretSecurite():
    """ Fonction pour arrêter le robot en cas d'alerte """
    while not stop_event.is_set():
        if alerte_event.is_set():

            coord_1 = mc.get_coords()
            time.sleep(0.5 )
            coord_2 = mc.get_coords()

            diff_coord = [abs(coord_1[i] - coord_2[i]) for i in range(6)]
            dist_parcourue = sqrt(sum([diff ** 2 for diff in diff_coord]))
            
            if dist_parcourue < DISTANCE_SEUIL:
                print("Alerte détectée ! Arrêt du robot.")
                mc.stop()
                stop_event.set()
        time.sleep(0.1)
    mc.set_coords(0, 0, 0, 0, 0, 0)  # Ramener le robot à une position sûre
    print("Robot arrêté et ramené à une position sûre.")
    stop_event.clear()  


t = threading.Thread(target=arretSecurite, daemon=True)
t.start()

### Pour l'instant, pour faire connaitre le main, TO DO ###
def main():
    return 0



main()
t.join()
### Dans le main, ou une fonction faisant bouger le robot, on déclenche l'alerte ###
### Donc on fait alerte_event.set() avant de faire bouger le robot, et on peut faire alerte_event.clear() après pour réinitialiser l'alerte ###