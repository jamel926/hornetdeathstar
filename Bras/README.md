# Module de Déplacement et Contrôle du Bras

Ce dossier centralise les scripts Python permettant de piloter le bras robotique et de gérer les entrées/sorties (GPIO) liées à la sécurité et au déclenchement du laser.

## Architecture

* **`ModuleDeDeplacement/`**
    * `jog_move_finale.py` : Script principal gérant la cinématique et le déplacement du bras vers les coordonnées cibles fournies par la caméra.
    * `PU_Securite.py` : Boucle de contrôle vérifiant les limites physiques du bras et prévenant les collisions.
* **`ModuleGPIO/`**
    * `PU_GPIO.py` : Interface de bas niveau pour interagir avec broches GPIO (activation du laser, lecture de capteurs).

## Consignes de Sécurité
Avant d'exécuter `jog_move_finale.py`, assurez-vous toujours que l'espace autour du bras robotique est dégagé et que le module de sécurité (`PU_Securite.py`) est actif.
