# Modèle IA (YOLO)

Ce répertoire est dédié à l'entraînement et à l'évaluation du modèle de détection d'objets (YOLOv8) utilisé pour identifier les frelons.

## Fichiers Importants
* **`train.py`** : Script d'entraînement du modèle sur notre dataset personnalisé.
* **`TestYOLO.py`** : Script pour tester les performances du modèle sur des images ou un flux vidéo de validation.
* **`Model/weights/`** : Contient les poids du réseau de neurones.
    * `best.pt` : **À utiliser en production.** Les poids optimaux générés lors du dernier entraînement.
    * `last.pt` : Les poids de la toute dernière époque d'entraînement.

## Résultats
Les métriques d'entraînement et les labels de classification sont disponibles dans `results.csv` et `labels.jpg`.

## Dataset
Le dataset utilsé est le suivant:
    https://www.kaggle.com/datasets/abdelhakimkhaouiti/bee-and-asian-hornet-detection
(Créer son propre data set avec en fond le fond du demonstarteur/ruche sera plus intéressant)
**NB:** l'entrainement a été fait sur kaggle voici les 2 notebook utilisés:
    https://www.kaggle.com/code/lucasmaythe4th/frelonpart1
    https://www.kaggle.com/code/lucasmaythe4th/frelonpart2

