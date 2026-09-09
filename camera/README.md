# Module Vision et Détection

Ce dossier contient le cœur du système de repérage du Hornet Death Star. Il combine la capture de flux vidéo avec la reconnaissance d'objets en temps réel via stéréovision.

## Fonctionnement Général
Le système s'articule autour de deux composants majeurs :
1.  **Le Module Caméra (`ModuleCam/ModuleCamera.py`) :** Gère l'acquisition des flux vidéo via OpenCV pour les deux caméras, le calibrage stéréo et la transmission des frames.
2.  **L'Intelligence Artificielle (`ReconnaissanceObjet/ModuleAI.py`) :** Analyse les frames pour détecter les frelons et calcule leurs coordonnées spatiales 3D par triangulation.

Pour les détails spécifiques sur l'entraînement du modèle IA, consultez le sous-dossier [IA/README.md](./IA/README.md).
