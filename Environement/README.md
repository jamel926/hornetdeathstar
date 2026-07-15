# Configuration de l'Environnement

Ce dossier contient tout le nécessaire pour isoler les dépendances du projet Hornet Death Star et garantir que le code s'exécute correctement sur n'importe quelle machine.

## Contenu
* `brasRobotEnv.yml` : Fichier d'export de l'environnement (idéal pour Conda).
* `hornetdeathstart_venv.zip` : Archive de l'environnement virtuel Python prêt à l'emploi.

## Installation

**Option 1 : Utilisation avec Conda (Recommandé)**
```bash
conda env create -f brasRobotEnv.yml
conda activate brasRobotEnv
```

**Option 2 : Utilisation du venv classique**
1. Décompressez `hornetdeathstart_venv.zip` dans votre répertoire de travail.
2. Activez l'environnement :
   * Sous Linux/Mac : `source hornetdeathstart_venv/bin/activate`
   * Sous Windows : `hornetdeathstart_venv\Scripts\activate`
