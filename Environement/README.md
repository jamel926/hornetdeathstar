# Configuration de l'Environnement

Ce dossier contient tout le nécessaire pour isoler les dépendances du projet Hornet Death Star et garantir que le code s'exécute correctement sur n'importe quelle machine — en particulier sur les **deux PC embarqués** (myCobot 280 Jetson Nano), qui doivent avoir un environnement **identique**.

## Contenu

| Fichier | Rôle |
|---|---|
| `install_env.sh` | Script d'installation : crée le `.venv` du projet et y installe les dépendances. **Méthode recommandée.** |
| `requirements.txt` | Versions figées des dépendances de base (bras, caméra, son). C'est ce fichier qui garantit que les deux robots sont identiques. |
| `requirements-ia.txt` | Dépendances supplémentaires pour la détection YOLO (lourdes : PyTorch). |
| `brasRobotEnv.yml` | Export de l'environnement Conda d'origine du robot — source des versions figées ci-dessus. |
| `hornetdeathstart_venv.zip` | Archive de l'ancien environnement virtuel. Conservée par sécurité, mais un venv n'est pas censé être déplacé d'une machine à l'autre : préférer le script. |

## Installation (recommandé)

Depuis la racine du projet :

```bash
chmod +x Environement/install_env.sh
./Environement/install_env.sh
```

Le script crée `.venv/` à la racine du dépôt, installe les versions figées, puis vérifie que les modules critiques s'importent réellement. Il est relançable sans risque.

Options :

```bash
./Environement/install_env.sh --avec-ia    # ajoute ultralytics / YOLO (~2-3 Go)
./Environement/install_env.sh --recreer    # repart d'un .venv vide
./Environement/install_env.sh --python /usr/bin/python3.10   # forcer un interpréteur
./Environement/install_env.sh --help
```

Activation ensuite :

```bash
source .venv/bin/activate
```

### Prérequis système (une fois par robot)

```bash
sudo apt install python3-venv python3-dev build-essential
```

Il faut un **Python ≥ 3.10** (celui de l'environnement validé sur le robot ; `numpy 2.2` ne s'installe pas en dessous). Si le `python3` du système est trop ancien, pointer le script vers un autre interpréteur, par exemple celui de miniconda déjà présent sur le robot :

```bash
./Environement/install_env.sh --python ~/miniconda3/envs/brasRobotEnv/bin/python
```

Pour accéder au port série du bras (`/dev/ttyTHS1`), l'utilisateur doit appartenir au groupe `dialout` — le script le vérifie et affiche la commande à lancer sinon.

## Alternatives

**Conda** (environnement d'origine du robot) :
```bash
conda env create -f brasRobotEnv.yml
conda activate brasRobotEnv
```

**Archive ZIP** (dépannage uniquement) : décompresser `hornetdeathstart_venv.zip` puis `source hornetdeathstart_venv/bin/activate`. Un environnement virtuel contient des chemins absolus, il ne se transporte pas proprement — à n'utiliser que si `install_env.sh` n'est pas une option.

## Mettre à jour une dépendance

Les versions sont figées volontairement (`==`) : une différence entre les deux robots rendrait un bug reproductible sur l'un et pas sur l'autre. Pour changer une version, modifier `requirements.txt`, relancer le script **sur les deux robots**, et commiter le fichier.
