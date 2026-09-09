#!/usr/bin/env bash
#
# install_env.sh -- installe l'environnement Python du projet Hornet Death Star.
#
# A lancer sur le PC embarque de CHAQUE robot (myCobot 280 Jetson Nano) pour que
# les deux aient rigoureusement le meme environnement. Le script cree un
# environnement virtuel .venv a la racine du projet et y installe les versions
# figees de Environement/requirements.txt.
#
# Le script est relancable sans risque : par defaut il reutilise le .venv existant
# et se contente de remettre les paquets a la version demandee.
#
# Usage :
#   ./Environement/install_env.sh                 # environnement de base (bras, camera, son)
#   ./Environement/install_env.sh --avec-ia       # + YOLO / ultralytics (gros, voir requirements-ia.txt)
#   ./Environement/install_env.sh --recreer       # repart d'un .venv vide (en cas d'environnement casse)
#   ./Environement/install_env.sh --python /usr/bin/python3.10   # forcer un interpreteur precis
#
# Prerequis systeme (une seule fois par robot, demande le mot de passe) :
#   sudo apt install python3-venv python3-dev build-essential
#
# set -e : on s'arrete a la premiere erreur. Un environnement a moitie installe
# est pire qu'une installation qui echoue franchement, parce qu'il casse plus tard
# au milieu d'un essai avec le bras.
set -euo pipefail

### --- Emplacements --- ###
# On calcule la racine du projet a partir du chemin du script, pas du dossier
# courant : le script marche donc meme lance depuis ailleurs.
DOSSIER_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RACINE_PROJET="$(dirname "$DOSSIER_SCRIPT")"
DOSSIER_VENV="$RACINE_PROJET/.venv"
FICHIER_REQUIREMENTS="$DOSSIER_SCRIPT/requirements.txt"
FICHIER_REQUIREMENTS_IA="$DOSSIER_SCRIPT/requirements-ia.txt"

### --- Version minimale de Python --- ###
# 3.10 parce que c'est la version de l'environnement valide sur le robot
# (brasRobotEnv.yml) et que numpy 2.2 ne s'installe pas en dessous.
VERSION_MIN_MAJEURE=3
VERSION_MIN_MINEURE=10

### --- Options de la ligne de commande --- ###
avec_ia=0
recreer=0
python_impose=""

afficherAide() {
    # Affiche l'entete de ce fichier comme aide : on repart des lignes de
    # commentaire du debut et on s'arrete avant les notes destinees au mainteneur,
    # comme ca l'aide et l'entete ne peuvent pas diverger.
    awk 'NR >= 3 { if ($0 ~ /^# set -e/) exit; sub(/^# ?/, ""); print }' "${BASH_SOURCE[0]}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --avec-ia)  avec_ia=1 ;;
        --recreer)  recreer=1 ;;
        --python)
            if [ $# -lt 2 ]; then
                echo "ERREUR : --python attend un chemin d'interpreteur." >&2
                exit 1
            fi
            python_impose="$2"
            shift
            ;;
        -h|--help)  afficherAide; exit 0 ;;
        *)
            echo "ERREUR : option inconnue '$1'. Utiliser --help." >&2
            exit 1
            ;;
    esac
    shift
done

### --- Petites fonctions d'affichage --- ###
etape()  { printf '\n\033[1;34m==> %s\033[0m\n' "$1"; }
info()   { printf '    %s\n' "$1"; }
alerte() { printf '\033[1;33m/!\\  %s\033[0m\n' "$1"; }
echouer() { printf '\033[1;31mERREUR : %s\033[0m\n' "$1" >&2; exit 1; }

### --- 1. Choix de l'interpreteur Python --- ###
# On teste plusieurs candidats et on garde le premier assez recent, au lieu de
# supposer que "python3" est le bon : sur une Jetson, le python3 du systeme est
# souvent trop vieux alors qu'un python3.10 est installe a cote.
versionSuffisante() {
    "$1" -c "import sys; sys.exit(0 if sys.version_info >= ($VERSION_MIN_MAJEURE, $VERSION_MIN_MINEURE) else 1)" 2>/dev/null
}

etape "Recherche d'un interpreteur Python >= $VERSION_MIN_MAJEURE.$VERSION_MIN_MINEURE"
PYTHON=""
if [ -n "$python_impose" ]; then
    command -v "$python_impose" >/dev/null 2>&1 || echouer "interpreteur introuvable : $python_impose"
    versionSuffisante "$python_impose" || echouer "$python_impose est trop ancien (il faut >= $VERSION_MIN_MAJEURE.$VERSION_MIN_MINEURE)."
    PYTHON="$python_impose"
else
    for candidat in python3.12 python3.11 python3.10 python3; do
        if command -v "$candidat" >/dev/null 2>&1 && versionSuffisante "$candidat"; then
            PYTHON="$(command -v "$candidat")"
            break
        fi
    done
fi

if [ -z "$PYTHON" ]; then
    alerte "Aucun Python >= $VERSION_MIN_MAJEURE.$VERSION_MIN_MINEURE trouve."
    info "Pistes : sudo apt install python3.10 python3.10-venv"
    info "ou, si miniconda est deja installe sur le robot, reutiliser son interpreteur :"
    info "    ./Environement/install_env.sh --python ~/miniconda3/envs/brasRobotEnv/bin/python"
    exit 1
fi
info "Interpreteur retenu : $PYTHON ($("$PYTHON" --version 2>&1))"

# Choix de l'outil qui va creer l'environnement.
# Piege Debian/Ubuntu : "python3 -m venv" repond a --help meme quand le paquet
# python3-venv n'est pas installe, et n'echoue qu'au moment de la creation avec
# "ensurepip is not available". On teste donc ensurepip directement, et on se
# rabat sur virtualenv (qui embarque son propre pip) quand il manque : ca evite
# d'exiger un sudo sur une machine ou virtualenv est deja la.
if "$PYTHON" -c 'import ensurepip' >/dev/null 2>&1; then
    OUTIL_VENV="venv"
    info "Creation via le module venv."
elif "$PYTHON" -m virtualenv --version >/dev/null 2>&1; then
    OUTIL_VENV="virtualenv_module"
    info "Module venv incomplet (ensurepip absent) : utilisation de virtualenv."
elif command -v virtualenv >/dev/null 2>&1; then
    OUTIL_VENV="virtualenv_commande"
    info "Module venv incomplet (ensurepip absent) : utilisation de la commande virtualenv."
else
    alerte "Impossible de creer un environnement virtuel avec $PYTHON."
    info "Le module venv est incomplet (ensurepip absent) et virtualenv n'est pas installe."
    info "Corriger avec l'un des deux :"
    paquet_venv="$("$PYTHON" -c 'import sys; print(f"python3.{sys.version_info.minor}-venv")')"
    info "    sudo apt install $paquet_venv"
    info "    pip install --user virtualenv"
    exit 1
fi

### --- 2. Creation de l'environnement virtuel --- ###
if [ "$recreer" -eq 1 ] && [ -d "$DOSSIER_VENV" ]; then
    etape "Suppression de l'ancien environnement (--recreer)"
    rm -rf "$DOSSIER_VENV"
fi

# Un dossier .venv sans interpreteur dedans est un reste d'installation
# interrompue : on le refait au lieu de le reutiliser et d'echouer plus loin.
if [ -d "$DOSSIER_VENV" ] && [ ! -x "$DOSSIER_VENV/bin/python" ]; then
    alerte "Environnement existant incomplet : il est recree."
    rm -rf "$DOSSIER_VENV"
fi

if [ -d "$DOSSIER_VENV" ]; then
    etape "Environnement existant reutilise : $DOSSIER_VENV"
    info "Pour repartir de zero : ./Environement/install_env.sh --recreer"
else
    etape "Creation de l'environnement virtuel : $DOSSIER_VENV"
    case "$OUTIL_VENV" in
        venv)                "$PYTHON" -m venv "$DOSSIER_VENV" ;;
        virtualenv_module)   "$PYTHON" -m virtualenv -p "$PYTHON" "$DOSSIER_VENV" ;;
        virtualenv_commande) virtualenv -p "$PYTHON" "$DOSSIER_VENV" ;;
    esac
    # En cas d'echec partiel, on ne laisse pas un dossier a moitie fait derriere nous.
    if [ ! -x "$DOSSIER_VENV/bin/python" ]; then
        rm -rf "$DOSSIER_VENV"
        echouer "la creation de l'environnement a echoue (voir le message ci-dessus)."
    fi
fi

PIP="$DOSSIER_VENV/bin/pip"
PYTHON_VENV="$DOSSIER_VENV/bin/python"

### --- 3. Installation des paquets --- ###
# --no-cache-dir : la eMMC de la Jetson Nano est petite, et le cache pip ne sert
# a rien ici puisque chaque robot n'installe qu'une fois.
OPTIONS_PIP="--no-cache-dir --disable-pip-version-check"

etape "Mise a jour de pip / setuptools / wheel"
# shellcheck disable=SC2086
"$PIP" install $OPTIONS_PIP --upgrade pip setuptools wheel

etape "Installation des dependances de base"
info "Cette etape peut etre longue sur la Jetson (compilations aarch64)."
# shellcheck disable=SC2086
"$PIP" install $OPTIONS_PIP -r "$FICHIER_REQUIREMENTS"

if [ "$avec_ia" -eq 1 ]; then
    etape "Installation des dependances IA (YOLO)"
    alerte "Sur Jetson Nano, PyTorch doit venir de la wheel NVIDIA correspondant au JetPack."
    alerte "Voir l'avertissement en tete de Environement/requirements-ia.txt."
    # shellcheck disable=SC2086
    "$PIP" install $OPTIONS_PIP -r "$FICHIER_REQUIREMENTS_IA"
fi

### --- 4. Verification --- ###
# On importe reellement les modules critiques : une installation qui se termine
# sans erreur ne garantit pas que la bibliotheque se charge sur cette machine
# (wheel d'une mauvaise architecture, bibliotheque systeme manquante...).
etape "Verification des imports"
"$PYTHON_VENV" - <<'VERIF'
import importlib
import sys

# (module importe, nom affiche) -- le nom du module ne suit pas toujours celui du paquet pip.
MODULES = [
    ("pymycobot", "pymycobot"),
    ("serial", "pyserial"),
    ("cv2", "opencv-contrib-python"),
    ("numpy", "numpy"),
    ("scipy", "scipy"),
]

echecs = []
for module, nom_paquet in MODULES:
    try:
        objet = importlib.import_module(module)
        version = getattr(objet, "__version__", "version inconnue")
        print(f"    OK   {nom_paquet:<26} {version}")
    except Exception as erreur:
        echecs.append(nom_paquet)
        print(f"    KO   {nom_paquet:<26} {erreur}")

# Jetson.GPIO s'importe mais leve une erreur hors carte Jetson : ce n'est pas
# un echec d'installation, on l'affiche a part sans faire echouer le script.
try:
    import Jetson.GPIO  # noqa: F401
    print("    OK   Jetson.GPIO")
except Exception as erreur:
    print(f"    --   Jetson.GPIO indisponible ({type(erreur).__name__}) : normal hors Jetson.")

sys.exit(1 if echecs else 0)
VERIF

### --- 5. Verifications materielles (informatif, ne bloque pas) --- ###
etape "Acces au port serie du bras"
if [ -e /dev/ttyTHS1 ]; then
    info "/dev/ttyTHS1 present (liaison de la Jetson vers la carte du bras)."
elif ls /dev/ttyUSB* >/dev/null 2>&1; then
    info "Pas de /dev/ttyTHS1, mais un port USB serie existe : $(ls /dev/ttyUSB* | tr '\n' ' ')"
    info "Adapter le port dans Bras/ModuleGPIO/PU_GPIO.py si besoin."
else
    alerte "Aucun port serie detecte : le bras ne repondra pas tant qu'il n'est pas branche."
fi

if ! id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    alerte "L'utilisateur $USER n'est pas dans le groupe 'dialout' : l'ouverture du port serie sera refusee."
    info "Corriger avec :  sudo usermod -aG dialout $USER   puis se deconnecter / reconnecter."
fi

### --- 6. Conclusion --- ###
etape "Environnement pret"
cat <<FIN
    Activation :
        source "$DOSSIER_VENV/bin/activate"

    Verification rapide du bras une fois active :
        python -c "from pymycobot.mycobot280 import MyCobot280; print('pymycobot OK')"

    Pour installer le meme environnement sur le second robot, copier le depot
    et relancer ce script : les versions sont figees, les deux seront identiques.
FIN
