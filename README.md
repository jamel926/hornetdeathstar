# Hornet Death Star


#  Hornet Death Star (Étoile de la Mort pour Frelons) (VF)

> **Système automatisé de défense de ruche open-source basé sur la vision par ordinateur et la robotique.**

Le projet **Hornet Death Star** est une solution conçue pour protéger les ruches contre les attaques de prédateurs, notamment les frelons asiatiques. Il combine la détection par intelligence artificielle et la précision robotique pour neutraliser les menaces de manière autonome à l'aide d'un laser.

---

##  Comment ça marche ?

1. **Surveillance :** Une caméra placée en amont de la ruche capture le flux vidéo d'une zone d'environ 1m².
2. **Détection (IA) :** Un modèle d'intelligence artificielle basé sur **YOLO** analyse le flux vidéo en temps réel pour détecter les frelons qui entrent dans le champ de vision.
3. **Ciblage et Neutralisation :** Dès la détection d'une cible, l'IA extrait les coordonnées spatiales du frelon et les transmet au bras robotique (**myCobot 280 Jetson Nano**). La tête du bras se déplace alors pour aligner son laser et brûler la cible.

![Illustration du système de ciblage](Remplacer_par_le_chemin_de_votre_image.png)

---

## Matériel Requis

* **Bras Robotique :** Elephant Robotics myCobot 280 (Version Jetson Nano)
* **Caméra :** Caméra capable de capturer une surface d'1m² avec un bon framerate.
* **Module Laser :** Laser de découpe/gravure (puissance adaptée à la neutralisation).
* **Environnement :** L'architecture a été pensée pour un environnement Linux (type Ubuntu) afin d'optimiser l'exécution de l'IA et la communication série avec le bras.

---

##  Environnement et Dépendances

Il est fortement recommandé de faire tourner ce projet dans un environnement virtuel Python. Bien que la configuration exacte dépende de votre modèle, le système repose principalement sur :
* `ultralytics` pour l'architecture YOLO.
* `opencv-python` pour le traitement des trames vidéo et la création des filtres visuels.
* `pymycobot` pour le contrôle des servomoteurs du bras.

---

##  Arborescence du Projet

```text
hornetdeathstar/
├── README.md               # Documentation principale du projet
├── main.py                 # Script principal orchestrant la vision, l'IA et le bras
├── gui.py                  # Interface graphique (contrôle, armement du système, retours visuels)
├── HornetDeathStar.sh      # Script bash de lancement rapide
├── Bras/                   # Scripts relatifs au contrôle du myCobot
│   ├── Moduledeplacement/
│   │   ├── toCoordinate.py # Calcul et conversion des coordonnées spatiales
│   │   ├── withAngles.py   # Gestion des mouvements par angles de jointures
│   │   └── pointapoint.py  # Fonctions de déplacement d'un point A à un point B
│   └── ModuleGPIO/
│       └── laser.py        # Pilotage des broches GPIO pour activer/désactiver le laser
├── Camera/                 # Scripts relatifs à la vision par ordinateur
│   ├── camBasic.py         # Capture et traitement primaire du flux vidéo
│   └── reconaissanceObj/
│       ├── ia.py           # Chargement du modèle YOLO et inférence
│       └── algo.py         # Logique de tri (ciblage, application de filtres, tracking)
└── Documentation/          # Ressources annexes
    ├── img/                # Captures d'écran, schémas et illustrations
    └── Documentation_fonctionelle/ # Manuels détaillés de chaque sous-système
```

## Utilisation

Ouvrez votre dossier `hornetdeathstar` dans un terminal (assurez-vous d'avoir accordé les droits d'exécution au script) puis exécutez la commande suivante pour lancer le système complet :

Bash

```
chmod +x HornetDeathStar.sh
./HornetDeathStar.sh
```

_(Ce script se charge de préparer l'environnement et de lancer l'interface graphique le processus `main.py` principal)._

##  Avertissement de Sécurité

**L'utilisation de lasers de haute puissance (Classe 3B ou 4) présente des risques graves et irréversibles pour les yeux (humains et animaux), ainsi que des risques d'incendie.**

- Ce projet doit être manipulé, testé et calibré avec des **lunettes de protection certifiées** et adaptées à la longueur d'onde de votre module laser.
    
- L'utilisation en extérieur exige des mesures de confinement physique autour de la zone de tir (1m²) pour éviter que des faisceaux errants n'atteignent des passants, des animaux non ciblés ou des aéronefs. L'auteur de ce dépôt décline toute responsabilité en cas de dommages matériels ou corporels.

# Hornet Death Star (EV)

> **Open-source automated hive defense system based on computer vision and robotics.**

The **Hornet Death Star** project is a solution designed to protect beehives from predators, specifically Asian hornets. It combines artificial intelligence detection with robotic precision to autonomously neutralize threats using a laser.

---

##  How it works?

1. **Surveillance:** A camera placed in front of the hive captures the video feed of an area of approximately 1 square meter.
2. **Detection (AI):** An artificial intelligence model based on **YOLO** analyzes the live video feed to detect hornets entering its field of view.
3. **Targeting and Neutralization:** Upon detecting a target, the AI extracts the hornet's spatial coordinates and sends them to the robotic arm (**myCobot 280 Jetson Nano**). The arm's head then moves to align its laser and burn the target.

![Targeting system illustration](Replace_with_your_image_path.png)

---

## 🛠️ Hardware Requirements

* **Robotic Arm:** Elephant Robotics myCobot 280 (Jetson Nano Version)
* **Camera:** Camera capable of capturing a 1m² area with a good framerate.
* **Laser Module:** Cutting/engraving laser (power adapted for neutralization).
* **Environment:** The architecture has been designed for a Linux environment (Ubuntu) to optimize AI execution and serial communication with the arm.

---

##  Environment & Dependencies

It is highly recommended to run this project in a Python virtual environment. While the exact setup depends on your specific hardware model, the system mainly relies on:
* `ultralytics` for the YOLO architecture.
* `opencv-python` for video frame processing and visual filters.
* `pymycobot` to control the arm's servomotors.

---

##  Project Structure

```text
hornetdeathstar/
├── README.md               # Main project documentation
├── main.py                 # Main script orchestrating vision, AI, and the arm
├── gui.py                  # Graphical interface (control, system arming, visual feedback)
├── HornetDeathStar.sh      # Quick launch bash script
├── Bras/                   # Scripts related to myCobot control
│   ├── Moduledeplacement/
│   │   ├── toCoordinate.py # Calculation and conversion of spatial coordinates
│   │   ├── withAngles.py   # Movement management via joint angles
│   │   └── pointapoint.py  # Point-to-point movement functions
│   └── ModuleGPIO/
│       └── laser.py        # GPIO pin control to activate/deactivate the laser
├── Camera/                 # Scripts related to computer vision
│   ├── camBasic.py         # Video feed capture and primary processing
│   └── reconaissanceObj/
│       ├── ia.py           # YOLO model loading and inference
│       └── algo.py         # Sorting logic (targeting, filters, tracking)
└── Documentation/          # Additional resources
    ├── img/                # Screenshots, diagrams, and illustrations
    └── Documentation_fonctionelle/ # Detailed manuals for each subsystem
```

## Usage

Open your `hornetdeathstar` folder in a terminal (ensure you have granted execution rights to the script) and run the following command to launch the full system:

Bash

```
chmod +x HornetDeathStar.sh
./HornetDeathStar.sh
```

_(This script sets up the environment and launches either the `gui.py` graphical interface or the main `main.py` process)._

##  Safety Warning

**Using high-power lasers (Class 3B or 4) presents serious and irreversible risks to the eyes (both human and animal), as well as fire hazards.**

- This project must be handled, tested, and calibrated while wearing **certified protective glasses** appropriate for your laser module's wavelength.
    
- Outdoor use requires physical confinement measures around the firing zone (1m²) to prevent stray beams from hitting bystanders, non-targeted animals, or aircraft. The author of this repository assumes no responsibility for any property damage or personal injury.