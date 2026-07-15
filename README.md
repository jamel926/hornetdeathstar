# Hornet Death Star (Étoile de la Mort pour Frelons) (VF)

> **Système automatisé de défense de ruche open-source basé sur la vision stéréoscopique, l'analyse acoustique et la robotique.**

Le projet **Hornet Death Star** est une solution conçue pour protéger les ruches contre les attaques de prédateurs, notamment les frelons asiatiques. Il combine la détection par intelligence artificielle (caméras), la confirmation acoustique (microphones) et la précision robotique pour neutraliser les menaces de manière autonome à l'aide d'un laser.

## L'Équipe
Projet développé à l'ENSICAEN par :
* **Étudiants :** Lucas Brunet, Jamel-Eddine Dridi, Mamadou Mamoudou Diallo, Tristan Perigois, Constance Heveline, Tom Jousselin.
* **Sous la tutelle de :** Matthieu Denoual et Olivier Clouard.

---

## Comment ça marche ?

1.  **Surveillance Stéréoscopique :** Un système composé de **deux caméras** placées en amont de la ruche capture les flux vidéo croisés d'une zone de couverture d'environ **75cm x 75cm**.
2.  **Détection Acoustique & Confirmation :** Un microphone capte l'environnement sonore. Le système analyse les fréquences de vol pour distinguer avec certitude le bourdonnement d'un frelon de celui d'une abeille, évitant ainsi les tirs alliés.
3.  **Suivi IA :** Un modèle d'intelligence artificielle basé sur **YOLO** analyse les flux vidéo en temps réel pour traquer la menace confirmée dans le champ de vision.
4.  **Triangulation et Neutralisation :** Dès que la cible est verrouillée sur les deux flux, le système triangule ses coordonnées spatiales (3D) exactes et les transmet au bras robotique (**myCobot 280 Jetson Nano**). La tête du bras se déplace alors pour aligner son laser et neutraliser la cible.

![Illustration du système de ciblage](Remplacer_par_le_chemin_de_votre_image.png)

---

## Matériel Requis

* **Bras Robotique :** Elephant Robotics myCobot 280 (Version Jetson Nano).
* **Caméras :** **Deux caméras** capables de capturer une surface de **75cm x 75cm** avec un bon framerate pour assurer la triangulation.
* **Audio :** Un microphone sensible pour la captation des fréquences de vol.
* **Module Laser :** Laser de découpe/gravure (puissance adaptée à la neutralisation).
* **Environnement :** L'architecture a été pensée pour un environnement Linux (type Ubuntu).

---

## Environnement et Dépendances

Il est fortement recommandé de faire tourner ce projet dans un environnement virtuel Python. Le système repose principalement sur :
* `ultralytics` pour l'architecture YOLO.
* `opencv-python` pour le traitement des trames vidéo et la stéréovision.
* `pymycobot` pour le contrôle du bras.
* `numpy` et `scipy` (ou équivalents) pour le traitement du signal audio (FFT).

---

## Arborescence du Projet

```text
.
├── Annexes/          # Modèles 3D (démonstrateur, supports), électronique et diagrammes
├── Bras/             # Scripts du ModuleDeDeplacement et ModuleGPIO (sécurité, laser)
├── camera/           # IA (YOLOv8, poids du modèle), Caméra (Stéréovision) et Reconnaissance
├── Detection_son/    # Analyse fréquentielle pour discriminer les abeilles des frelons
├── Environement/     # Fichiers YAML et archives ZIP pour l'environnement virtuel
├── GUI_brouillon_vibecode.py # Interface graphique en cours de développement
└── README.md         # Documentation principale du projet
```

## Utilisation

Ouvrez votre dossier `hornetdeathstar` dans un terminal (assurez-vous d'avoir accordé les droits d'exécution au script) puis exécutez :

```bash
chmod +x HornetDeathStar.sh
./HornetDeathStar.sh
```

## Avertissement de Sécurité

**L'utilisation de lasers de haute puissance (Classe 3B ou 4) présente des risques graves et irréversibles pour les yeux, ainsi que des risques d'incendie.**
* Ce projet doit être manipulé avec des **lunettes de protection certifiées**.
* L'utilisation en extérieur exige des mesures de confinement physique (**75cm x 75cm**). L'auteur décline toute responsabilité en cas de dommages.

---
---

# Hornet Death Star (EV)

> **Open-source automated hive defense system based on stereoscopic vision, acoustic analysis, and robotics.**

The **Hornet Death Star** project is a solution designed to protect beehives from predators, specifically Asian hornets. It combines AI detection (cameras), acoustic confirmation (microphones), and robotic precision to autonomously neutralize threats using a laser.

## Team
Project developed at ENSICAEN by:
* **Students:** Lucas Brunet, Jamel-Eddine Dridi, Mamadou Mamoudou Diallo, Tristan Perigois, Constance Heveline, Tom Jousselin.
* **Supervisors:** Matthieu Denoual and Olivier Clouard.

---

## How it works?

1.  **Stereoscopic Surveillance:** **Two cameras** placed in front of the hive capture crossing video feeds of a coverage area of approximately **75cm x 75cm**.
2.  **Acoustic Detection & Confirmation:** A microphone captures the soundscape. The system analyzes flight frequencies to accurately distinguish a hornet's buzz from a bee's, preventing friendly fire.
3.  **AI Tracking:** A **YOLO-based** AI model analyzes the video feeds in real-time to track the confirmed threat.
4.  **Triangulation and Neutralization:** Once locked on both feeds, the system triangulates the exact 3D spatial coordinates and sends them to the robotic arm (**myCobot 280 Jetson Nano**). The arm aligns its laser and neutralizes the target.

![Targeting system illustration](Replace_with_your_image_path.png)

---

## Hardware Requirements

* **Robotic Arm:** Elephant Robotics myCobot 280 (Jetson Nano Version).
* **Cameras:** **Two cameras** capable of capturing a **75cm x 75cm** area with a good framerate to ensure accurate triangulation.
* **Audio:** A sensitive microphone to capture flight frequencies.
* **Laser Module:** Cutting/engraving laser (power adapted for neutralization).
* **Environment:** The architecture has been designed for a Linux environment (Ubuntu).

---

## Environment & Dependencies

It is highly recommended to run this project in a Python virtual environment. The system mainly relies on:
* `ultralytics` for the YOLO architecture.
* `opencv-python` for video frame processing and stereovision.
* `pymycobot` to control the arm's servomotors.
* `numpy` and `scipy` (or equivalents) for audio signal processing (FFT).

---

## Project Structure

```text
.
├── Annexes/          # 3D models (demonstrator, mounts), electronics, and diagrams
├── Bras/             # Movement and GPIO scripts (safety, laser) for the myCobot
├── camera/           # AI (YOLOv8, model weights), Camera Module (Stereovision), and Object Recognition
├── Detection_son/    # Frequency analysis to discriminate bees from hornets
├── Environement/     # YAML files and ZIP archives for the virtual environment
├── GUI_brouillon_vibecode.py # Graphical User Interface draft
└── README.md         # Main project documentation
```

## Usage

Open your `hornetdeathstar` folder in a terminal (ensure you have granted execution rights to the script) and run the following command to launch the full system:

```bash
chmod +x HornetDeathStar.sh
./HornetDeathStar.sh
```

## Safety Warning

**Using high-power lasers (Class 3B or 4) presents serious and irreversible risks to the eyes (both human and animal), as well as fire hazards.**
* This project must be handled, tested, and calibrated while wearing **certified protective glasses** appropriate for your laser module's wavelength.
* Outdoor use requires physical confinement measures around the firing zone (**75cm x 75cm**). The author of this repository assumes no responsibility for any property damage or personal injury.
