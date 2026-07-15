# Perspectives et Axes d'Amélioration - Hornet Death Star

Ce document recense les pistes d'évolution technique pour le projet Hornet Death Star. L'objectif est de finaliser l'architecture logicielle, d'optimiser les algorithmes, de consolider le déploiement sur le terrain et de renforcer la fiabilité globale du système.

## 1. Architecture Logicielle et Interface (Priorité)

* **Développement du Programme Principal (main) :** Création du script central chargé d'orchestrer tous les sous-systèmes. Ce programme devra gérer la boucle d'exécution principale, la synchronisation des processus (multithreading/multiprocessing) et la communication fluide entre le module de vision, le module de déplacement du bras, les GPIO et le module audio.
* **Finalisation de l'Interface Graphique (GUI) :** Transformer le brouillon actuel en un tableau de bord fonctionnel. L'interface devra permettre à l'utilisateur de visualiser les retours vidéo stéréoscopiques, de surveiller l'analyse spectrale audio, de calibrer le système, et d'armer ou de désarmer manuellement le laser en toute sécurité.
* **Intégration Audio comme Déclencheur IA :** Modifier la logique d'exécution pour que le microphone serve de système de veille ("wake-word"). L'analyse fréquentielle (peu gourmande en calcul) tournera en continu. Le traitement vidéo lourd par l'intelligence artificielle (YOLO) ne sera réveillé et activé que lorsque le système acoustique aura confirmé la fréquence de vol d'un frelon. Cela permettra d'économiser drastiquement les ressources processeur de la carte embarquée.

## 2. Électronique et Intégration Matérielle

* **Fabrication et Assemblage du Connecteur :** Passage de la conception à la réalisation physique de l'interface électronique reliant le système de commande au module laser. Les étapes concrètes impliquent :
    * Le tirage (fabrication) de la carte PCB (Printed Circuit Board) conçue pour le projet.
    * La soudure des câbles de liaison afin d'assurer une transmission propre et fiable des signaux de commande (GPIO) et de l'alimentation vers la tête laser.

## 3. Optimisations Vision et Intelligence Artificielle

* **Création d'un Dataset Spécifique en Conditions Réelles :** Constitution d'une base de données d'images et de vidéos capturées directement par les caméras du système sur le terrain. Cela permettra de réentraîner et d'affiner le modèle YOLO pour mieux différencier les frelons des abeilles et autres insectes locaux, en tenant compte des variations de lumière, du flou de mouvement et de l'angle de vue spécifique à l'entrée de la ruche.
* **Prédiction de Trajectoire (Filtres de Kalman) :** Le temps de calcul de la stéréovision couplé au déplacement physique du bras crée une latence. L'implémentation d'un algorithme prédictif permettrait d'estimer le vecteur de déplacement du frelon et d'anticiper sa position future, compensant ainsi le décalage temporel lors du ciblage.
* **Accélération de l'Inférence (TensorRT) :** Optimisation du modèle YOLO actuel pour exploiter au maximum l'architecture GPU de la carte embarquée. L'objectif est d'augmenter le nombre de trames par seconde (FPS) traitées simultanément par les deux caméras.

## 4. Traitement du Signal Acoustique

* **Filtrage Dynamique du Bruit Environnant :** Amélioration du script d'analyse fréquentielle (FFT) en intégrant des filtres adaptatifs ou une soustraction de profil de bruit. Cela garantira une détection fiable de la fréquence de vol du frelon (100-150 Hz) même en présence de bruits parasites constants (vent, trafic routier).
* **Classification Audio par Machine Learning :** Remplacer la simple lecture des pics de fréquence par un modèle de classification audio léger. Le système pourrait apprendre à discriminer des signatures sonores plus complexes, réduisant le taux de faux positifs.

## 5. Déploiement et Infrastructure (Terrain)

* **Système d'Alimentation :** Conception d'une infrastructure d'alimentation adaptée à un déploiement en rucher (souvent isolé du réseau électrique). Cela implique l'étude et l'intégration d'un système de gestion de l'énergie, potentiellement via un couplage de panneaux solaires avec des batteries adaptées, pour garantir le fonctionnement continu du bras robotique et des modules de calcul.
* **Système de Transmission des Données :** Mise en place d'un module de communication distant (ex: routeur 4G/LTE, module LoRaWAN ou pont Wi-Fi longue portée) pour permettre la supervision à distance. Ce système transmettra la télémétrie, les journaux d'activité (logs), les statistiques de détection et les alertes de sécurité sans nécessiter d'intervention physique sur le site.

## 6. Sécurité Logique et Fiabilité Opérationnelle

* **Zones d'Exclusion Virtuelles (Geofencing Logiciel) :** Programmation de "masques" sur les flux vidéo pour définir des zones de non-tir strictes (par exemple, la planche d'envol de la ruche où la densité d'abeilles est maximale). Le laser reste désarmé jusqu'à ce que la cible atteigne une zone de tir sûre.
* **Routine d'Autodiagnostic Logiciel :** Développement d'un script de vérification au démarrage s'assurant que les flux vidéo sont synchronisés, que le microphone capte le son, et que la communication série avec le bras répond avant d'autoriser le passage en mode armé.
