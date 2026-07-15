# Module de Détection Acoustique (VF)

Ce sous-module du projet **Hornet Death Star** est dédié à la reconnaissance et à l'analyse sonore de l'environnement immédiat de la ruche. Il agit comme un système de sécurité critique (safety check) permettant de confirmer la nature de la cible détectée par les caméras avant toute activation du laser.

## Principe de Fonctionnement

Les abeilles (*Apis mellifera*) et les frelons asiatiques (*Vespa velutina*) possèdent des morphologies et des masses très différentes, ce qui se traduit par des vitesses de battement d'ailes distinctes. 
* L'abeille génère un bourdonnement généralement situé autour de **200-250 Hz**.
* Le frelon asiatique génère un son plus grave, souvent mesuré autour de **100-150 Hz** (avec des harmoniques spécifiques).

Ce module utilise un microphone pour enregistrer le son ambiant. Le script applique ensuite une **Transformée de Fourier Rapide (FFT)** sur le signal audio pour isoler les fréquences dominantes, analyser le spectre sonore, et ainsi discriminer un frelon d'une abeille avec un haut niveau de certitude.

## Contenu du Dossier

* **`analysefreq.py`** : Script principal de traitement du signal. Il capture l'audio (en direct ou via un fichier) et exécute l'analyse fréquentielle pour déterminer la probabilité de présence d'un frelon.
* **`testmicro.py`** : Outil de diagnostic pour vérifier que le microphone est correctement reconnu par le système, régler le gain d'entrée et s'assurer de l'absence de saturation.
* **`sonbee.wav`** : Fichier audio de référence contenant le bourdonnement d'une abeille (utilisé pour les tests de validation de l'algorithme).
* **`sonfrelon.wav`** : Fichier audio de référence contenant le bourdonnement d'un frelon asiatique.

## Utilisation

### 1. Tester le microphone
Avant de lancer le système complet, assurez-vous que votre périphérique d'entrée audio est bien configuré :
```bash
python testmicro.py
```

### 2. Lancer une analyse sur les fichiers de test
Pour vérifier l'efficacité de l'algorithme de FFT, vous pouvez exécuter le script d'analyse sur les fichiers audio fournis :
```bash
# Test avec le son du frelon (doit déclencher une alerte/confirmation positive)
python analysefreq.py --input sonfrelon.wav

# Test avec le son de l'abeille (ne doit pas déclencher d'alerte)
python analysefreq.py --input sonbee.wav
```

---
---

# Acoustic Detection Module (EV)

This submodule of the **Hornet Death Star** project is dedicated to recognizing and analyzing the sound environment immediately surrounding the hive. It acts as a critical safety check to confirm the nature of the target detected by the cameras before any laser activation.

## How it Works

Bees (*Apis mellifera*) and Asian hornets (*Vespa velutina*) have very different morphologies and masses, resulting in distinct wingbeat frequencies.
* A bee's buzz is generally located around **200-250 Hz**.
* An Asian hornet produces a lower-pitched sound, often measured around **100-150 Hz** (with specific harmonics).

This module uses a microphone to record ambient sound. The script then applies a **Fast Fourier Transform (FFT)** to the audio signal to isolate dominant frequencies, analyze the sound spectrum, and thus discriminate a hornet from a bee with a high degree of certainty.

## Folder Contents

* **`analysefreq.py`**: The main signal processing script. It captures audio (live or via a file) and performs frequency analysis to determine the probability of a hornet's presence.
* **`testmicro.py`**: A diagnostic tool to verify that the microphone is correctly recognized by the system, adjust input gain, and ensure there is no clipping.
* **`sonbee.wav`**: Reference audio file containing a bee's buzz (used for testing and validating the algorithm).
* **`sonfrelon.wav`**: Reference audio file containing an Asian hornet's buzz.

## Usage

### 1. Test the Microphone
Before launching the full system, ensure your audio input device is properly configured:
```bash
python testmicro.py
```

### 2. Run Analysis on Test Files
To check the effectiveness of the FFT algorithm, you can run the analysis script on the provided audio files:
```bash
# Test with hornet sound (should trigger an alert/positive confirmation)
python analysefreq.py --input sonfrelon.wav

# Test with bee sound (should not trigger an alert)
python analysefreq.py --input sonbee.wav
```
