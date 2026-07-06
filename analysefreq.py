import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile

# 1. Charger le fichier audio (au format WAV obligatoire)
frequence_echantillonnage, signal = wavfile.read('sonbee.wav')
print(frequence_echantillonnage)
# Si le fichier est en stéréo, on ne prend qu'un seul canal (le gauche)
if len(signal.shape) > 1:
    signal = signal[:, 0]

# 2. Préparer les données pour la FFT
n = len(signal) # Nombre total d'échantillons
k = np.arange(n)
T = n / frequence_echantillonnage # Durée totale de l'audio en secondes

# Vecteur des fréquences (l'axe X de notre futur graphique)
frequences = k / T
frequences = frequences[range(int(n/2))] # On ne garde que la moitié positive (symétrie)

# 3. Calculer la FFT
fft_signal = np.fft.fft(signal) / n # Calcul de la FFT et normalisation
fft_signal = fft_signal[range(int(n/2))] # On garde aussi la moitié positive
amplitude = np.abs(fft_signal) # On prend le module (l'amplitude absolue)

# 4. Afficher la courbe
plt.figure(figsize=(10, 5))
plt.plot(frequences, amplitude, color='blue')
plt.title("Spectre de fréquence du signal audio")
plt.xlabel("Fréquence (Hz)")
plt.ylabel("Amplitude")
plt.grid(True)

# Optionnel : Limiter l'affichage aux fréquences audibles (ex: 0 à 5000 Hz ou 20000 Hz)
plt.xlim(0, 500) 

plt.show()