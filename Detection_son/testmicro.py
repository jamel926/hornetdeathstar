import machine
import time
import uarray
import neopixel

led = neopixel.NeoPixel(machine.Pin(27), 1)
def led_rouge():
    led[0] = (100, 0, 0) # (Rouge, Vert, Bleu) - Intensité modérée (0-255)
    led.write()

def led_vert():
    led[0] = (0, 150, 0) # (Rouge, Vert, Bleu) - Intensité modérée (0-255)
    led.write()

#Pin 25 
# def toggle_gpio(niveau):
#     gpio=machine.Pin(33,machine.Pin.OUT)
#     if niveau in (0,1):
#         if niveau==1:
#             gpio.value(1)                        
#         else:
#             gpio.value(0)

I2S_ID = 0
BCLK_PIN = 19  
LRCL_PIN = 22  
DOUT_PIN = 23  

SAMPLE_RATE = 16000 
BUFFER_SIZE = 1024  

audio_in = machine.I2S(
    I2S_ID,
    sck=machine.Pin(BCLK_PIN),
    ws=machine.Pin(LRCL_PIN),
    sd=machine.Pin(DOUT_PIN),
    mode=machine.I2S.RX,
    bits=16,                              
    format=machine.I2S.MONO,
    rate=SAMPLE_RATE,
    ibuf=BUFFER_SIZE * 4
)

samples = uarray.array('h', [0] * BUFFER_SIZE)
# Tableau pour stocker le signal filtré
filtered_samples = [0] * BUFFER_SIZE

print("Système de detection sonore de frelon activé !")

while True:
    audio_in.readinto(samples)
    #toggle_gpio(1)
    # 1. Filtre Passe-Bas très simple (Lissage)
    # On élimine les variations trop rapides (hautes fréquences / sifflements)
    filtered_samples[0] = samples[0]
    for i in range(1, BUFFER_SIZE):
        # Filtre exponentiel : on garde 85% de la valeur précédente + 15% de la nouvelle
        filtered_samples[i] = int(0.85 * filtered_samples[i-1] + 0.15 * samples[i])
        
    # 2. Calcul de la moyenne du signal lissé
    moyenne = sum(filtered_samples) / BUFFER_SIZE
    croisements = 0
    
    # 3. Calcul des passages par zéro sur le signal propre
    for i in range(1, BUFFER_SIZE):
        if (filtered_samples[i-1] < moyenne and filtered_samples[i] >= moyenne) or (filtered_samples[i-1] > moyenne and filtered_samples[i] <= moyenne):
            croisements += 1
            
    duree_capture_s = BUFFER_SIZE / SAMPLE_RATE
    frequence_estimee = (croisements / 2) / duree_capture_s
    
    # On calcule l'amplitude sur le signal d'origine
    amplitude_max = max(samples) - min(samples)
    
    # 4. Analyse avec un seuil d'amplitude plus élevé (2000 au lieu de 800)
    if amplitude_max > 100: 
        print("Fréquence : {:.1f} Hz | Amplitude : {}".format(frequence_estimee, amplitude_max))
        
        if 285 <= frequence_estimee <= 315:
            print("⚠️ ALERTE : Battement d'ailes suspect (Frelon) !")
            led_rouge()
    else:
        print("Silence... (Bruit de fond ignoré)")
        led_vert()
    #toggle_gpio(0)
    time.sleep(0.3)
    