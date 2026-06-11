"""
diagnostic_mycobot.py
=====================
Script de diagnostic et reset pour myCobot280 Jetson Nano.
Lance ce script en premier pour déboguer un robot figé.

Usage :
    python diagnostic_mycobot.py
"""

import time
from pymycobot.mycobot import MyCobot

# ── Adapte ces deux valeurs ─────────────────────────────────────────
PORT     = "/dev/ttyUSB0"   # ou /dev/ttyTHS1
BAUDRATE = 1000000
# ────────────────────────────────────────────────────────────────────

print("=== Connexion au robot ===")
mc = MyCobot(PORT, BAUDRATE)
time.sleep(1.0)   # laisse le temps à l'UART de s'initialiser

# ── 1. Lire l'état actuel ───────────────────────────────────────────
print("\n── État actuel ──")

angles = mc.get_angles()
print(f"  Angles articulaires  : {angles}")

coords = mc.get_coords()
print(f"  Coordonnées (x,y,z,rx,ry,rz) : {coords}")

moving = mc.is_moving()
print(f"  Est en mouvement     : {moving}")

# ── 2. Forcer la sortie de tout mode bloquant ───────────────────────
print("\n── Reset des modes ──")
mc.stop()                    # stoppe tout mouvement en cours
time.sleep(0.3)

mc.set_fresh_mode(1)         # 1 = exécute chaque commande immédiatement
time.sleep(0.3)

mc.release_all_servos()      # relâche les servos (robot devient mou)
time.sleep(1.0)
print("  Servos relâchés — bouge le bras manuellement à la main vers")
print("  une position neutre si nécessaire, puis appuie sur Entrée.")
input("  [Entrée pour continuer] ")

mc.power_on()                # réactive les servos
time.sleep(0.5)
print("  Servos réactivés.")

# ── 3. Test de mouvement articulaire (le plus fiable) ───────────────
print("\n── Test mouvement articulaire (angles joints) ──")
print("  Envoi : tous les angles à 0° (position neutre)…")
mc.send_angles([0, 0, 0, 0, 0, 0], 20)
time.sleep(4.0)

angles_apres = mc.get_angles()
print(f"  Angles après move    : {angles_apres}")

# ── 4. Test avec send_coords ────────────────────────────────────────
print("\n── Test send_coords ──")
coords_cible = [0.0, 0.0, 250.0, 0.0, 0.0, 0.0]
print(f"  Envoi coords : {coords_cible}  vitesse=20  mode=1")
mc.send_coords(coords_cible, 20, 1)
time.sleep(4.0)

coords_apres = mc.get_coords()
print(f"  Coords après move    : {coords_apres}")

# ── 5. Test avec une inclinaison visible ────────────────────────────
print("\n── Test inclinaison laser (rx=45°) ──")
coords_test = [0.0, 0.0, 250.0, 45.0, 0.0, 0.0]
print(f"  Envoi coords : {coords_test}  vitesse=20  mode=1")
mc.send_coords(coords_test, 20, 1)
time.sleep(4.0)

coords_apres2 = mc.get_coords()
print(f"  Coords après move    : {coords_apres2}")

print("\n=== Diagnostic terminé ===")
print("Si le robot n'a toujours pas bougé aux étapes 3-5,")
print("le problème est matériel (câble, alimentation, firmware).")
print("Si l'étape 3 fonctionne mais pas 4-5, utilise send_angles")
print("à la place de send_coords dans le code principal.")
