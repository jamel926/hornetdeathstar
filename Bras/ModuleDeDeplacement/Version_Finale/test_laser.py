from pymycobot import MyCobot280
import time

mc = MyCobot280('/dev/ttyTHS1', 1000000)

PIN = 33

print("Configuration G33 en sortie")
mc.set_pin_mode(PIN, 1)
time.sleep(1)

print("G33 = 0 -> LASER ALLUME")
mc.set_digital_output(PIN, 0)
time.sleep(3)

print("G33 = 1 -> LASER ETEINT")
mc.set_digital_output(PIN, 1)
time.sleep(3)

print("G33 = 0 -> LASER ALLUME")
mc.set_digital_output(PIN, 0)
time.sleep(3)

print("G33 = 1 -> LASER ETEINT")
mc.set_digital_output(PIN, 1)
time.sleep(3)

print("Fin du test")