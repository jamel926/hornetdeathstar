from pymycobot import MyCobot280
import time

mc = MyCobot280('/dev/ttyTHS1', 1000000)

PIN = 2

print("GPIO 2 -> LOW")
mc.set_basic_output(PIN, 0)
time.sleep(3)

print("GPIO 2 -> HIGH")
mc.set_basic_output(PIN, 1)
time.sleep(3)

print("GPIO 2 -> LOW")
mc.set_basic_output(PIN, 0)
time.sleep(3)

print("Fin du test")
