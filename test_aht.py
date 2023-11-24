import time

from drivers.aht21b import AHT21B
from drivers.interface import register_interface

register_interface("cp2112", "i2c", clock=400000)

aht = AHT21B()

while True:
    aht.measure()
    print(f"AHT20 | Temperature: {aht.temperature} C, Humidity: {aht.humidity} %")
    time.sleep(1)
