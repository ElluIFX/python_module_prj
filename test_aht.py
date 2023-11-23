import time

from drivers.aht21b import AHT21B
from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder

CP2112_I2CInterfaceBuilder().register()

aht = AHT21B()

while True:
    aht.measure()
    print(f"AHT20 | Temperature: {aht.temperature} C, Humidity: {aht.humidity} %")
    time.sleep(1)
