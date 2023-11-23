import time

from loguru import logger

from drivers.aht20 import AHT20
from drivers.bmp280 import BMP280
from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder

CP2112_I2CInterfaceBuilder().register()

aht = AHT20()
bmp = BMP280()

while True:
    time.sleep(1)
    aht.measure()
    logger.info(f"AHT20 | Temperature: {aht.temperature} C, Humidity: {aht.humidity} %")
    logger.info(
        f"BMP280 | Temperature: {bmp.read_temperature()} C, Pressure: {bmp.read_pressure()} Pa, Altitude: {bmp.read_altitude()} m"
    )
