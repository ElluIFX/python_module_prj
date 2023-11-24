import time

from loguru import logger

from drivers.aht20 import AHT20
from drivers.bmp280 import BMP280
from drivers.interface import register_interface

register_interface("cp2112", "i2c", clock=400000)

aht = AHT20()
bmp = BMP280()

while True:
    time.sleep(1)
    aht.measure()
    logger.info(f"AHT20 | Temperature: {aht.temperature} C, Humidity: {aht.humidity} %")
    logger.info(
        f"BMP280 | Temperature: {bmp.read_temperature()} C, Pressure: {bmp.read_pressure()} Pa, Altitude: {bmp.read_altitude()} m"
    )
