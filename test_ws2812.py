import time

from drivers import register_interface
from drivers.ina226 import INA226Monitor
from drivers.ws2812 import WS2812, param_calculator

if __name__ == "__main__":
    register_interface("ch347", "spi")
    register_interface("ch347", "i2c")
    m = INA226Monitor().start()
    led = WS2812(12, 7_500_000)
    hue = 0
    while True:
        hue += 0.5
        for i in range(len(led)):
            led[i] = led.hsv_to_rgb(hue + i * 10, 255, 255, merge=True)
        led.show()
        time.sleep(0.01)
