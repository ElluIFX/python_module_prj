import time

from drivers.interface import register_interface
from drivers.tcs34725 import TCS34725

register_interface("cp2112", "i2c", clock=400000)

sensor = TCS34725()
# Change sensor integration time to values between 2.4 and 614.4 milliseconds
# sensor.integration_time = 150

# Change sensor gain to 1, 4, 16, or 60
sensor.gain = 16
while True:
    # Raw data from the sensor in a 4-tuple of red, green, blue, clear light component values
    # print(sensor.color_raw)

    color = sensor.color
    color_rgb = sensor.color_rgb_bytes
    print(
        f"RGB color as 8 bits per channel int: #{color:08X} or as 3-tuple: {color_rgb}"
    )

    # Read the color temperature and lux of the sensor too.
    temp = sensor.color_temperature
    lux = sensor.lux
    print(f"Temperature: {temp:.2f}K Lux: {lux:.2f}")
    # Delay for a second and repeat.
    time.sleep(1.0)
