import time

from drivers.interface.if_cp2112 import CP2112_I2CInterfaceBuilder
from drivers.mlx90614 import MLX90614

CP2112_I2CInterfaceBuilder(clock=100000).register()
sensor = MLX90614()
while True:
    ambient_temp = sensor.ambient_temperature
    object_temp = sensor.object_temperature
    print(
        f"Ambient Temperature: {ambient_temp:.2f}C Object Temperature: {object_temp:.2f}C"
    )
    time.sleep(1.0)
