import time

from loguru import logger

from drivers.interface.if_pyserial import PySerial_UartInterfaceBuilder
from drivers.lc319 import LC319

PySerial_UartInterfaceBuilder("COM29").register()

flow = LC319()

while True:
    time.sleep(0.1)
    logger.info(
        f"x_integral: {flow.state.flow_x_integral:6.2f} mm, y_integral: {flow.state.flow_y_integral:6.2f} mm"
    )
