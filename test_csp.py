import cv2
import numpy as np
from loguru import logger

from drivers.csp202tt import CSP202TT
from drivers.interface.if_pyserial import Pyserial_UartInterfaceBuilder

Pyserial_UartInterfaceBuilder("COM29").register()


radar = CSP202TT()
radar.start_listen()
cv2.namedWindow("radar", cv2.WINDOW_NORMAL)
cv2.resizeWindow("radar", 800, 800)
center = [400, 100]
bimg = np.zeros((800, 800, 3), np.uint8)
cv2.circle(bimg, (center[0], center[1]), 7, (0, 255, 255), -1)
while True:
    img = bimg.copy()
    for target in radar.targets:
        if target.x == 0 and target.y == 0:
            continue
        x = int(target.x / 10) + center[0]
        y = int(target.y / 10) + center[1]
        cv2.circle(img, (x, y), 5, (255, 255, 0), -1)
        cv2.putText(
            img,
            f"(x={target.x}, y={target.y}, spd={target.speed})",
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
        )
        cv2.circle(bimg, (x, y), 2, (100, 100, 100 + round(abs(target.speed) * 2)), -1)
    cv2.imshow("radar", img)
    if cv2.waitKey(1) == ord("q"):
        break
