import time

import ADD_PATH  # noqa: F401
import cv2
import numpy as np
from PIL import Image

from drivers.interface import register_interface
from drivers.mono_lcd import ST7302

register_interface("ch347", "spi")
register_interface(
    "ch347",
    "gpio",
    pinmap={"DC": "GPIO4", "RST": "GPIO7"},
)

lcd = ST7302(rotation=0, fps=8)
print(f"screen initialized with {lcd.WIDTH}x{lcd.HEIGHT}")


def bad_apple():
    video = cv2.VideoCapture("badapple.mp4")
    frame_num = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_cnt = 0
    while True:
        # time.sleep(0.01)
        ret, frame = video.read()
        if not ret:
            break
        frame_cnt += 1
        frame = cv2.resize(frame, (lcd.WIDTH, lcd.HEIGHT))
        t0 = time.perf_counter()
        lcd.draw_cv2_img(frame)
        lcd.refresh()
        dt = time.perf_counter() - t0
        print(f"FPS: {1/dt:.2f} dt: {dt:.6f} frame: {frame_cnt}/{frame_num}")


def screen_cast():
    import pyautogui

    while True:
        t0 = time.perf_counter()
        img = pyautogui.screenshot()
        img = np.array(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.resize(img, (lcd.WIDTH, lcd.HEIGHT))
        img = Image.fromarray(img)
        img = img.convert("1")
        lcd.draw_pil_img(img, polarity=False)
        lcd.refresh()
        dt = time.perf_counter() - t0
        print(f"FPS: {1/dt:.2f} dt: {dt:.6f}")


bad_apple()
# screen_cast()
