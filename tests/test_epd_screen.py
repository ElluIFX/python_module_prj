import os
import time
from math import floor

import ADD_PATH  # noqa: F401
import cv2

from drivers import register_interface
from drivers.epd import EPD_BW_154, EPD_G4_42

os.chdir(os.path.dirname(os.path.abspath(__file__)))
register_interface("ch347", "spi")
# register_interface("ch347", "i2c")
register_interface(
    "ch347",
    "gpio",
    pinmap={
        "DC": "GPIO7",
        "RST": "GPIO4",
        "BUSY": "GPIO6",
    },
)


def vertical_text(
    img, text, pos, font, scale, color, thinkness, x_offset=None, y_offset=3
):
    x, y = pos
    text_len = len(text)
    sizes = [
        cv2.getTextSize(text[i], font, scale, thinkness)[0] for i in range(text_len)
    ]
    avg_x = sum([sizes[i][0] for i in range(text_len)]) / text_len
    for i in range(text_len):
        if x_offset is None:
            x_offset_set = floor((avg_x - sizes[i][0]) / 2)
        else:
            x_offset_set = x_offset
        y += sizes[i][1] + y_offset
        cv2.putText(img, text[i], (x + x_offset_set, y), font, scale, color, thinkness)


def test_image():
    with EPD_G4_42() as epd:
        cvimg = cv2.imread("./res/amiya.jpg")
        # cvimg = cv2.threshold(cvimg, 185, 255, cv2.THRESH_BINARY)[1]
        cvimg = epd.fit_cv2(cvimg)
        # epd.clear()
        t0 = time.time()
        epd.clear()
        epd.display_base(cvimg)
        t1 = time.time()
        print("display_base: ", t1 - t0)
        # t0 = time.time()
        # epd.display_partial(cvimg)
        # t1 = time.time()
        # print("display_partial: ", t1 - t0)
        # t0 = time.time()
        # epd.display_partial(cvimg)
        # t1 = time.time()
        # print("display_partial: ", t1 - t0)
        # epd.display_grayscale(cvimg)


test_image()
