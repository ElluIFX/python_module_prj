import time
from typing import Literal

import cv2
import numpy as np
from PIL import Image

from .interface import request_interface


class ST7302:
    """
    中景圆 ZJY213S0700TG01 2.13inch 250x122 仿电子纸单色LCD屏驱动
    """

    WIDTH = 250
    HEIGHT = 122

    def __init__(
        self,
        fps: Literal[1, 2, 4, 8, 16, 32] = 8,
        inverse: bool = False,
        rotation: Literal[0, 1] = 0,
    ) -> None:
        self._spi = request_interface("spi", "ST7302", 0, 10_000_000)
        self._gpio = request_interface("gpio", "ST7302")

        self._pin_dc = self._gpio.get_pin("DC")
        self._pin_rst = self._gpio.get_pin("RST")
        self._pin_dc.set_mode("output_push_pull")
        self._pin_rst.set_mode("output_push_pull")
        self._pin_dc.write(True)  # DC keeps in high
        self._pin_rst.write(True)  # RST keeps in high

        self._inv = inverse
        self._rot = rotation
        self._fps = fps

        self._buf = bytearray(4125)

        self.reset_hardware()
        self._init_cmd()
        self._set_window_full()
        self.clear()

    def _cmd(self, data):
        self._pin_dc.write(False)
        self._spi.transfer([data & 0xFF])
        self._pin_dc.write(True)

    def _data(self, data):
        if isinstance(data, int):
            data = [data & 0xFF]
        self._spi.transfer(data)

    def _write_cmd(self, cmd, data):
        self._cmd(cmd)
        if data is not None:
            self._data(data)

    def _write(self, data):
        MAX_SEND = 4080
        if len(data) <= MAX_SEND:
            self._spi.transfer(data)
        else:
            for i in range(0, len(data), MAX_SEND):
                self._spi.transfer(data[i : i + MAX_SEND])

    def reset_hardware(self):
        self._pin_rst.write(True)
        time.sleep(0.1)
        self._pin_rst.write(False)
        time.sleep(0.1)
        self._pin_rst.write(True)
        time.sleep(0.5)

    def _init_cmd(self):
        self._write_cmd(0xEB, b"\x02")  # Enable OTP
        self._write_cmd(0xD7, b"\x68")  # OTP Load Control
        self._write_cmd(0xD1, b"\x01")  # Auto Power Control
        self._write_cmd(0xC0, b"\x80")  # Gate Voltage Setting VGH=12V  VGL=-5V

        # Fps in HP=(16/32),LP=(0.25/0.5/1/2/4/8)
        # self._write_cmd(0xB2, b"\x01\x05")  # Frame Rate Control
        # self._write_cmd(0x38, None)  # Enable High Power Mode
        # self._write_cmd(0x39, None)  # Enable Low Power Mode
        fps_h = {16: 0, 32: 1}.get(self._fps, 1)
        fps_l = {0.25: 0, 0.5: 1, 1: 2, 2: 3, 4: 4, 8: 5}.get(self._fps, 5)
        self._write_cmd(0xB2, bytes([fps_h, fps_l]))
        self._write_cmd(0x38 if self._fps > 8 else 0x39, None)

        self._write_cmd(0xC1, b"\x28\x28\x28\x28\x14\x00")  # VSH Setting
        self._write_cmd(0xC2, b"\x00\x00\x00\x00")  # VSL Setting VSL=0
        self._write_cmd(0xCB, b"\x14")  # VCOMH Setting
        self._write_cmd(
            0xB4, b"\xE5\x77\xF1\xFF\xFF\x4F\xF1\xFF\xFF\x4F"
        )  # Gate EQ Setting HPM EQ LPM EQ
        self._write_cmd(0x11, None)  # Sleep out
        time.sleep(0.1)
        self._write_cmd(0xC7, b"\xA6\xE9")  # OSC Setting
        self._write_cmd(0xB0, b"\x64")  # Duty Setting
        if self._rot == 0:
            self._write_cmd(0x36, b"\x00")  # Memory Data Access Control
        elif self._rot == 1:
            self._write_cmd(0x36, b"\x4C")
        self._write_cmd(0x3A, b"\x11")  # Data Format Select 4 write for 24 bit
        self._write_cmd(0xB9, b"\x23")  # Source Setting
        self._write_cmd(0xB8, b"\x09")  # Panel Setting Frame inversion
        self._write_cmd(0x2A, b"\x05\x36")  ##Column Address Setting S61~S182
        self._write_cmd(0x2B, b"\x00\xC7")  ##Row Address Setting G1~G250
        self._write_cmd(0xD0, b"\x1F")
        if self._inv:
            self._write_cmd(0x21, None)
        else:
            self._write_cmd(0x20, None)
        self._write_cmd(0x29, None)  # Display on
        self._write_cmd(0xB9, b"\xE3")  # enable CLR RAM
        time.sleep(0.1)
        self._write_cmd(0xB9, b"\x23")  # enable CLR RAM
        self._write_cmd(0x72, b"\x00")  # Destress OFF
        self._write_cmd(0x2A, b"\x19\x23")  # Column Address Setting
        self._write_cmd(0x2B, b"\x00\x7C")  # Row Address Setting
        self._write_cmd(0x2C, None)  # write image data
        time.sleep(0.2)

    def _set_window(self, x0: int, y0: int, x1: int, y1: int):
        self._cmd(0x2A)
        self._data(x0 & 0x3F)
        self._data(x1 & 0x3F)
        self._cmd(0x2B)
        self._data(y0 & 0xFF)
        self._data(y1 & 0xFF)
        self._cmd(0x2C)

    def _set_window_full(self):
        if self._rot == 0:
            self._set_window(0x19, 0x00, 0x23, 0x7C)
        elif self._rot == 1:
            self._set_window(0x19, 0x4B, 0x23, 0xC7)
        else:
            raise ValueError("Rotation must be 0 or 1")

    def refresh(self):
        self._cmd(0x2C)
        self._write(self._buf)

    def clear(self, data: int = 0x00):
        self._buf = bytearray([data] * len(self._buf))
        self.refresh()

    def draw_pixel(self, x: int, y: int, color: bool = True):
        if x < 0 or x >= self.WIDTH or y < 0 or y >= self.HEIGHT:
            return
        mask = 0x80 >> (x % 2 + (y % 4) * 2)
        if color:
            self._buf[x // 2 * 0x21 + y // 4] |= mask
        else:
            self._buf[x // 2 * 0x21 + y // 4] &= ~mask

    def draw_cv2_img(
        self,
        img: np.ndarray,
        x0: int = 0,
        y0: int = 0,
        threshold: int = 128,
        polarity: bool = True,
    ):
        """
        Draw a cv2 image to the screen
        """
        if len(img.shape) == 3 and img.shape[2] != 1:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = img > threshold
        for y, row in enumerate(img):
            for x, pixel in enumerate(row):
                self.draw_pixel(x0 + x, y0 + y, pixel if polarity else not pixel)

    def draw_pil_img(
        self,
        img: Image.Image,
        x0: int = 0,
        y0: int = 0,
        polarity: bool = True,
    ):
        """
        Draw a PIL image to the screen
        """
        if img.mode != "1":
            img = img.convert("1")
        for y in range(img.height):
            for x in range(img.width):
                self.draw_pixel(
                    x0 + x,
                    y0 + y,
                    img.getpixel((x, y)) if polarity else not img.getpixel((x, y)),
                )
