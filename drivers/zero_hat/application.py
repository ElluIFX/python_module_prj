import time
from typing import List

from loguru import logger
from .protocal import ZHProtocolLayer
from .utils import hsv_to_rgb


class WS2812(object):
    def _write(self, data: bytes) -> None:
        ...

    def deinit(self) -> None:
        # cmd: 0x00
        self._write(bytes([0x00]))
        time.sleep(0.1)

    def init(self, num_leds: int, send: bool = True) -> None:
        # cmd: 0x01
        data = int(send) << 7 | 0x01
        self._write(bytes([data, num_leds]))
        time.sleep(0.1)

    def set_color(self, colors: List[int], from_n: int = 0, send: bool = True) -> None:
        # cmd: 0x02 / 0x04
        if from_n == 0:
            data = bytes([int(send) << 7 | 0x02])
        else:
            data = bytes([int(send) << 7 | 0x04, from_n])
        for c in colors:
            r = c >> 16 & 0xFF
            g = c >> 8 & 0xFF
            b = c & 0xFF
            data += bytes([r, g, b])
        self._write(data)

    def set_color_range(
        self, color: int, from_n: int, to_n: int, send: bool = True
    ) -> None:
        # cmd:0x03
        data = bytes([int(send) << 7 | 0x03, from_n, to_n])
        r = color >> 16 & 0xFF
        g = color >> 8 & 0xFF
        b = color & 0xFF
        data += bytes([r, g, b])
        self._write(data)

    def clear(self, send: bool = True) -> None:
        # cmd: 0x05
        data = bytes([int(send) << 7 | 0x05])
        self._write(data)

    def send(self) -> None:
        # cmd: 0x06
        data = bytes([0x06])
        self._write(data)

    def send_part(self, send_to: int) -> None:
        # cmd: 0x07
        data = bytes([0x07, send_to])
        self._write(data)

    @staticmethod
    def hsv_to_rgb(h: int, s: int, v: int) -> int:
        return hsv_to_rgb(h, s, v, True)  # type: ignore


class ZHAppLayer(ZHProtocolLayer):
    """
    应用层, 基于协议层进行开发, 不触及底层通信
    """

    def __init__(self, *args, **kwargs) -> None:
        self.ws2812 = WS2812()
        self.ws2812._write = self._write_ws2812
        super().__init__(*args, **kwargs)

    def _write_ws2812(self, data: bytes) -> None:
        self.send_raw_data(data, 0x0F)

    def wait_for_connection(self, timeout_s=-1) -> bool:
        """
        等待连接
        """
        t0 = time.perf_counter()
        while not self.connected:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[ZH] wait for fc connection timeout")
                return False
        return True

    def set_hsv_led(self, h, s, v):
        """
        设置HSV LED
        """
        self.set_rgb_led(*hsv_to_rgb(h, s, v, False))  # type: ignore
