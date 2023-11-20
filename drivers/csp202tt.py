import math
import threading
import time
from dataclasses import dataclass
from struct import unpack

from loguru import logger

from .interface import InterfaceManager

PACKAGE_HEAD = b"\xAA\xFF\x03\x00"
PACKAGE_TAIL = b"\x55\xCC"


@dataclass
class Target:
    x: int  # mm
    y: int  # mm
    speed: int  # cm/s
    resolution: int  # mm
    degree: float = 0.0  # degree
    distance: float = 0.0  # m


class CSP202TT(object):
    def __init__(self, start_listen=True) -> None:
        self._ser = InterfaceManager.request_uart_interface("CSP202TT", 256000)
        self.targets = [Target(0, 0, 0, 0)] * 3
        self.last_update_time = 0
        if start_listen:
            self.start_listen()

    @property
    def target1(self) -> Target:
        return self.targets[0]

    @property
    def target2(self) -> Target:
        return self.targets[1]

    @property
    def target3(self) -> Target:
        return self.targets[2]

    @property
    def insight_num(self) -> int:
        return len([t for t in self.targets if t.x != 0 or t.y != 0])

    def _unpack_one(self, data: bytes) -> Target:
        x, y, s, r = unpack("HHHH", data)  # 4 uint16_t
        x = -x if x < 0x8000 else x - 0x8000
        y = -y if y < 0x8000 else y - 0x8000
        s = -s if s < 0x8000 else s - 0x8000
        if x > 0 or y > 0:
            degree = math.atan2(y, x) / math.pi * 180 - 90
            distance = math.sqrt(x**2 + y**2) / 1000
            return Target(x, y, s, r, degree, distance)
        return Target(0, 0, 0, 0)

    def _unpack_targets(self, data: bytes):
        if len(data) != 24:  # 3 targets
            return
        self.targets = [self._unpack_one(data[i * 8 : i * 8 + 8]) for i in range(3)]
        self.last_update_time = time.time()

    def _read_worker(self):
        buf = bytes()
        head_size = len(PACKAGE_HEAD)
        pack_size = 24
        tail_size = len(PACKAGE_TAIL)
        while True:
            try:
                buf += self._ser.read(self._ser.in_waiting)
                if (idx := (buf.find(PACKAGE_HEAD))) != -1:
                    buf = buf[idx + head_size :]
                    while True:
                        buf += self._ser.read(self._ser.in_waiting)
                        if len(buf) >= pack_size + tail_size:
                            self._unpack_targets(buf[:pack_size])
                            buf = buf[pack_size + tail_size :]
                            break
                else:
                    time.sleep(0.01)
            except Exception:
                logger.exception("CSP202TT read error")

    def start_listen(self):
        self._read_thread = threading.Thread(target=self._read_worker, daemon=True)
        self._read_thread.start()
