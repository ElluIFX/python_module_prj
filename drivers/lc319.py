import threading
import time
from dataclasses import dataclass
from struct import unpack

from loguru import logger

from .interface import request_interface

PACKAGE_HEAD = b"\xFE\x0A"
PACKAGE_TAIL = b"\x55"
# pack: HEADx2 + flow_x_raw:i16 + flow_y_raw:i16 + integral_time:u16 + tof_dist:u16 + flow_valid:u8 + tof_conf:u8 + XOR[3:12] + TAIL


def calc_xor(data: bytes):
    xor = data[0]
    for i in data[1:]:
        xor ^= i
    return xor


@dataclass
class FlowFrame:
    flow_x_raw: int  # mm
    flow_y_raw: int  # mm
    integral_time: int  # us
    tof_distance: int  # mm
    flow_valid: int  # 0:invalid, 0xF5:valid
    tof_conf: int  # 0-100:conefidence


@dataclass
class FlowState:
    flow_x: float  # mm
    flow_y: float  # mm
    flox_x_speed: float  # mm/s
    flow_y_speed: float  # mm/s
    flow_x_integral: float  # mm (accumulated)
    flow_y_integral: float  # mm (accumulated)
    tof_distance: int  # mm
    flow_valid: bool
    tof_conf: int


class LC319(object):
    """
    LC319 flow/tof intergrated sensor driver
    """

    def __init__(self, start_listen=True) -> None:
        self._ser = request_interface("uart", "LC319", 115200)
        self.state = FlowState(0, 0, 0, 0, 0, 0, 0, False, 0)
        self.last_update_time = 0
        if start_listen:
            self.start_listen()

    def _handle_data(self, data: bytes):
        if calc_xor(data[:-1]) != data[-1]:
            logger.error("LC319 XOR check error")
        frame = FlowFrame(*unpack("hhHHBB", data[:-1]))
        self.state.flow_valid = frame.flow_valid == 0xF5
        self.state.flow_x = frame.flow_x_raw / 10000 * frame.tof_distance
        self.state.flow_y = frame.flow_y_raw / 10000 * frame.tof_distance
        self.state.flox_x_speed = self.state.flow_x / frame.integral_time * 1000000
        self.state.flow_y_speed = self.state.flow_y / frame.integral_time * 1000000
        self.state.flow_x_integral += self.state.flow_x
        self.state.flow_y_integral += self.state.flow_y
        self.state.tof_distance = frame.tof_distance
        self.state.tof_conf = frame.tof_conf
        self.last_update_time = time.time()
        # logger.debug(f"LC319: {frame}")

    def _read_worker(self):
        buf = bytes()
        head_size = len(PACKAGE_HEAD)
        pack_size = 11
        tail_size = len(PACKAGE_TAIL)
        while True:
            try:
                buf += self._ser.read(self._ser.in_waiting)
                if (idx := (buf.find(PACKAGE_HEAD))) != -1:
                    buf = buf[idx + head_size :]
                    while True:
                        buf += self._ser.read(self._ser.in_waiting)
                        if len(buf) >= pack_size + tail_size:
                            if buf[pack_size : pack_size + tail_size] == PACKAGE_TAIL:
                                self._handle_data(buf[:pack_size])
                            buf = buf[pack_size + tail_size :]
                            break
                else:
                    time.sleep(0.01)
            except Exception:
                logger.exception("LC319 listen error")

    def start_listen(self):
        self._read_thread = threading.Thread(target=self._read_worker, daemon=True)
        self._read_thread.start()
