import os
import re
import subprocess as sp
from functools import lru_cache
from typing import Dict

from loguru import logger

from .errors import InterfacefIOError


def i2c_bus_scan() -> list[int]:
    from . import request_interface

    i2c_bus = request_interface("i2c", "bus-scanner", 0x01)
    addrs = []
    for addr in range(1, 128):
        try:
            i2c_bus.address = addr
            if i2c_bus.check_address():
                addrs.append(addr)
        except InterfacefIOError:
            pass
    logger.info(f"I2C bus scan result: {[hex(addr) for addr in addrs]}")
    i2c_bus.destroy()
    return addrs


def get_permission(dev):
    """
    Get permission to access device
    """
    if os.name != "posix":
        raise Exception(f"Permission denied to access {dev}")
    logger.warning(f"Permission denied to access {dev}, tring get permission")
    if os.system(f"sudo chmod 666 {dev}") != 0:
        raise Exception(f"Access to {dev} failed, please check your permission")


@lru_cache(64)
def find_gpio(name: str) -> tuple[int, int]:
    """
    Use gpiofind to find gpio chip and offset
    """
    cmd = f"gpiofind {name}"
    ret = sp.getoutput(cmd)
    if ret == "":
        raise Exception(f"GPIO {name} not found")
    chip, offset = ret.split(" ")
    return int(chip[-1]), int(offset)


def list_gpio() -> Dict[str, tuple[int, int, bool]]:
    """
    Use gpioinfo to find available gpio

    Returns:
        dict: {name: (chip, offset, used)}
    """
    gpios = {}
    chip_list = []
    ret = sp.getoutput("gpiodetect")
    for line in ret.split("\n"):
        match = re.search(r"gpiochip(\d+)", line)
        if match is None:
            continue
        chip_list.append(int(match.group(1)))
    for chip in chip_list:
        ret = sp.getoutput(f"gpioinfo gpiochip{chip}")
        for line in ret.split("\n"):
            try:
                # find string \"xxx\"
                match_name = re.search(r"\".+?\"", line)
                match_line = re.search(r"line\s*(\d+):", line)
                if not match_name or not match_line:
                    continue
                name = match_name.group(0).strip('"')
                line_num = int(match_line.group(1))
                gpios[name] = (chip, line_num, "[used]" in line)
            except Exception:
                pass
    return gpios


class FakeLock:
    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass
