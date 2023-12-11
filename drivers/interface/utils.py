import os
import re
import subprocess as sp
import time
from functools import lru_cache
from typing import Dict

from loguru import logger

from .errortype import InterfacefIOError


def i2c_bus_scan(highest_addr: int = 0x78) -> list[int]:
    from . import request_interface

    i2c_bus = request_interface("i2c", "i2c-bus-scanner", 0x01)
    addrs = []
    for addr in range(highest_addr + 1):
        try:
            i2c_bus.address = addr
        except InterfacefIOError as e:
            logger.warning(f"Address {hex(addr)} not supported by bus driver: {e}")
            continue
        try:
            if i2c_bus.check_address():
                addrs.append(addr)
        except InterfacefIOError as e:
            logger.warning(f"Address {hex(addr)} communication failed: {e}")
    logger.info(f"I2C bus scan result: {[hex(addr) for addr in addrs]}")
    i2c_bus.destroy()
    return addrs


def get_permission(path: str):
    """
    Get permission to access device
    """
    if os.name != "posix":
        raise RuntimeError(f"Permission denied to access {path:str}")
    logger.warning(f"Permission denied to access {path:str}, tring get permission")
    if os.system(f"sudo chmod 666 {path:str}") != 0:
        raise RuntimeError(f"Access to {path:str} failed, please check your permission")


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


def retry_if_raise(func, retry=3, delay=0.01):
    def wrapper(*args, **kwargs):
        for i in range(retry):
            try:
                return func(*args, **kwargs)
            except Exception:
                if i == retry - 1:
                    raise
                logger.debug(f"Retry-{i+1}: {func.__name__}")
                time.sleep(delay)
        raise RuntimeError("Unreachable")

    return wrapper


def retry_if_none(func, retry=3, delay=0.01):
    def wrapper(*args, **kwargs):
        for i in range(retry):
            ret = func(*args, **kwargs)
            if ret is not None:
                return ret
            logger.debug(f"Retry-{i+1}: {func.__name__}")
            time.sleep(delay)
        return None

    return wrapper
