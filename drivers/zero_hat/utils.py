import re
import struct
import time
from threading import Event
from typing import List, Literal

from loguru import logger


class ByteVar:
    """
    C-like byte类型变量与python泛型变量的转换类
    使用时直接操作成员bytes和value即可
    """

    name = ""

    def __set_name__(self, _, name):
        self.name = name

    def __init__(
        self,
        ctype: Literal[
            "u8", "u16", "u32", "s8", "s16", "s32", "float", "double"
        ] = "u8",
        var_type: type = int,
        value_multiplier: float = 1.0,
        little_endian: bool = True,
    ):
        """Args:
        ctype (str): C-like类型
        py_var_type (_type_): python类型
        value_multiplier (float, optional): 值在从byte向python转换时的乘数.
        """
        self.reset(0, ctype, var_type, value_multiplier, little_endian)

    def reset(
        self,
        init_value,
        ctype: Literal["u8", "u16", "u32", "s8", "s16", "s32", "float", "double"],
        py_var_type: type,
        value_multiplier: float = 1.0,
        little_endian: bool = True,
    ):
        """重置变量

        Args:
            init_value (_type_): 初始值(浮点值或整数值)
            ctype (str): C-like类型
            py_var_type (_type_): python类型
            value_multiplier (float, optional): 值在从byte向python转换时的乘数.
        """
        fmt_dict = {
            "u8": (1, "B", False),
            "u16": (2, "H", False),
            "u32": (4, "I", False),
            "s8": (1, "b", False),
            "s16": (2, "h", False),
            "s32": (4, "i", False),
            "float": (4, "f", True),
            "double": (8, "d", True),
        }
        if ctype not in fmt_dict:
            raise ValueError(f"Invalid ctype: {ctype}")
        self._byte_length, self._struct_fmt, self._float = fmt_dict[ctype]
        if py_var_type not in [int, float, bool]:
            raise ValueError(f"Invalid var_type: {py_var_type}")
        self._var_type = py_var_type
        self._multiplier = value_multiplier
        self._value = self._var_type(init_value)
        self._le = little_endian
        self._fmt = ("<" if self._le else ">") + self._struct_fmt
        return self

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = self._var_type(value)

    @property
    def multiplier(self) -> float:
        return self._multiplier

    @multiplier.setter
    def multiplier(self, value):
        self._multiplier = value

    @property
    def raw_value(self):
        return self._value / self._multiplier

    @raw_value.setter
    def raw_value(self, value):
        self._value = self._var_type(value * self._multiplier)

    @property
    def bytes(self):
        if self._multiplier != 1:
            val = self._value / self._multiplier
        else:
            val = self._value
        if not self._float:
            val = round(val)
        return struct.pack(self._fmt, val)

    @bytes.setter
    def bytes(self, value):
        self._value = self._var_type(struct.unpack(self._fmt, value)[0])

    @property
    def byte_length(self):
        return self._byte_length

    @property
    def struct_fmt(self):
        return self._struct_fmt

    def __str__(self) -> str:
        return str(self._value)

    def __int__(self) -> int:
        return int(self._value)

    def __float__(self) -> float:
        return float(self._value)

    def __bool__(self) -> bool:
        return bool(self._value)

    def __bytes__(self) -> "bytes":
        return self.bytes


class BaseStruct:
    PACK: List[ByteVar] = []  # 存放结构体对象的列表

    def __init__(self, little_endian=True):
        if little_endian:
            self._fmt_string = "<" + "".join([i.struct_fmt for i in self.PACK])
        else:
            self._fmt_string = "<" + "".join([i.struct_fmt for i in self.PACK])
        for i in self.PACK:
            i._le = little_endian
        self._fmt_length = struct.calcsize(self._fmt_string)
        self.update_event = Event()

    def __getitem__(self, key):
        assert isinstance(key, str)
        return getattr(self, key).value

    def update_from_bytes(self, bytes, print_update=False):
        if len(bytes) != self._fmt_length:
            raise ValueError(
                f"Invalid bytes length: {len(bytes)} != {self._fmt_length}"
            )
        vals = struct.unpack(self._fmt_string, bytes)
        for i, val in enumerate(vals):
            self.PACK[i].raw_value = val
        self.update_event.set()
        if print_update:
            self.print()

    @property
    def struct_fmt(self):
        return self._fmt_string

    @property
    def struct_length(self):
        return self._fmt_length

    def print(
        self,
        extra_info: List[str] = [],
        extra_data: List[ByteVar] = [],
        line_width: int = 90,  # 每行最多显示的字符数
        upper_margin: int = 1,  # 为日志留出的空间
    ):
        RED = "\033[1;31m"
        GREEN = "\033[1;32m"
        YELLOW = "\033[1;33m"
        BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        BACK = "\033[F"
        HEAD = f"{BLUE}| {RESET}"
        TAIL = f"{BLUE} |{RESET}"
        lines = [
            BLUE
            + "-" * ((line_width - 32) // 2)
            + PURPLE
            + " ▲ System log / ▼ System status "
            + BLUE
            + "-" * ((line_width - 32) // 2)
            + RESET,
            HEAD,
        ]

        def remove_color(text):
            return re.sub(r"\033\[[0-9;]*m", "", text)

        def len_s(text):
            return len(remove_color(text))

        varlist = [
            f"{YELLOW}{var.name}: {f'{GREEN}√ ' if var.value else f'{RED}x {RESET}'}"
            if isinstance(var.value, bool)
            else (
                f"{YELLOW}{var.name}:{CYAN}{var.value:>6.02f}{RESET}"
                if isinstance(var.value, float)
                else f"{YELLOW}{var.name}:{CYAN}{var.value:>5d}{RESET}"
            )
            for var in (self.PACK + extra_data)
        ] + extra_info
        for vartext in varlist:
            if vartext[-1] != " ":
                vartext += " "
            if len_s(lines[-1]) + len_s(vartext) > line_width - 2:
                lines[-1] += " " * (line_width - len_s(lines[-1]) - 2) + TAIL
                lines.append(HEAD)
            lines[-1] += vartext
        lines[-1] += " " * (line_width - len_s(lines[-1]) - 2) + TAIL
        lines.append(f"{BLUE}{'-' * line_width}{RESET}")
        for _ in range(upper_margin):
            lines.insert(0, " " * line_width)
        text = "\n".join(lines) + BACK * (len(lines) - 1)
        print(text, end="")


class BaseEvent:
    """事件类"""

    name = ""

    def __set_name__(self, owner, name):
        self.name = name

    def __init__(self):
        self._status = False
        self._callback = None
        self._callback_trigger = True

    def __bool__(self):
        return self._status

    def set(self):
        self._status = True
        self._check_callback()

    def clear(self):
        self._status = False
        self._check_callback()

    def wait(self, timeout=None) -> bool:
        """
        等待事件置位
        """
        if timeout is None:
            while not self._status:
                time.sleep(0.1)
        else:
            start_time = time.perf_counter()
            while not self._status:
                time.sleep(0.1)
                if time.perf_counter() - start_time > timeout:
                    logger.warning("[ZH] Wait for event timeout")
                    break
        return self._status

    def wait_clear(self, timeout=None) -> bool:
        """
        等待事件置位后自动清除
        """
        ret = self.wait(timeout)
        if ret:
            self.clear()
        return ret

    def _check_callback(self):
        if callable(self._callback) and self._status == self._callback_trigger:
            self._callback()

    def set_callback(self, callback, trigger=True):
        """设置回调函数

        Args:
            callback (function): 目标函数
            trigger (bool, optional): 回调触发极性 (True为事件置位时触发)
        """
        self._callback = callback
        self._callback_trigger = trigger

    def is_set(self) -> bool:
        return self._status


def hsv_to_rgb(h, s, v, merge=False):
    max_val = v
    min_val = max_val * (255 - s) / 255
    h = h % 360

    adj = (max_val - min_val) * (h % 60) / 60.0
    h_div_60 = int(h / 60)

    if h_div_60 == 0:
        r, g, b = max_val, min_val + adj, min_val
    elif h_div_60 == 1:
        r, g, b = max_val - adj, max_val, min_val
    elif h_div_60 == 2:
        r, g, b = min_val, max_val, min_val + adj
    elif h_div_60 == 3:
        r, g, b = min_val, max_val - adj, max_val
    elif h_div_60 == 4:
        r, g, b = min_val + adj, min_val, max_val
    else:
        r, g, b = max_val, min_val, max_val - adj
    if merge:
        return (int(r) << 16) | (int(g) << 8) | int(b)
    else:
        return int(r), int(g), int(b)
