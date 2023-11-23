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
        ctype: Literal["u8", "u16", "u32", "s8", "s16", "s32", "float"] = "u8",
        var_type: type = int,
        value_multiplier: float = 1.0,
    ):
        """Args:
        ctype (str): C-like类型
        py_var_type (_type_): python类型
        value_multiplier (float, optional): 值在从byte向python转换时的乘数.
        """
        self.reset(0, ctype, var_type, value_multiplier)

    def reset(
        self,
        init_value,
        ctype: Literal["u8", "u16", "u32", "s8", "s16", "s32", "float"],
        py_var_type: type,
        value_multiplier: float = 1.0,
    ):
        """重置变量

        Args:
            init_value (_type_): 初始值(浮点值或整数值)
            ctype (str): C-like类型
            py_var_type (_type_): python类型
            value_multiplier (float, optional): 值在从byte向python转换时的乘数.
        """
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]
        if ctype_word_part.lower() == "u":
            self._byte_length = int(int(ctype_number_part) // 8)
            self._signed = False
            self._float = False
        elif ctype_word_part.lower() == "s":
            self._byte_length = int(int(ctype_number_part) // 8)
            self._signed = True
            self._float = False
        elif ctype == "float":
            self._byte_length = 4
            self._signed = True
            self._float = True
        else:
            raise ValueError(f"Invalid ctype: {ctype}")
        if not self._float and int(ctype_number_part) % 8 != 0:
            raise ValueError(f"Invalid ctype: {ctype}")
        if py_var_type not in [int, float, bool]:
            raise ValueError(f"Invalid var_type: {py_var_type}")
        self._var_type = py_var_type
        self._multiplier = value_multiplier
        self._value = self._var_type(init_value)
        return self

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = self._var_type(value)

    def update_value_with_mul(self, value):
        self._value = self._var_type(value * self._multiplier)

    @property
    def bytes(self):
        if self._float:
            return struct.pack("<f", self._value / self._multiplier)
        if self._multiplier != 1:
            return int(round(self._value / self._multiplier)).to_bytes(
                self._byte_length, "little", signed=self._signed
            )
        else:
            return int(self._value).to_bytes(
                self._byte_length, "little", signed=self._signed
            )

    @bytes.setter
    def bytes(self, value):
        if self._float:
            self._value = self._var_type(
                struct.unpack("<f", value)[0] * self._multiplier
            )
        else:
            self._value = self._var_type(
                int.from_bytes(value, "little", signed=self._signed) * self._multiplier
            )

    @property
    def byte_length(self):
        return self._byte_length

    @property
    def struct_fmt_type(self):
        if self._float:
            return "f"
        base_dict = {1: "b", 2: "h", 4: "i", 8: "q"}
        if self._signed:
            return base_dict[self._byte_length]
        else:
            return base_dict[self._byte_length].upper()


class BaseStruct:
    PACK: List[ByteVar] = []  # 存放结构体对象的列表

    def __init__(self):
        self._fmt_string = "<" + "".join([i.struct_fmt_type for i in self.PACK])
        self._fmt_length = struct.calcsize(self._fmt_string)
        self.update_event = Event()

    def update_from_bytes(self, bytes, print_update=False):
        if len(bytes) != self._fmt_length:
            raise ValueError(
                f"Invalid bytes length: {len(bytes)} != {self._fmt_length}"
            )
        vals = struct.unpack(self._fmt_string, bytes)
        for i, val in enumerate(vals):
            self.PACK[i].update_value_with_mul(val)
        self.update_event.set()
        if print_update:
            self.print()

    def print(
        self,
        extra_info: List[str] = [],
        extra_data: List[ByteVar] = [],
    ):
        RED = "\033[1;31m"
        GREEN = "\033[1;32m"
        YELLOW = "\033[1;33m"
        BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        BACK = "\033[F"
        LINELIMIT = 90  # 每行最多显示的字符数
        LOG_SPACE = 1  # 为日志留出的空间
        BOXCOLOR = BLUE
        HEAD = f"{BOXCOLOR}| {RESET}"
        TAIL = f"{BOXCOLOR} |{RESET}"
        lines = [
            BOXCOLOR
            + "-" * ((LINELIMIT - 32) // 2)
            + PURPLE
            + " ▲ System log / ▼ System status "
            + BOXCOLOR
            + "-" * ((LINELIMIT - 32) // 2)
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
            if len_s(lines[-1]) + len_s(vartext) > LINELIMIT - 2:
                lines[-1] += " " * (LINELIMIT - len_s(lines[-1]) - 2) + TAIL
                lines.append(HEAD)
            lines[-1] += vartext
        lines[-1] += " " * (LINELIMIT - len_s(lines[-1]) - 2) + TAIL
        lines.append(f"{BOXCOLOR}{'-' * LINELIMIT}{RESET}")
        for _ in range(LOG_SPACE):
            lines.insert(0, " " * LINELIMIT)
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
            trigger (bool, optional): 回调触发方式 (True为事件置位时触发)
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
