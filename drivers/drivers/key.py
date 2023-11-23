import logging
import threading
import time

import mraa
import uinput

########## state bit ##########
IDLE = 0b0000_0000
TRIGGERED = 0b1111_0000
_PRESSED_MASK = 0b0000_0001
_RELEASED_MASK = 0b0000_0010
_CLICK_MASK = 0b0000_0100
_LONGPRESS_MASK = 0b0000_1000
_NUMBER_1_MASK = 0b0001_0000
_NUMBER_2_MASK = 0b0010_0000
_NUMBER_3_MASK = 0b0100_0000
# _NUMBER_4_MASK = 0b1000_0000
CLICK = _NUMBER_1_MASK | _CLICK_MASK
DOUBLE = _NUMBER_2_MASK | _CLICK_MASK
TRIPLE = _NUMBER_3_MASK | _CLICK_MASK
LONGPRESS = _NUMBER_1_MASK | _LONGPRESS_MASK
DOUBLE_LONGPRESS = _NUMBER_2_MASK | _LONGPRESS_MASK
TRIPLE_LONGPRESS = _NUMBER_3_MASK | _LONGPRESS_MASK
_FIRST_PRESSED = _NUMBER_1_MASK | _PRESSED_MASK
_FIRST_RELEASED = _NUMBER_1_MASK | _RELEASED_MASK
_SECOND_PRESSED = _NUMBER_2_MASK | _PRESSED_MASK
_SECOND_RELEASED = _NUMBER_2_MASK | _RELEASED_MASK
_THIRD_PRESSED = _NUMBER_3_MASK | _PRESSED_MASK
_THIRD_RELEASED = _NUMBER_3_MASK | _RELEASED_MASK
_NUMBER_MASK = 0b1111_0000
_STATE_MASK = 0b0000_1111
######### default config #########
_CLICK_DELAY = 0.2
_LONG_PRESS_DELAY = 0.5
_REPEAT_WAIT = 0.5
_REPEAT_INTERVAL = 0.1
##########################

logger = logging.getLogger(__name__)


class State(int):
    @staticmethod
    def _get_state_name(state):
        state_name = {
            CLICK: "click",
            DOUBLE: "double",
            TRIPLE: "triple",
            LONGPRESS: "longpress",
            DOUBLE_LONGPRESS: "double_longpress",
            TRIPLE_LONGPRESS: "triple_longpress",
            IDLE: "idle",
            TRIGGERED: "triggered",
        }
        return state_name[state]

    def __str__(self) -> str:
        return self._get_state_name(self)

    def __repr__(self) -> str:
        return str(self)

    @staticmethod
    def from_name(name):
        state_name = {
            "click": CLICK,
            "double": DOUBLE,
            "triple": TRIPLE,
            "longpress": LONGPRESS,
            "double_longpress": DOUBLE_LONGPRESS,
            "triple_longpress": TRIPLE_LONGPRESS,
            "idle": IDLE,
            "triggered": TRIGGERED,
        }
        return State(state_name[name])


class Key_State_Machine(object):
    state = IDLE
    final_state = None  # 最大状态
    repeat_enabled = False  # 替代长按, 重复发送点击事件
    _change_time = time.perf_counter()
    _last_key_value = 0
    _last_send_repeat_time = 0
    _click_delay = _CLICK_DELAY
    _long_press_delay = _LONG_PRESS_DELAY
    _repeat_wait = _REPEAT_WAIT
    _repeat_interval = _REPEAT_INTERVAL

    def __init__(self) -> None:
        pass

    def update(self, key_value: int) -> int:
        t = time.perf_counter()
        last_value = self._last_key_value
        self._last_key_value = key_value
        if key_value == last_value:  # 按键状态保持, 进行超时判断
            if (
                (self.state == _FIRST_PRESSED and not self.repeat_enabled)
                or self.state == _SECOND_PRESSED
                or self.state == _THIRD_PRESSED
            ):  # 长按
                if t - self._change_time > self._long_press_delay:
                    ret = (self.state & _NUMBER_MASK) | _LONGPRESS_MASK
                    self.state = TRIGGERED
                    return ret
            elif self.state == _FIRST_PRESSED:  # 重复点击
                if t - self._change_time > self._repeat_wait:
                    if t - self._last_send_repeat_time > self._repeat_interval:
                        self._last_send_repeat_time = t
                        return CLICK
                return self.state
            elif self.state & _RELEASED_MASK == _RELEASED_MASK:  # 点击
                if t - self._change_time > self._click_delay:
                    ret = (self.state & _NUMBER_MASK) | _CLICK_MASK
                    self.state = IDLE
                    return ret
        else:  # 按键状态变化
            self._change_time = t
            if key_value == 0:  # 按键释放
                if self.state & _PRESSED_MASK == _PRESSED_MASK:
                    self.state = (self.state & _NUMBER_MASK) | _RELEASED_MASK
                if self.state == TRIGGERED:
                    self.state = IDLE
                if (
                    (self.final_state == CLICK and self.repeat_enabled)
                    or self.final_state == LONGPRESS
                ) and self.state == _FIRST_RELEASED:
                    self.state = IDLE
                    return CLICK
                if self.state == _THIRD_RELEASED:
                    self.state = IDLE
                    return TRIPLE
            else:  # 按键按下
                if self.state == IDLE:  # 单击
                    self.state = _FIRST_PRESSED
                    if self.final_state == CLICK and not self.repeat_enabled:
                        self.state = IDLE
                        return CLICK
                elif self.state == _FIRST_RELEASED:  # 双击
                    self.state = _SECOND_PRESSED
                    if self.final_state == DOUBLE:
                        self.state = IDLE
                        return DOUBLE
                elif self.state == _SECOND_RELEASED:  # 三击
                    self.state = _THIRD_PRESSED
                    if self.final_state == TRIPLE:
                        self.state = IDLE
                        return TRIPLE
        return self.state


ARROW_KEYS_MAP = {"up": 32, "down": 40, "left": 35, "right": 37, "middle": 22}


class Key(object):
    # User state
    # CLICK = CLICK
    # CLICK_WITH_LONG = LONGPRESS | CLICK
    # DOUBLE = DOUBLE
    # TRIPLE = TRIPLE
    # LONGPRESS = LONGPRESS
    # DOUBLE_LONGPRESS = DOUBLE_LONGPRESS
    # TRIPLE_LONGPRESS = TRIPLE_LONGPRESS

    def __init__(self, key_map=ARROW_KEYS_MAP, inverted=False):
        self._key_num = len(key_map)
        self._key_map = key_map
        self._key_pins = key_map.values()
        ids = [i for i in range(self._key_num)]
        self._key_name_id_map = dict(zip(key_map.keys(), ids))
        self._key_id_name_map = dict(zip(ids, key_map.keys()))
        self._key_gpio = [mraa.Gpio(pin) for pin in self._key_pins]
        self._inverted = inverted
        self._callbacks = []
        self._callback_states = []
        self._callback_threading = []
        self._global_callback = None
        self._global_callback_threading = False
        self._key_machines = [Key_State_Machine() for _ in range(self._key_num)]
        self._key_state_storage = [IDLE for _ in range(self._key_num)]
        self._callback_threads = []
        threading.Thread(target=self._key_watcher, daemon=True).start()
        logger.debug(f"Key init finished")

    def _read_value(self, key_id):
        value = int(self._key_gpio[key_id].read())
        if self._inverted:
            return 1 - value
        else:
            return value

    def _key_watcher(self):
        while True:
            time.sleep(0.02)
            for n in range(self._key_num):
                value = self._read_value(n)
                state = self._key_machines[n].update(value)
                if state & _CLICK_MASK or state & _LONGPRESS_MASK:
                    self._key_state_storage[n] = state
                    if (n, state) in self._callback_states:
                        index = self._callback_states.index((n, state))
                        if self._callback_threading[index]:
                            self._callback_threads.append(
                                threading.Thread(
                                    target=self._callbacks[index], daemon=True
                                )
                            )
                            self._callback_threads[-1].start()
                        else:
                            self._callbacks[index]()
                    if self._global_callback is not None:
                        if self._global_callback_threading:
                            self._callback_threads.append(
                                threading.Thread(
                                    target=self._global_callback,
                                    args=(self._key_id_name_map[n], State(state)),
                                    daemon=True,
                                )
                            )
                            self._callback_threads[-1].start()
                        else:
                            self._global_callback(self._key_id_name_map[n], State(state))
                    try:
                        while not self._callback_threads[0].is_alive():
                            self._callback_threads.pop(0)
                    except IndexError:
                        pass

    def read(self, key_name) -> State:
        """
        Return last available state of the key
        """
        key_id = self._key_name_id_map[key_name]
        state = self._key_state_storage[key_id]
        self._key_state_storage[key_id] = IDLE
        return State(state)

    def read_value(self, key_name) -> int:
        """
        Read the value of the key
        """
        key_id = self._key_name_id_map[key_name]
        return self._read_value(key_id)

    def read_state(self, key_name) -> State:
        """
        Read current state of the key (for debug)
        """
        key_id = self._key_name_id_map[key_name]
        return State(self._key_machines[key_id].state)

    def set_final_state(self, key_name, state):
        """
        Set the final state of the key
        """
        if isinstance(state, State):
            state = int(state)
        elif isinstance(state, str):
            state = int(State.from_name(state))
        assert state in [CLICK, DOUBLE, TRIPLE, LONGPRESS]
        key_id = self._key_name_id_map[key_name]
        self._key_machines[key_id].final_state = state

    def set_reapeat(self, key_name, enable):
        """
        Set the repeat function of the key
        """
        key_id = self._key_name_id_map[key_name]
        self._key_machines[key_id].repeat_enabled = enable

    def register_callback(self, key_name, callback, state, threading: bool = False):
        """
        Callback params: None
        threading: run callback in a new thread
        """
        if isinstance(state, State):
            state = int(state)
        elif isinstance(state, str):
            state = int(State.from_name(state))
        assert callable(callback), "callback must be callable"
        key_id = self._key_name_id_map[key_name]
        self._callback_states.append((key_id, state))
        self._callbacks.append(callback)
        self._callback_threading.append(bool(threading))

    def register_global_callback(self, callback, threading: bool = False):
        """
        Callback params: key_name, state
        threading: run callback in a new thread
        """
        assert callable(callback), "callback must be callable"
        self._global_callback = callback
        self._global_callback_threading = bool(threading)

    def unregister_callback(self, key_name, state):
        if isinstance(state, State):
            state = int(state)
        elif isinstance(state, str):
            state = int(State.from_name(state))
        key_id = self._key_name_id_map[key_name]
        index = self._callback_states.index((key_id, state))
        self._callback_states.pop(index)
        self._callbacks.pop(index)
        self._callback_threading.pop(index)

    def unregister_global_callback(self):
        self._global_callback = None
        self._global_callback_threading = False

    def set_delay(
        self,
        key_name: str = ...,
        click_delay: float = ...,
        long_press_delay: float = ...,
        repeat_wait: float = ...,
        repeat_interval: float = ...,
    ):
        if key_name is ...:
            target = [i for i in range(self._key_num)]
        else:
            target = [self._key_name_id_map[key_name]]
        for n in target:
            if click_delay is not ...:
                self._key_machines[n]._click_delay = click_delay
            if long_press_delay is not ...:
                self._key_machines[n]._long_press_delay = long_press_delay
            if repeat_wait is not ...:
                self._key_machines[n]._repeat_wait = repeat_wait
            if repeat_interval is not ...:
                self._key_machines[n]._repeat_interval = repeat_interval

    def triggered(self, key_name):
        key_id = self._key_name_id_map[key_name]
        return self._key_machines[key_id].state == TRIGGERED


class Arrow_Keys(object):
    def __init__(self) -> None:
        key_map = ARROW_KEYS_MAP
        self._key_num = len(key_map)
        self._key_map = key_map
        self._key_pins = key_map.values()
        ids = [i for i in range(self._key_num)]
        self._key_name_id_map = dict(zip(key_map.keys(), ids))
        self._key_id_name_map = dict(zip(ids, key_map.keys()))
        self._key_state_storage = [0 for _ in range(self._key_num)]
        self._key_gpio = [mraa.Gpio(pin) for pin in self._key_pins]
        self.event_map = {
            "up": uinput.KEY_UP,
            "down": uinput.KEY_DOWN,
            "left": uinput.KEY_LEFT,
            "right": uinput.KEY_RIGHT,
            "middle": uinput.KEY_ENTER,
        }
        self.dev = uinput.Device(self.event_map.values())
        threading.Thread(target=self._key_watcher, daemon=True).start()
        logger.debug(f"Key init finished")

    def _read_value(self, key_id):
        return int(self._key_gpio[key_id].read())

    def _key_watcher(self):
        while True:
            for n in range(self._key_num):
                state = self._read_value(n)
                if state != self._key_state_storage[n]:
                    self._key_state_storage[n] = state
                    event = self.event_map[self._key_id_name_map[n]]
                    if state == 1:
                        self.dev.emit(event, 1)
                    else:
                        self.dev.emit(event, 0)
            time.sleep(0.01)
