import csv
import datetime
import multiprocessing as mp
import threading as th
import time
from bisect import bisect_left
from typing import Optional

from PyQt5 import QtGui, QtWidgets
from PyQt5.QtCore import Qt

from .interface import request_interface

import pyqtgraph as pg  # isort:skip

# data from datasheet
MAX_SHUT_VOLTS = (0.08192, -0.0819175)
MAX_BUS_VOLT = 36.0
SHUT_VOLT_LSB = 2.5e-6
BUS_VOLT_LSB = 1.25e-3


def _calc_two_complement(word: int) -> int:
    """
    Calculate two's complement
    """
    if word & (1 << 15):
        return -((word ^ 0xFFFF) + 1)
    else:
        return word


class INA226:
    """
    INA226 power monitor driver
    """

    def __init__(
        self,
        address: int = 0x40,
        rshut: float = 0.05,
        software_filter_N: int = 5,
    ) -> None:
        """
        Initialize INA226

        address: I2C address
        rshut: Shunt resistor value in ohm
        software_filter_N: Software filter average number, 0/1 = disabled
        force_current_positive: Force current to be minimum 0
        """
        self._rshut = rshut
        self._sfn = software_filter_N
        self._sf_volt = []
        self._sf_curr = []
        self._sf_power = []
        self._bus = request_interface("i2c", "INA226", address)
        self.reset()
        time.sleep(0.01)
        if self._read_word(0xFE) != 0x5449 or self._read_word(0xFF) != 0x2260:
            raise RuntimeError("Device is not INA226")
        self._last_confreg_data = self._read_word(0x00)
        self._hardware_current_limit = MAX_SHUT_VOLTS[0] / rshut
        self._alert_func = None
        self.set_current_measurement_range(self._hardware_current_limit * 0.8)

    def _write_word(self, reg: int, data: int) -> None:
        dl = [data >> 8, data & 0xFF]
        self._bus.write_reg_data(reg, dl)

    def _read_word(self, reg: int) -> int:
        dl = self._bus.read_reg_data(reg, 2)
        return (dl[0] << 8) | dl[1]

    def reset(self) -> None:
        """
        Reset device
        """
        self._write_word(0x00, 1 << 15)

    def set_averaging_mode(self, mode: int) -> None:
        """
        Set averaging mode (0-5)

        Num = 4 ^ mode
        Default: 0 (1 sample)
        """
        if mode not in range(6):
            raise ValueError("Invalid averaging mode")
        data = self._read_word(0x00)
        data &= ~(0b111 << 9)
        data |= mode << 9
        self._write_word(0x00, data)
        self._last_confreg_data = data

    def set_bus_conversion_time(self, time: int) -> None:
        """
        Set bus voltage conversion time (0-7)

        conversion time = [140, 204, 332, 588, 1100, 2116, 4156, 8244]us
        Default: 4 (1100us)
        """
        if time not in range(8):
            raise ValueError("Invalid bus voltage conversion time")
        data = self._read_word(0x00)
        data &= ~(0b111 << 6)
        data |= time << 6
        self._write_word(0x00, data)
        self._last_confreg_data = data

    def set_shunt_conversion_time(self, time: int) -> None:
        """
        Set shunt voltage conversion time (0-7)

        conversion time = [140, 204, 332, 588, 1100, 2116, 4156, 8244]us
        Default: 4 (1100us)
        """
        if time not in range(8):
            raise ValueError("Invalid shunt voltage conversion time")
        data = self._read_word(0x00)
        data &= ~(0b111 << 3)
        data |= time << 3
        self._write_word(0x00, data)
        self._last_confreg_data = data

    def set_conversion_mode(self, mode: int) -> None:
        """
        Set conversion mode and target (0-7)

        For mode b2 b1 b0:
        b2 = 1: Continuous, 0: Triggered
        b1 = Bus voltage conversion, 1: Enabled, 0: Disabled
        b0 = Shunt voltage conversion, 1: Enabled, 0: Disabled
        mode 0: Power-down
        Default: 0b111 (Continuous, Bus and Shunt enabled)
        """
        if mode not in range(8):
            raise ValueError("Invalid mode")
        data = self._read_word(0x00)
        data &= ~(0b111)
        data |= mode
        self._write_word(0x00, data)
        self._last_confreg_data = data

    def trigger_single_conversion(self) -> None:
        """
        Trigger single conversion
        """
        self._write_word(0x00, self._last_confreg_data)

    @property
    def conversion_time(self) -> float:
        """
        Calculate total conversion time in seconds

        Return = (bus_time + shunt_time) * avg_num
        """
        data = self._read_word(0x00)
        bus_time = [140, 204, 332, 588, 1100, 2116, 4156, 8244]
        shunt_time = [140, 204, 332, 588, 1100, 2116, 4156, 8244]
        avg_num = 4 ** ((data >> 9) & 0b111)
        return (
            (bus_time[(data >> 6) & 0b111] + shunt_time[(data >> 3) & 0b111]) * avg_num
        ) / 1e6

    @property
    def shunt_voltage(self) -> float:
        """
        Get shunt voltage (can be negative)

        Without software filter
        """
        data = self._read_word(0x01)
        return _calc_two_complement(data) * SHUT_VOLT_LSB

    @property
    def bus_voltage(self) -> float:
        """
        Get bus voltage (always positive)

        Without software filter
        """
        data = self._read_word(0x02)
        return data * BUS_VOLT_LSB

    @property
    def voltage(self) -> float:
        """
        Get bus voltage (always positive)
        """
        if self._sfn < 2:
            return self.bus_voltage
        else:
            self._sf_volt.append(self.bus_voltage)
            self._sf_volt = self._sf_volt[-self._sfn :]
            return sum(self._sf_volt) / len(self._sf_volt)

    @property
    def power(self) -> float:
        """
        Get power (always positive)
        """
        val = self._read_word(0x03) * self._power_lsb
        if self._sfn > 1:
            self._sf_power.append(val)
            self._sf_power = self._sf_power[-self._sfn :]
            val = sum(self._sf_power) / len(self._sf_power)
        return val

    @property
    def current(self) -> float:
        """
        Get current (can be negative)
        """
        val = _calc_two_complement(self._read_word(0x04)) * self._current_lsb
        if self._sfn > 1:
            self._sf_curr.append(val)
            self._sf_curr = self._sf_curr[-self._sfn :]
            val = sum(self._sf_curr) / len(self._sf_curr)
        return val

    def set_current_measurement_range(self, value: float) -> float:
        """
        Current measurement range in A (Affects current accuracy)

        Write calibration register (0x05)
        Returns actual current measurement range
        """
        if value > self._hardware_current_limit:
            raise ValueError(
                f"Current measurement range should be less than {self._hardware_current_limit}A (hardware limit)"
            )
        current_lsb = value / (2**15)
        self._calreg_data = int(0.00512 / (current_lsb * self._rshut))
        self._write_word(0x05, self._calreg_data)
        if (rd := self._read_word(0x05)) != self._calreg_data:
            raise RuntimeError(
                f"Failed to set calibration register ({rd} != {self._calreg_data}))"
            )
        self._current_lsb = 0.00512 / (self._calreg_data * self._rshut)  # true value
        self._power_lsb = 25 * self._current_lsb
        return self._current_lsb * (2**15)

    @property
    def conversion_ready(self) -> bool:
        """
        Check if conversion is ready
        """
        data = self._read_word(0x06)
        return bool(data & (1 << 3))

    @property
    def alert_source_is_alert_function(self) -> bool:
        """
        Check if alert source is alert function

        Useful if alert function and data ready function are both enabled
        """
        data = self._read_word(0x06)
        return bool(data & (1 << 5))

    @property
    def alert_source_is_conversion_ready(self) -> bool:
        """
        Check if alert source is conversion ready

        Useful if alert function and data ready function are both enabled
        """
        return not self.alert_source_is_alert_function

    @property
    def math_overflow(self) -> bool:
        """
        Check if overflow error occurred
        """
        data = self._read_word(0x06)
        return bool(data & (1 << 2))

    def set_alert_polarity(self, active_high: bool) -> None:
        """
        Set alert pin polarity

        Default: False (active low)
        """
        data = self._read_word(0x06)
        if active_high:
            data |= 1 << 1
        else:
            data &= ~(1 << 1)
        self._write_word(0x06, data)

    def set_alert_latch(self, latch: bool) -> None:
        """
        Set alert latch

        If latch is True, alert pin will remain active \
        even if alert condition is cleared
        Default: False (not latched)
        """
        data = self._read_word(0x06)
        if latch:
            data |= 1 << 0
        else:
            data &= ~(1 << 0)
        self._write_word(0x06, data)

    def enable_alert_function(self, function: int) -> None:
        """
        Set alert function

        0: Power Over-Limit
        1: Bus Voltage Under-Voltage
        2: Bus Voltage Over-Voltage
        3: Shunt Voltage Under-Voltage
        4: Shunt Voltage Over-Voltage

        Default: N/A (disabled)
        """
        if function not in range(5):
            raise ValueError("Invalid alert function")
        data = self._read_word(0x06)
        data &= ~(0b11111 << 11)
        data |= 1 << (function + 11)
        self._write_word(0x06, data)
        self._alert_func = function

    def disable_alert_function(self) -> None:
        """
        Disable alert function
        """
        data = self._read_word(0x06)
        data &= ~(0b11111 << 11)
        self._write_word(0x06, data)
        self._alert_func = None

    def set_alert_data_ready_function(self, enable: bool) -> None:
        """
        Enable or disable alert function for data ready

        Default: False (disabled)
        """
        data = self._read_word(0x06)
        if enable:
            data |= 1 << 10
        else:
            data &= ~(1 << 10)
        self._write_word(0x06, data)

    def set_alert_limit(self, limit: float) -> None:
        """
        Set alert limit

        Call enable_alert_function() first to determine data ratio
        Default: 0.0
        """
        if self._alert_func is None:
            raise RuntimeError("Set alert function first before setting alert limit")
        if self._alert_func == 0:
            limit /= self._power_lsb
        elif self._alert_func in [1, 2]:
            limit /= BUS_VOLT_LSB
        elif self._alert_func in [3, 4]:
            limit /= SHUT_VOLT_LSB
        else:
            raise RuntimeError("Invalid alert function")
        self._write_word(0x07, int(limit))

    def set_alert_limit_current_to_shunt(self, current_limit: float) -> None:
        """
        Convert current limit to shunt voltage limit and set alert limit
        """
        if self._alert_func not in [3, 4]:
            raise RuntimeError(
                "Alert function should be Shunt Voltage Under-Voltage or Shunt Voltage Over-Voltage"
            )
        limit = (current_limit / self._current_lsb) * 2048 / self._calreg_data
        self._write_word(0x07, int(limit))


class _INA226MonitorMultiprocWorker(mp.Process):
    def __init__(
        self,
        shared_queue: mp.Queue,
        stop_event,
        open_event,
        view_time,
        save_data,
        topmost,
    ):
        self.shared_queue = shared_queue
        self.stop_event = stop_event
        self.open_event = open_event
        self.view_time = view_time
        self.save_data = save_data
        self.topmost = topmost
        if self.save_data:
            self.last_save_time = time.perf_counter()
            self.filename = (
                f"./ina226_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            with open(self.filename, "w", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(
                    ["Time (s)", "Voltage (V)", "Current (A)", "Power (W)", "Marker"]
                )
            self.new_data = []
        self.time_data = []
        self.voltage_data = []
        self.current_data = []
        self.power_data = []
        self.run()
        exit(0)

    def strf(self, value: float, unit: str, accuracy=6) -> str:
        if value < 1e-3:
            s = f"{value*1e6:.{accuracy}f} u{unit}"
        elif value < 1:
            s = f"{value*1e3:.{accuracy}f} m{unit}"
        elif value < 1e3:
            s = f"{value:.{accuracy}f} {unit}"
            unit += " "
        elif value < 1e6:
            s = f"{value/1e3:.{accuracy}f} k{unit}"
        else:
            s = f"{value/1e6:.{accuracy}f} M{unit}"
        if len(s) < len(unit) + accuracy + 7:
            s += " " * (len(unit) + accuracy + 7 - len(s))
        return s.replace(" ", "&nbsp;")

    def update(self):
        if not self.open_event.is_set():
            self.open_event.set()
        if self.stop_event.is_set() or self.wg.isHidden():
            self.timer.stop()
            self.wg.close()
            self.app.exit()
            return
        if self.shared_queue.empty():
            return
        while not self.shared_queue.empty():
            data = self.shared_queue.get()
            if self.save_data:
                self.new_data.append(data)
            self.time_data.append(data[0])
            self.voltage_data.append(data[1])
            self.current_data.append(data[2])
            self.power_data.append(data[3])
            if data[4] != "":
                self.add_marker(*data)
        self.voltage_line.setData(self.time_data, self.voltage_data)
        self.current_line.setData(self.time_data, self.current_data)
        self.power_line.setData(self.time_data, self.power_data)
        if not self.plt_volt.mouseHovering:
            self.plt_volt.setTitle(f"Voltage: {self.strf(self.voltage_data[-1], 'V')}")
        if not self.plt_curr.mouseHovering:
            self.plt_curr.setTitle(f"Current: {self.strf(self.current_data[-1], 'A')}")
        idx = bisect_left(self.time_data, self.time_data[-1] - self.view_time)
        power = self.power_data[idx:]
        if not self.plt_power.mouseHovering:
            self.plt_power.setTitle(
                f"Power: {self.strf(self.power_data[-1], 'W')} Max: {self.strf(max(power), 'W')} Min: {self.strf(min(power), 'W')}",
            )
        title = f"INA226 Monitor ( Time: {self.time_data[-1]:.2f} / Sample: {len(self.time_data)}"
        if len(self.time_data) > 101:
            title += f" / SPS: {100/(self.time_data[-1]-self.time_data[-101]):.2f}"
        title += " )"
        self.wg.setWindowTitle(title)
        self.auto_range(self.plt_volt, self.time_data, self.voltage_data, idx)
        self.auto_range(self.plt_curr, self.time_data, self.current_data, idx)
        self.auto_range(self.plt_power, self.time_data, self.power_data, idx)
        if self.save_data and time.perf_counter() - self.last_save_time > 1:
            self.last_save_time = time.perf_counter()
            with open(self.filename, "a", newline="") as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self.new_data)
                self.new_data = []

    def auto_range(self, plt, xdata, ydata, idx, padding=0.05):
        if not plt.mouseHovering:
            plt.setXRange(xdata[-1] - self.view_time, xdata[-1])
            ydata = ydata[idx:]
            max_val = max(ydata)
            min_val = min(ydata)
            diff = max_val - min_val
            if max_val != min_val:
                plt.setYRange(min_val - diff * padding, max_val + diff * padding)

    def run(self):
        self.app = QtWidgets.QApplication([])
        self.wg = pg.GraphicsLayoutWidget()
        self.wg.setAntialiasing(True)
        self.wg.resize(800, 600)
        self.wg.setWindowTitle("INA226 Monitor")
        self.plt_volt = pg.PlotItem()
        self.plt_curr = pg.PlotItem()
        self.plt_power = pg.PlotItem()
        font = QtGui.QFont("Consolas", 12)
        self.plt_volt.titleLabel.item.setFont(font)
        self.plt_curr.titleLabel.item.setFont(font)
        self.plt_power.titleLabel.item.setFont(font)
        font = QtGui.QFont("Consolas", 9)
        self.plt_volt.setLabel("left", "Voltage", "V")
        self.plt_curr.setLabel("left", "Current", "A")
        self.plt_power.setLabel("left", "Power", "W")
        self.plt_volt.getAxis("left").label.setFont(font)
        self.plt_curr.getAxis("left").label.setFont(font)
        self.plt_power.getAxis("left").label.setFont(font)
        layout = pg.GraphicsLayout()
        layout.addItem(self.plt_volt, 0, 0)
        layout.addItem(self.plt_curr, 0, 1)
        layout.addItem(self.plt_power, 1, 0, 1, 2)
        self.wg.addItem(layout)
        self.plt_volt.showGrid(x=True, y=True)
        self.plt_curr.showGrid(x=True, y=True)
        self.plt_power.showGrid(x=True, y=True)
        self.voltage_line = self.plt_volt.plot(
            pen=pg.mkPen(color=(255, 50, 20), width=1)
        )
        self.current_line = self.plt_curr.plot(
            pen=pg.mkPen(color=(50, 255, 20), width=1)
        )
        self.power_line = self.plt_power.plot(
            pen=pg.mkPen(color=(255, 180, 30), width=1)
        )
        self.timer = pg.QtCore.QTimer()  # type: ignore
        self.timer.timeout.connect(self.update)
        self.timer.start(20)  # 50 Hz
        if self.topmost:
            self.wg.setWindowFlags(self.wg.windowFlags() | Qt.WindowStaysOnTopHint)  # type: ignore
        self.wg.show()
        self.app.exec()

    def add_marker(self, time, volt, curr, power, marker):
        v_line = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color=(60, 200, 255), width=1),
            label=f"{marker}\n{time:.3f}s\n{self.strf(volt, 'V',3)}",
            labelOpts={
                "position": 0.9,
                "color": (80, 200, 255),
                "movable": False,
            },
        )
        c_line = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color=(60, 200, 255), width=1),
            label=f"{marker}\n{time:.3f}s\n{self.strf(curr, 'A',3)}",
            labelOpts={
                "position": 0.9,
                "color": (80, 200, 255),
                "movable": False,
            },
        )
        p_line = pg.InfiniteLine(
            angle=90,
            movable=False,
            pen=pg.mkPen(color=(60, 200, 255), width=1),
            label=f"{marker}\n{time:.3f}s\n{self.strf(power, 'W',3)}",
            labelOpts={
                "position": 0.9,
                "color": (80, 200, 255),
                "movable": False,
            },
        )
        self.plt_volt.addItem(v_line, ignoreBounds=True)
        self.plt_curr.addItem(c_line, ignoreBounds=True)
        self.plt_power.addItem(p_line, ignoreBounds=True)
        v_line.setPos(time)
        c_line.setPos(time)
        p_line.setPos(time)


class INA226Monitor:
    def __init__(
        self,
        save_data: bool = False,
        view_time: float = 10,
        force_positive=True,
        calc_power_by_cv=False,
        topmost=True,
        wait_for_window_open=True,
        *,
        ina: Optional[INA226] = None,
        software_filter_N: int = 1,
        measurement_range: float = 0.5,
        averaging_mode: int = 2,
        convert_time: int = 2,
    ):
        if ina is None:
            self._ina = INA226(software_filter_N=software_filter_N)
            self._ina.set_current_measurement_range(measurement_range)
            self._ina.set_averaging_mode(averaging_mode)
            self._ina.set_bus_conversion_time(convert_time)
            self._ina.set_shunt_conversion_time(convert_time)
        else:
            self._ina = ina
        self._shared_queue = mp.Queue(2048)
        self._worker_stop_event = mp.Event()
        self._window_open_event = mp.Event()
        self._worker = mp.Process(
            target=_INA226MonitorMultiprocWorker,
            args=(
                self._shared_queue,
                self._worker_stop_event,
                self._window_open_event,
                view_time,
                save_data,
                topmost,
            ),
            daemon=True,
        )
        self._update_th = th.Thread(target=self._update_thread, daemon=True)
        self._marker = ""
        self._force_positive = force_positive
        self._calc_power_by_cv = calc_power_by_cv

    def _update_thread(self):
        start_time = time.perf_counter()
        while self.is_alive():
            try:
                while not self._ina.conversion_ready:
                    time.sleep(0.001)
                v, c = self._ina.voltage, self._ina.current
                p = self._ina.power if not self._calc_power_by_cv else v * c
                t = time.perf_counter() - start_time
                if self._force_positive and c <= 0:
                    c = 0
                    p = 0
                if self._shared_queue.full():
                    self._shared_queue.get()
                self._shared_queue.put((t, v, c, p, self._marker))
                self._marker = ""
            except Exception as e:
                print(e)
                time.sleep(0.1)

    def start(self):
        if not self.is_alive():
            self._worker_stop_event.clear()
            self._window_open_event.clear()
            self._worker.start()
            if not self._window_open_event.wait(8) or not self._worker.is_alive():
                raise RuntimeError(
                    "Failed to start worker process, check if you forgot to wrap your code in if __name__ == '__main__':"
                )
            self._update_th.start()
        return self

    def stop(self):
        self._worker_stop_event.set()
        self._worker.join(1)
        self._worker.kill()
        self._update_th.join()

    def add_marker(self, marker: str):
        self._marker = marker

    def is_alive(self) -> bool:
        return self._worker.exitcode is None and self._worker.is_alive()

    @property
    def ina226(self) -> INA226:
        return self._ina
