import threading
import time
from typing import Dict

from loguru import logger
from pynmeagps import NMEAMessage, NMEAReader, exceptions

from drivers.interface import InterfaceManager

from .status import GGA_Status, RMC_Status, VTG_Status
from .utils import ZKW_Commands, add_nema_checksum


class GPS(object):
    # Public
    last_messages: Dict[str, NMEAMessage] = {}
    RMC: RMC_Status = RMC_Status()
    GGA: GGA_Status = GGA_Status()
    VTG: VTG_Status = VTG_Status()
    # Private
    _status_talker_filter = ("GP", "GN", "GB", "GL")
    _COMMANDS = ZKW_Commands
    _fixed = 0
    _module_connected = False

    def __init__(self, baudrate=115200, start_listening=True):
        # self._serial = serial.Serial(port, baudrate, timeout=2)
        self._ser = InterfaceManager.request_uart_interface("GPS", baudrate)
        # self._sio = io.TextIOWrapper(io.BufferedRWPair(self._serial, self._serial))
        # Giveup io.TextIOWrapper, it's too slow
        self._listen_thread = None
        self.running = False
        if start_listening:
            self.listen()

    def send_command(self, command, add_checksum=True):
        """
        Send a command to the GPS module

        add_checksum: Add checksum to the command
        """
        if add_checksum:
            command = add_nema_checksum(command)
        self._ser.write(command.encode("ascii"))
        # self._sio.write(command)
        # self._sio.flush()

    def switch_baudrate(self, baudrate, save=True):
        """
        Set the baudrate of the GPS module
        Available baudrates: 4800, 9600, 19200, 38400, 57600, 115200

        save: Save the baudrate to the module's memory
        """
        assert baudrate in self._COMMANDS.AVAILABLE_BAUDRATES
        command = getattr(self._COMMANDS, f"BAUDRATE_{baudrate}")
        for baud in self._COMMANDS.AVAILABLE_BAUDRATES:
            # We dont know the current baudrate, so we try all of them
            if baud == baudrate:
                continue
            self._ser.set_baudrate(baud)
            self.send_command(command)
        self._ser.set_baudrate(baudrate)
        if save:
            self.save_parms()
        logger.info(f"Switched baudrate to {baudrate}")

    def set_output_rate(self, rate_hz):
        """
        Set the output rate of the GPS module
        Available rates: 1, 2, 4, 5, 10
        """
        assert rate_hz in self._COMMANDS.AVAILABLE_OUTPUT_RATES
        command = getattr(self._COMMANDS, f"OUTPUT_{rate_hz}HZ")
        self.send_command(command)

    def save_parms(self):
        """
        Save the current settings to the GPS module's memory
        """
        self.send_command(self._COMMANDS.SAVE_PARMS)

    def reboot(self, mode="HOT"):
        """
        Reboot the GPS module
        Modes: HOT, WARM, COLD, FACTORY
        """
        command = getattr(self._COMMANDS, f"BOOT_{mode.upper()}")
        self.send_command(command)

    def set_nema_talkers(self, GPS=True, BEIDOU=True, GLONASS=True):
        """
        Set the NEMA talkers
        """
        command = self._COMMANDS.SET_NEMA_TALKERS(GPS, BEIDOU, GLONASS)
        self.send_command(command)

    def set_status_talker_filter(self, GPS=True, BEIDOU=True, GLONASS=True):
        """
        Set the talkers to be ignored when updating the status
        """
        filters = []
        if GPS:
            filters.append("GP")
            filters.append("GN")
        if BEIDOU:
            filters.append("GB")
        if GLONASS:
            filters.append("GL")
        self._status_talker_filter = tuple(filters)

    def _update_status(self, pack: NMEAMessage):
        if pack.talker not in self._status_talker_filter:
            return
        if pack.msgID == "RMC":
            self.RMC.update(pack)
        elif pack.msgID == "GGA":
            self.GGA.update(pack)
            if self.GGA.fix != self._fixed:
                self._fixed = self.GGA.fix
                if self._fixed == 0:
                    logger.warning("GPS lost")
                elif self._fixed == 1:
                    logger.info("GPS fixed")
                elif self._fixed == 6:
                    logger.info("GPS fixed (DGPS)")
        elif pack.msgID == "VTG":
            self.VTG.update(pack)

    def _listen_worker(self):
        inited = False
        buf = bytes()
        last_read = time.perf_counter()
        while self.running:
            try:
                try:
                    while self._ser.in_waiting:
                        buf += self._ser.read(self._ser.in_waiting)
                        last_read = time.perf_counter()
                    if self._module_connected and time.perf_counter() - last_read > 2:
                        logger.warning("GPS module disconnected")
                        self._module_connected = False
                    idx = buf.find(b"\n")
                    if idx == -1:
                        time.sleep(0.01)
                        continue
                    line = buf[:idx].decode("ascii")
                    buf = buf[idx + 1 :]
                except UnicodeDecodeError:
                    logger.warning("Failed to decode line")
                    continue
                if not inited:
                    inited = True
                    continue  # Skip first line
                line = line.strip()
                if line[:3] == "$BD":  # Beidou adopt to NEMA4.0
                    checksum = int(line[-2:], 16)
                    checksum ^= ord("G") ^ ord("D")  # Fix checksum
                    line = "$GB" + line[3:-2] + f"{checksum:02X}"
                pack: NMEAMessage = NMEAReader.parse(line)  # type: ignore
                if not self._module_connected:
                    logger.info("GPS module connected")
                    self._module_connected = True
                # logger.debug(pack)
                id = pack.talker + pack.msgID
                self.last_messages[id] = pack
                self._update_status(pack)
            except exceptions.NMEAParseError as e:
                logger.debug(f"GPS parse error: {e}")
                continue
            except Exception as e:
                logger.exception(e)
        logger.debug("Listener thread stopped")

    def listen(self):
        """
        Start listening to the GPS module
        """
        self.running = True
        self._listen_thread = threading.Thread(target=self._listen_worker, daemon=True)
        self._listen_thread.start()
        logger.info("Started listening to GPS module")

    def wait_for_connection(self, timeout=-1):
        self._module_connected = False
        t0 = time.time()
        info_printed = False
        while not self._module_connected:
            if timeout > 0 and time.time() - t0 > timeout:
                logger.error("Wait for GPS module connection timeout")
                raise TimeoutError
            if not info_printed and time.time() - t0 > 1:
                logger.info("Waiting for GPS module connection")
                info_printed = True

    @property
    def connected(self):
        return self._module_connected
