import ctypes
import os
import platform
from typing import Callable, Literal, Optional, Tuple

from loguru import logger

from .ch347_dll import DeviceInfo, EEPROMType, SPIConfig, init_ch347_dll

_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib")
if platform.architecture()[0] == "64bit":
    _LIBPATH = os.path.join(_PATH, "CH347DLLA64.DLL")
else:
    _LIBPATH = os.path.join(_PATH, "CH347DLL.DLL")

# Define the callback function type
INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
NOTIFY_ROUTINE = ctypes.CFUNCTYPE(None, ctypes.c_ulong)
GPIO_INT_ROUTINE = ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_ubyte))


class CH347:
    def __init__(self) -> None:
        self._dev = init_ch347_dll(_LIBPATH)
        # 创建回调函数对象并绑定到实例属性
        self._callback_funcs = [NOTIFY_ROUTINE(self.default_event_callback)]

    def list_devices(self) -> list[DeviceInfo]:
        dev_infos = []
        print("Listing devices:")
        print("-" * 40)
        for i in range(8):
            if self._dev.CH347OpenDevice(i) == INVALID_HANDLE_VALUE:
                break
            dev_info = DeviceInfo()
            if self._dev.CH347GetDeviceInfor(i, ctypes.byref(dev_info)):
                dev_infos.append(dev_info)
                for field_name, _ in dev_info._fields_:
                    value = getattr(dev_info, field_name)
                    print(f"{field_name}: {value}")
            print("-" * 40)
            self._dev.CH347CloseDevice(i)
        print(f"Number of devices: {len(dev_infos)}")
        return dev_infos

    @staticmethod
    def default_event_callback(iEventStatus) -> None:
        logger.debug("Callback event status:", iEventStatus)
        if iEventStatus == 0:
            logger.warning("Device unplugged")
        elif iEventStatus == 3:
            logger.info("Device inserted")

    def open_device(self, device_index: int = 0) -> Optional[int]:
        """
        Open USB device.

        Args:
            device_index (int, optional): Device index. Default is 0.

        Returns:
            int: Handle to the opened device if successful, None otherwise.
        """
        self._device_index = device_index
        handle = self._dev.CH347OpenDevice(self._device_index)
        if handle != INVALID_HANDLE_VALUE:
            return handle
        else:
            return None

    def close_device(self) -> bool:
        """
        Close USB device.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347CloseDevice(self._device_index)
        return result

    def get_device_info(self) -> Optional[DeviceInfo]:
        """
        Get device information.

        Returns:
            bool: True if successful, False otherwise.
            DeviceInfo: Device information.
        """
        dev_info = DeviceInfo()
        result = self._dev.CH347GetDeviceInfor(
            self._device_index, ctypes.byref(dev_info)
        )
        if result:
            return dev_info
        else:
            return None

    def get_version(self) -> Optional[Tuple[int, int, int, int]]:
        """
        Obtain driver version, library version, device version, and chip type.

        This method retrieves various versions related to the CH347 device and returns them as a tuple.

        Returns:
            tuple or None: A tuple containing the following information if successful:
                - driver_ver (int): The driver version.
                - dll_ver (int): The library version.
                - device_ver (int): The device version.
                - chip_type (int): The chip type.
            Returns None if the retrieval fails.
        """
        # Create variables to store the version information
        driver_ver = ctypes.c_ubyte()
        dll_ver = ctypes.c_ubyte()
        device_ver = ctypes.c_ubyte()
        chip_type = ctypes.c_ubyte()

        # Call the CH347GetVersion function
        result = self._dev.CH347GetVersion(
            self._device_index,
            ctypes.byref(driver_ver),
            ctypes.byref(dll_ver),
            ctypes.byref(device_ver),
            ctypes.byref(chip_type),
        )
        if result:
            return driver_ver.value, dll_ver.value, device_ver.value, chip_type.value
        else:
            return None

    def set_device_notify(
        self, iDeviceID: str, iNotifyRoutine: Optional[Callable] = None
    ) -> bool:
        """
        Configure device event notifier.

        Args:
            iDeviceID (str): Optional parameter specifying the ID of the monitored device.
            iNotifyRoutine (callable): Callback function to handle device events.

        Returns:
            bool: True if successful, False otherwise.
        """
        if iNotifyRoutine is None:
            callback = self._callback_funcs[0]
        else:
            callback = NOTIFY_ROUTINE(iNotifyRoutine)
            self._callback_funcs.append(callback)
        str_device_id = ctypes.c_char_p(iDeviceID.encode("utf-8"))
        result = self._dev.CH347SetDeviceNotify(
            self._device_index, str_device_id, callback
        )
        return result

    def read_data(self, oBuffer, ioLength) -> bool:
        """
        Read USB data block.

        Args:
            oBuffer (ctypes.c_void_p): Pointer to a buffer to store the read data.
            ioLength (ctypes.POINTER(ctypes.c_ulong)): Pointer to the length unit. Contains the length to be read as input and the actual read length after return.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347ReadData(self._device_index, oBuffer, ioLength)
        return result

    def write_data(self, iBuffer, ioLength) -> bool:
        """
        Write USB data block.

        Args:
            iBuffer (ctypes.c_void_p): Pointer to a buffer containing the data to be written.
            ioLength (ctypes.POINTER(ctypes.c_ulong)): Pointer to the length unit. Input length is the intended length, and the return length is the actual length.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347WriteData(self._device_index, iBuffer, ioLength)
        return result

    def set_timeout(self, iWriteTimeout: int, iReadTimeout: int) -> bool:
        """
        Set the timeout of USB data read and write.

        Args:
            iWriteTimeout (int): Timeout for USB to write data blocks, in milliseconds. Use 0xFFFFFFFF to specify no timeout (default).
            iReadTimeout (int): Timeout for USB to read data blocks, in milliseconds. Use 0xFFFFFFFF to specify no timeout (default).

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347SetTimeout(
            self._device_index, iWriteTimeout, iReadTimeout
        )
        return result

    def spi_init_with_config(self, spi_config: SPIConfig) -> bool:
        """
        Initialize the SPI Controller.

        Args:
            spi_config (SPIConfig): The configuration for the SPI controller.

        Returns:
            bool: True if initialization is successful, False otherwise.
        """
        result = self._dev.CH347SPI_Init(self._device_index, ctypes.byref(spi_config))
        return result

    def spi_init(
        self,
        mode: Literal[0, 1, 2, 3] = 0,
        clock: Literal[0, 1, 2, 3, 4, 5, 6, 7] = 0,
        byte_order: Literal[0, 1] = 0,
        write_read_interval: int = 0,
        default_data: int = 0,
        chip_select: int = 0,
        cs1_polarity: int = 0,
        cs2_polarity: int = 0,
        is_auto_deactive_cs: int = 0,
        active_delay: int = 0,
        delay_deactive: int = 0,
    ) -> bool:
        """
        Initialize the SPI Controller.

        Args:
            mode (int, optional): SPI mode. 0 = Mode0, 1 = Mode1, 2 = Mode2, 3 = Mode3. Default is 0.
            clock (int, optional): SPI clock frequency. 0 = 60MHz, 1 = 30MHz, 2 = 15MHz, 3 = 7.5MHz, 4 = 3.75MHz,
                                   5 = 1.875MHz, 6 = 937.5KHz, 7 = 468.75KHz. Default is 0.
            byte_order (int, optional): SPI byte order. 0 = LSB first, 1 = MSB first. Default is 0.
            write_read_interval (int, optional): Regular interval for SPI read/write commands, in microseconds.
                                                 Default is 0.
            default_data (int, optional): Default output data when reading from SPI. Default is 0.
            chip_select (int, optional): Chip select control. Bit 7 as 0 ignores chip select control,
                                          Bit 7 as 1 makes the parameters valid:
                                          Bit 1 and Bit 0 as 00/01 selects CS1/CS2 pin as the active low chip select.
                                          Default is 0.
            cs1_polarity (int, optional): CS1 polarity control. 0 = active low, 1 = active high. Default is 0.
            cs2_polarity (int, optional): CS2 polarity control. 0 = active low, 1 = active high. Default is 0.
            is_auto_deactive_cs (int, optional): Automatically de-assert chip select after the operation is completed.
                                                 Default is 0.
            active_delay (int, optional): Delay time for executing read/write operations after chip select is set,
                                          in microseconds. Default is 0.
            delay_deactive (int, optional): Delay time for executing read/write operations after chip select is
                                            de-asserted, in microseconds. Default is 0.

        Returns:
            bool: True if initialization is successful, False otherwise.
        """
        spi_config = SPIConfig()
        spi_config.Mode = mode
        spi_config.Clock = clock
        spi_config.ByteOrder = byte_order
        spi_config.SPIWriteReadInterval = write_read_interval
        spi_config.SPIOutDefaultData = default_data
        spi_config.ChipSelect = chip_select
        spi_config.CS1Polarity = cs1_polarity
        spi_config.CS2Polarity = cs2_polarity
        spi_config.IsAutoDeativeCS = is_auto_deactive_cs
        spi_config.ActiveDelay = active_delay
        spi_config.DelayDeactive = delay_deactive
        result = self._dev.CH347SPI_Init(self._device_index, ctypes.byref(spi_config))
        return result

    def spi_get_cfg(self) -> Optional[SPIConfig]:
        """
        Get SPI controller configuration information.

        Returns:
            tuple: A tuple containing a boolean value indicating if the operation was successful
            and the SPI configuration structure.

            The first element (bool) represents whether the operation was successful or not.
            - True: The operation was successful.
            - False: The operation failed.

            The second element (SPIConfig): An instance of the SPIConfig class, representing the
            SPI configuration structure. If the operation was successful, this object will contain
            the configuration information retrieved from the SPI controller. Otherwise, it will be
            an empty object with default values.
        """
        spi_config = SPIConfig()
        result = self._dev.CH347SPI_GetCfg(self._device_index, ctypes.byref(spi_config))
        if result:
            return spi_config
        else:
            return None

    def spi_change_cs(self, iStatus: int) -> bool:
        """
        Change the chip selection status.

        Args:
            iStatus (int): Chip selection status. 0 = Cancel the piece to choose, 1 = Set piece selected.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347SPI_ChangeCS(self._device_index, iStatus)
        return result

    def spi_set_databits(self, iDataBits: Literal[0, 1]) -> bool:
        """
        Set the number of data bits.

        Args:
            iDataBits (int): Number of data bits. 0 = 8 bits, 1 = 16 bits.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347SPI_SetDataBits(self._device_index, iDataBits)
        return result

    def spi_set_chip_select(
        self,
        iEnableSelect: int,
        iChipSelect: int,
        iIsAutoDeativeCS: int,
        iActiveDelay: int,
        iDelayDeactive: int,
    ) -> bool:
        """
        Set SPI slice selection.

        Args:
            iEnableSelect (int): Enable selection status. The lower octet is CS1 and the higher octet is CS2.
                                A byte value of 1 sets CS, 0 ignores this CS setting.
            iChipSelect (int): Chip selection status. The lower octet is CS1 and the higher octet is CS2.
                               A byte value of 1 sets CS, 0 ignores this CS setting.
            iIsAutoDeativeCS (int): Auto deactivation status. The lower 16 bits are CS1 and the higher 16 bits are CS2.
                                   Whether to undo slice selection automatically after the operation is complete.
            iActiveDelay (int): Latency of read/write operations after chip selection, in microseconds.
                                The lower 16 bits are CS1 and the higher 16 bits are CS2.
            iDelayDeactive (int): Delay time for read and write operations after slice selection, in microseconds.
                                  The lower 16 bits are CS1 and the higher 16 bits are CS2.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347SPI_SetChipSelect(
            self._device_index,
            iEnableSelect,
            iChipSelect,
            iIsAutoDeativeCS,
            iActiveDelay,
            iDelayDeactive,
        )
        return result

    def spi_write(
        self, chip_select: int, write_data: bytes, write_step: int = 512
    ) -> bool:
        """
        SPI write data.

        Args:
            chip_select (int): Chip selection control. When bit 7 is 0, chip selection control is ignored.
                                When bit 7 is 1, chip selection operation is performed.
            write_data (bytes): Data to write.
            write_step (int, optional): The length of a single block to be read. Default is 512.

        Returns:
            bool: True if successful, False otherwise.
        """
        write_length = len(write_data)
        write_buffer = ctypes.create_string_buffer(write_data)
        result = self._dev.CH347SPI_Write(
            self._device_index, chip_select, write_length, write_step, write_buffer
        )
        return result

    def spi_read(
        self, chip_select: int, write_data: bytes, read_length: int
    ) -> Optional[bytes]:
        """
        SPI read data.

        Args:
            chip_select (int): Chip selection control. When bit 7 is 0, chip selection control is ignored.
                            When bit 7 is 1, chip selection operation is performed.
            write_data (bytes): Data to write.
            read_length (int): Number of bytes to read.

        Returns:
            bytes: Data read in from the SPI stream if successful, None otherwise.
        """
        write_length = len(write_data)

        # Create ctypes buffer for write data
        write_buffer = ctypes.create_string_buffer(write_data)

        # Create ctypes buffer for read data
        read_buffer = ctypes.create_string_buffer(read_length)

        # Create combined buffer for read and write data
        combined_buffer = ctypes.create_string_buffer(
            write_buffer.raw[:write_length] + read_buffer.raw
        )

        read_len = ctypes.c_ulong(read_length)
        result = self._dev.CH347SPI_Read(
            self._device_index,
            chip_select,
            write_length,
            ctypes.byref(read_len),
            combined_buffer,
        )

        if result:
            # Extract the read data from the combined buffer
            read_data = combined_buffer[: int(read_len.value)]
            return bytes(read_data)
        else:
            return None

    def spi_write_read(self, chip_select: int, write_data: bytes) -> Optional[bytes]:
        """
        Handle SPI data stream 4-wire interface, read after write is completed.

        Args:
            chip_select (int): Chip selection control. When bit 7 is 0, chip selection control is ignored.
                            When bit 7 is 1, chip selection operation is performed.
            write_data (bytes): Data to write.
        Returns:
            bytes: Data read in from the SPI stream if successful, None otherwise.
        """
        io_length = len(write_data)
        io_buffer = ctypes.create_string_buffer(write_data)
        result = self._dev.CH347SPI_WriteRead(
            self._device_index, chip_select, io_length, io_buffer
        )
        if result:
            return bytes(io_buffer)[:io_length]
        else:
            return None

    def spi_stream_write_read(
        self, chip_select: int, write_data: bytes
    ) -> Optional[bytes]:
        """
        Handle SPI data stream 4-wire interface, read and write data at the same time.

        Args:
            chip_select (int): Film selection control. If bit 7 is 0, slice selection control is ignored.
                               If bit 7 is 1, the parameter is valid: Bit 1 bit 0 is 00/01/10.
                               Select D0/D1/D2 pins as low-level active chip options, respectively.
            write_data (bytes): Data to write.

        Returns:
            bytes: Data read in from the SPI stream if successful, None otherwise.
        """
        io_length = len(write_data)
        io_buffer = ctypes.create_string_buffer(write_data)
        result = self._dev.CH347StreamSPI4(
            self._device_index, chip_select, io_length, io_buffer
        )
        if result:
            return bytes(io_buffer)[:io_length]
        else:
            return None

    def i2c_set(self, interface_speed: Literal[0, 1, 2, 3, 4, 5, 6] = 1) -> bool:
        """
        Set the serial port flow mode.

        Args:
            interface_speed (int): I2C interface speed / SCL frequency. Bit 1-bit 0:
                                0 = low speed / 20KHz
                                1 = standard / 100KHz (default)
                                2 = fast / 400KHz
                                3 = high speed / 750KHz
                                4 = 50KHz
                                5 = 200KHz
                                6 = 1MHz

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347I2C_Set(self._device_index, interface_speed)
        return result

    def i2c_set_delay_ms(self, delay_ms: int) -> bool:
        """
        Set the hardware asynchronous delay to a specified number of milliseconds before the next stream operation.

        Args:
            delay_ms (int): Delay duration in milliseconds (ms).

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347I2C_SetDelaymS(self._device_index, delay_ms)
        return result

    def i2c_set_stretch(self, enable: bool) -> bool:
        """
        Set the clock stretching function.

        Args:
            enable (bool)

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347I2C_SetStretch(self._device_index, enable)
        return result

    def i2c_stream(self, write_data: bytes, read_length: int) -> Optional[bytes]:
        """
        Process I2C data stream.

        Args:
            write_data (bytes): Data to write. The first byte is usually the I2C device address and read/write direction bit.
            read_length (int): Number of bytes of data to read.

        Returns:
            bytes: Data read from the I2C stream.
        """
        write_length = len(write_data)

        # Convert write_data to ctypes buffer
        write_buffer = ctypes.create_string_buffer(bytes(write_data))

        # Create ctypes buffer for read data
        read_buffer = ctypes.create_string_buffer(read_length)

        result = self._dev.CH347StreamI2C(
            self._device_index, write_length, write_buffer, read_length, read_buffer
        )

        if result:
            if read_length == 0:
                return bytes()
            return bytes(read_buffer)[:read_length]
        else:
            return None

    def i2c_stream_retACK(
        self, write_data: bytes, read_length: int
    ) -> Optional[Tuple[bytes, int]]:
        """
        Process I2C data stream, return ACK number during operation.

        Args:
            write_data (bytes): Data to write. The first byte is usually the I2C device address and read/write direction bit.
            read_length (int): Number of bytes of data to read.

        Returns:
            bytes: Data read from the I2C stream.
            int: ACK number.
        """
        write_length = len(write_data)

        # Convert write_data to ctypes buffer
        write_buffer = ctypes.create_string_buffer(bytes(write_data))

        # Create ctypes buffer for read data
        read_buffer = ctypes.create_string_buffer(write_length)
        ack_num = ctypes.c_ulong()

        result = self._dev.CH347StreamI2C_RetACK(
            self._device_index,
            write_length,
            write_buffer,
            read_length,
            read_buffer,
            ctypes.byref(ack_num),
        )
        if result:
            return bytes(read_buffer)[:read_length], ack_num.value
        else:
            return None

    def i2c_write_eeprom(
        self,
        eeprom_type: EEPROMType,
        address: int,
        write_data: bytes,
    ) -> bool:
        """
        Write data to i2c EEPROM.

        Args:
            eeprom_type (EEPROMType): EEPROM type.
            address (int): EEPROM address.
            write_data (bytes): Data to write.

        Returns:
            bool: True if successful, False otherwise.
        """
        length = len(write_data)
        write_buffer = ctypes.create_string_buffer(write_data)
        result = self._dev.CH347I2C_WriteEEPROM(
            self._device_index, eeprom_type, address, length, write_buffer
        )
        return result

    def i2c_read_eeprom(
        self,
        eeprom_type: EEPROMType,
        address: int,
        read_length: int,
    ) -> Optional[bytes]:
        """
        Read data from i2c EEPROM.

        Args:
            eeprom_type (EEPROMType): EEPROM type.
            address (int): EEPROM address.
            read_length (int): Number of bytes to read.

        Returns:
            bytes: Data read from the i2c EEPROM.
        """
        read_buffer = ctypes.create_string_buffer(read_length)
        result = self._dev.CH347I2C_ReadEEPROM(
            self._device_index, eeprom_type, address, read_length, read_buffer
        )
        if result:
            if read_length == 0:
                return bytes()
            return bytes(read_buffer)
        else:
            return None

    def open_uart(self, uart_index: int) -> Optional[int]:
        """
        Open UART device.

        Args:
            device_index (int, optional): Device index. Default is 0.

        Returns:
            int: Handle to the opened device if successful, None otherwise.
        """
        handle = self._dev.CH347Uart_Open(uart_index)
        if handle != INVALID_HANDLE_VALUE:
            return handle
        else:
            return None

    def close_uart(self, uart_index: int) -> bool:
        """
        Close UART device.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347Uart_Close(uart_index)
        return result

    def uart_set_device_notify(
        self, uart_index: int, iDeviceID: str, iNotifyRoutine: Optional[Callable] = None
    ) -> bool:
        """
        Configure device event notifier.

        Args:
            iDeviceID (str): Optional parameter specifying the ID of the monitored device.
            iNotifyRoutine (callable): Callback function to handle device events.

        Returns:
            bool: True if successful, False otherwise.
        """
        if iNotifyRoutine is None:
            callback = self._callback_funcs[0]
        else:
            callback = NOTIFY_ROUTINE(iNotifyRoutine)
            self._callback_funcs.append(callback)
        str_device_id = ctypes.c_char_p(iDeviceID.encode("utf-8"))
        result = self._dev.CH347Uart_SetDeviceNotify(
            uart_index, str_device_id, callback
        )
        return result

    def uart_init(
        self,
        uart_index: int,
        baudrate: int = 115200,
        bytesize: Literal[5, 6, 7, 8, 16] = 8,
        parity: Literal[0, 1, 2, 3, 4] = 0,
        stopbits: Literal[0, 1, 2] = 0,
        bytetimout: int = 0xFFFFFFFF,
    ) -> bool:
        """
        Initialize UART device.

        Args:
            baudrate (int, optional): Baud rate. Default is 115200.
            bytesize (int, optional): Number of data bits. Default is 8.
            parity (int, optional): Parity. 0 = None, 1 = Odd, 2 = Even, 3 = Mark, 4 = Space. Default is 0.
            stopbits (int, optional): Number of stop bits. 0 = 1, 1 = 1.5, 2 = 2. Default is 0.
            bytetimout (int, optional): Timeout for reading data blocks, in 100us. Use 0xFFFFFFFF to specify no timeout (default).

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347Uart_Init(
            uart_index, baudrate, bytesize, parity, stopbits, bytetimout
        )
        return result

    def uart_set_timeout(
        self, uart_index: int, write_timeout: int, read_timeout: int
    ) -> bool:
        """
        Set the timeout of UART data read and write.

        Args:
            write_timeout (int): Timeout for UART to write data blocks, in milliseconds. Use 0xFFFFFFFF to specify no timeout (default).
            read_timeout (int): Timeout for UART to read data blocks, in milliseconds. Use 0xFFFFFFFF to specify no timeout (default).

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347Uart_SetTimeout(uart_index, write_timeout, read_timeout)
        return result

    def uart_read(self, uart_index: int, read_length: int) -> Optional[bytes]:
        """
        Read UART data.

        Args:
            read_length (int): Number of bytes to read.

        Returns:
            bytes: Data read from the UART stream. None if the operation fails.
        """
        if read_length == 0:
            return bytes()
        # Create ctypes buffer for read data
        read_buffer = ctypes.create_string_buffer(read_length)

        read_len = ctypes.c_ulong(read_length)
        result = self._dev.CH347Uart_Read(
            uart_index,
            read_buffer,
            ctypes.byref(read_len),
        )

        if result:
            # Extract the read data from the combined buffer
            return bytes(read_buffer)[: int(read_len.value)]
        else:
            return None

    def uart_write(self, uart_index: int, write_data: bytes) -> bool:
        """
        Write UART data.

        Args:
            write_data (bytes): Data to write.

        Returns:
            bool: True if successful, False otherwise.
        """
        write_length = len(write_data)
        write_buffer = ctypes.create_string_buffer(write_data)
        result = self._dev.CH347Uart_Write(
            uart_index,
            write_buffer,
            ctypes.byref(ctypes.c_ulong(write_length)),
        )
        return result

    def uart_in_waiting(self, uart_index: int) -> Optional[int]:
        """
        Get the number of bytes in the UART input buffer.

        Returns:
            int: Number of bytes in the UART input buffer. None if the operation fails.
        """
        # BUG: CH347Uart_QueryBufUpload always returns 0
        num_bytes = ctypes.c_longlong()
        print(num_bytes)
        result = self._dev.CH347Uart_QueryBufUpload(uart_index, ctypes.byref(num_bytes))
        print(num_bytes)
        if result:
            return num_bytes.value
        else:
            return None

    def gpio_set(self, enable: int, dir_out: int, data_out: int) -> bool:
        """
        Set GPIO.

        Args:
            enable (int): each bit represents a GPIO pin, 0 = disable, 1 = enable.
            dir_out (int): each bit represents a GPIO pin, 0 = input, 1 = output.
            data_out (int): each bit represents a GPIO pin, 0 = low level, 1 = high level.

        Returns:
            bool: True if successful, False otherwise.
        """
        result = self._dev.CH347GPIO_Set(self._device_index, enable, dir_out, data_out)
        return result

    def gpio_get(self) -> Optional[Tuple[int, int]]:
        """
        Get GPIO.

        Returns:
            tuple: A tuple containing a boolean value indicating if the operation was successful
            and the GPIO status.

            The first element (int): GPIO direction. Each bit represents a GPIO pin, 0 = input, 1 = output.

            The second element (int): GPIO status. Each bit represents a GPIO pin, 0 = low level, 1 = high level.
        """
        dir_out = ctypes.c_ubyte()
        data_out = ctypes.c_ubyte()
        result = self._dev.CH347GPIO_Get(
            self._device_index, ctypes.byref(dir_out), ctypes.byref(data_out)
        )
        if result:
            return dir_out.value, data_out.value
        else:
            return None

    def gpio_set_interrupt(
        self,
        int0_pin: int,
        int0_edge: int,
        int1_pin: int,
        int1_edge: int,
        int_callback: Optional[Callable] = None,
    ) -> bool:
        """
        Set GPIO interrupt.

        Args:
            int0_pin (int): GPIO pin number for INT0, >7 to disable.
            int0_edge (int): GPIO edge for INT0. 0 = rising edge, 1 = falling edge, 2 = both edges.
            int1_pin (int): GPIO pin number for INT1, >7 to disable.
            int1_edge (int): GPIO edge for INT1. 0 = rising edge, 1 = falling edge, 2 = both edges.
            int_callback (callable): Callback function to handle GPIO interrupt, None to cancel all interrupt.

        Returns:
            bool: True if successful, False otherwise.
        """
        if int_callback is None:
            callback = GPIO_INT_ROUTINE()
        else:
            callback = GPIO_INT_ROUTINE(int_callback)
        self._callback_funcs.append(callback)
        result = self._dev.CH347SetIntRoutine(
            self._device_index, int0_pin, int0_edge, int1_pin, int1_edge, callback
        )
        return result

    def gpio_get_interrupt(self) -> Optional[int]:
        """
        Get GPIO interrupt.

        Returns:
            list: A 8 byte list containing the GPIO interrupt status.
            each byte represents a GPIO pin:
            bit 7: GPIO direction, 0 = input, 1 = output.
            bit 6: GPIO status, 0 = low level, 1 = high level.
            bit 5: GPIO interrupt, 0 = no interrupt, 1 = interrupt.
            bit 4-3: GPIO interrupt edge, 0 = rising edge, 1 = falling edge, 2 = both edges.
            other bits are reserved.
        """
        int_status = ctypes.c_ubyte()
        result = self._dev.CH347ReadInter(self._device_index, ctypes.byref(int_status))
        if result:
            return int_status.value
        else:
            return None


if __name__ == "__main__":
    ch347 = CH347()
    assert ch347.open_device()
    ch347.i2c_set(2)
    ch347.i2c_set_delay_ms(1)
    for addr in range(0x00, 0x80):
        data = ch347.i2c_stream(bytes([addr << 1, 0x00]), 1)
        if data is not None:
            print(f"0x{addr:02X}: {data.hex()}")
