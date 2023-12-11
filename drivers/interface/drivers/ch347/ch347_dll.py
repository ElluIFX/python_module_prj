import ctypes
from ctypes import wintypes
from dataclasses import dataclass


@dataclass(frozen=True)
class DeviceConst:
    # CH347 驱动接口
    CH347_USB_CH341 = 0
    CH347_USB_HID = 2
    CH347_USB_VCP = 3

    # 芯片功能接口号
    CH347_FUNC_UART = 0
    CH347_FUNC_SPI_IIC = 1
    CH347_FUNC_JTAG_IIC = 2
    CH347_FUNC_JTAG_IIC_SPI = 3


@dataclass(frozen=True)
class EEPROMType:
    ID_24C01 = 0
    ID_24C02 = 1
    ID_24C04 = 2
    ID_24C08 = 3
    ID_24C16 = 4
    ID_24C32 = 5
    ID_24C64 = 6
    ID_24C128 = 7
    ID_24C256 = 8
    ID_24C512 = 9
    ID_24C102 = 10
    ID_24C204 = 11
    ID_24C409 = 12


class DeviceInfo(ctypes.Structure):
    MAX_PATH = 260
    _fields_ = [
        ("DeviceIndex", ctypes.c_ubyte),                # 当前打开序号
        ("DevicePath", ctypes.c_char * MAX_PATH),       # 设备路径
        ("UsbClass", ctypes.c_ubyte),                   # USB设备类别: 0=CH341 Vendor; 1=CH347 Vendor; 2=HID
        ("FuncType", ctypes.c_ubyte),                   # 设备功能类型: 0=UART1; 1=SPI+I2C; 2=JTAG+I2C
        ("DeviceID", ctypes.c_char * 64),               # USB设备ID: USB\VID_xxxx&PID_xxxx
        ("ChipMode", ctypes.c_ubyte),                   # 芯片模式: 0=Mode0(UART*2); 1=Mode1(Uart1+SPI+I2C); 2=Mode2(HID Uart1+SPI+I2C); 3=Mode3(Uart1+Jtag+I2C)
        ("DevHandle", ctypes.c_void_p),                 # 设备句柄
        ("BulkOutEndpMaxSize", ctypes.c_ushort),        # 上传端点大小
        ("BulkInEndpMaxSize", ctypes.c_ushort),         # 下传端点大小
        ("UsbSpeedType", ctypes.c_ubyte),               # USB速度类型: 0=FS; 1=HS; 2=SS
        ("CH347IfNum", ctypes.c_ubyte),                 # USB接口号
        ("DataUpEndp", ctypes.c_ubyte),                 # 端点地址
        ("DataDnEndp", ctypes.c_ubyte),                 # 端点地址
        ("ProductString", ctypes.c_char * 64),          # USB产品字符串
        ("ManufacturerString", ctypes.c_char * 64),     # USB厂商字符串
        ("WriteTimeout", wintypes.ULONG),               # USB写超时
        ("ReadTimeout", wintypes.ULONG),                # USB读超时
        ("FuncDescStr", ctypes.c_char * 64),            # 接口功能描述符
        ("FirmwareVer", ctypes.c_ubyte)                 # 固件版本
    ]  # fmt: skip


class SPIConfig(ctypes.Structure):
    _fields_ = [
        ("Mode", ctypes.c_ubyte),                       # 0-3: SPI Mode0/1/2/3
        ("Clock", ctypes.c_ubyte),                      # 0=60MHz, 1=30MHz, 2=15MHz, 3=7.5MHz, 4=3.75MHz, 5=1.875MHz, 6=937.5KHz, 7=468.75KHz
        ("ByteOrder", ctypes.c_ubyte),                  # 0=LSB first(LSB), 1=MSB first(MSB)
        ("SPIWriteReadInterval", ctypes.c_ushort),      # Regular interval for SPI read/write commands, in microseconds
        ("SPIOutDefaultData", ctypes.c_ubyte),          # Default output data when reading from SPI
        ("ChipSelect", wintypes.ULONG),                 # Chip select control. Bit 7 as 0 ignores chip select control,
                                                        # Bit 7 as 1 makes the parameters valid:
                                                        # Bit 1 and Bit 0 as 00/01 selects CS1/CS2 pin as the active low chip select.
        ("CS1Polarity", ctypes.c_ubyte),                # Bit 0: CS1 polarity control, 0: active low, 1: active high
        ("CS2Polarity", ctypes.c_ubyte),                # Bit 0: CS2 polarity control, 0: active low, 1: active high
        ("IsAutoDeativeCS", ctypes.c_ushort),           # Automatically de-assert chip select after the operation is completed
        ("ActiveDelay", ctypes.c_ushort),               # Delay time for executing read/write operations after chip select is set, in microseconds
        ("DelayDeactive", wintypes.ULONG)               # Delay time for executing read/write operations after chip select is de-asserted, in microseconds
    ]  # fmt: skip


def init_ch347_dll(dll_path: str) -> ctypes.WinDLL:
    ch347dll = ctypes.WinDLL(dll_path)

    ch347dll.CH347OpenDevice.argtypes = [wintypes.ULONG]
    ch347dll.CH347OpenDevice.restype = ctypes.c_void_p

    ch347dll.CH347CloseDevice.argtypes = [wintypes.ULONG]
    ch347dll.CH347CloseDevice.restype = ctypes.c_bool

    ch347dll.CH347GetDeviceInfor.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(DeviceInfo),
    ]
    ch347dll.CH347GetDeviceInfor.restype = ctypes.c_bool

    ch347dll.CH347GetVersion.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_ubyte),
    ]
    ch347dll.CH347GetVersion.restype = ctypes.c_bool

    ch347dll.CH347SetDeviceNotify.argtypes = [
        wintypes.ULONG,
        ctypes.c_char_p,
        ctypes.CFUNCTYPE(None, wintypes.ULONG),
    ]
    ch347dll.CH347SetDeviceNotify.restype = ctypes.c_bool

    ch347dll.CH347ReadData.argtypes = [
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347ReadData.restype = ctypes.c_bool

    ch347dll.CH347WriteData.argtypes = [
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347WriteData.restype = ctypes.c_bool

    ch347dll.CH347SetTimeout.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
    ]
    ch347dll.CH347SetTimeout.restype = ctypes.c_bool

    ch347dll.CH347SPI_Init.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(SPIConfig),
    ]
    ch347dll.CH347SPI_Init.restype = ctypes.c_bool

    ch347dll.CH347SPI_GetCfg.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(SPIConfig),
    ]
    ch347dll.CH347SPI_GetCfg.restype = ctypes.c_bool

    ch347dll.CH347SPI_SetDataBits.argtypes = [wintypes.ULONG, ctypes.c_ubyte]
    ch347dll.CH347SPI_SetDataBits.restype = ctypes.c_bool

    ch347dll.CH347SPI_ChangeCS.argtypes = [wintypes.ULONG, ctypes.c_ubyte]
    ch347dll.CH347SPI_ChangeCS.restype = ctypes.c_bool

    ch347dll.CH347SPI_SetChipSelect.argtypes = [
        wintypes.ULONG,
        ctypes.c_ushort,
        ctypes.c_ushort,
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
    ]
    ch347dll.CH347SPI_SetChipSelect.restype = ctypes.c_bool

    ch347dll.CH347SPI_Write.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
    ]
    ch347dll.CH347SPI_Write.restype = ctypes.c_bool

    ch347dll.CH347SPI_Read.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.POINTER(wintypes.ULONG),
        ctypes.c_void_p,
    ]
    ch347dll.CH347SPI_Read.restype = ctypes.c_bool

    ch347dll.CH347SPI_WriteRead.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
    ]
    ch347dll.CH347SPI_WriteRead.restype = ctypes.c_bool

    ch347dll.CH347StreamSPI4.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
    ]
    ch347dll.CH347StreamSPI4.restype = ctypes.c_bool

    ch347dll.CH347I2C_Set.argtypes = [wintypes.ULONG, wintypes.ULONG]
    ch347dll.CH347I2C_Set.restype = ctypes.c_bool

    ch347dll.CH347I2C_SetStretch.argtypes = [wintypes.ULONG, ctypes.c_bool]
    ch347dll.CH347I2C_SetStretch.restype = ctypes.c_bool

    ch347dll.CH347I2C_SetDelaymS.argtypes = [wintypes.ULONG, wintypes.ULONG]
    ch347dll.CH347I2C_SetDelaymS.restype = ctypes.c_bool

    ch347dll.CH347StreamI2C.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
        wintypes.ULONG,
        ctypes.c_void_p,
    ]
    ch347dll.CH347StreamI2C.restype = ctypes.c_bool

    ch347dll.CH347StreamI2C_RetACK.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347StreamI2C_RetACK.restype = ctypes.c_bool

    ch347dll.CH347ReadEEPROM.argtypes = [
        wintypes.ULONG,
        ctypes.c_int,
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347ReadEEPROM.restype = ctypes.c_bool

    ch347dll.CH347WriteEEPROM.argtypes = [
        wintypes.ULONG,
        ctypes.c_int,
        wintypes.ULONG,
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347WriteEEPROM.restype = ctypes.c_bool

    ch347dll.CH347Uart_Open.argtypes = [wintypes.ULONG]
    ch347dll.CH347Uart_Open.restype = ctypes.c_void_p

    ch347dll.CH347Uart_Close.argtypes = [wintypes.ULONG]
    ch347dll.CH347Uart_Close.restype = ctypes.c_bool

    ch347dll.CH347Uart_SetDeviceNotify.argtypes = [
        wintypes.ULONG,
        ctypes.c_char_p,
        ctypes.CFUNCTYPE(None, wintypes.ULONG),
    ]
    ch347dll.CH347Uart_SetDeviceNotify.restype = ctypes.c_bool

    ch347dll.CH347Uart_Init.argtypes = [
        wintypes.ULONG,
        wintypes.DWORD,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
    ]
    ch347dll.CH347Uart_Init.restype = ctypes.c_bool

    ch347dll.CH347Uart_SetTimeout.argtypes = [
        wintypes.ULONG,
        wintypes.ULONG,
        wintypes.ULONG,
    ]
    ch347dll.CH347Uart_SetTimeout.restype = ctypes.c_bool

    ch347dll.CH347Uart_Read.argtypes = [
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347Uart_Read.restype = ctypes.c_bool

    ch347dll.CH347Uart_Write.argtypes = [
        wintypes.ULONG,
        ctypes.c_void_p,
        ctypes.POINTER(wintypes.ULONG),
    ]
    ch347dll.CH347Uart_Write.restype = ctypes.c_bool

    ch347dll.CH347Uart_QueryBufUpload.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(ctypes.c_longlong),
    ]
    ch347dll.CH347Uart_QueryBufUpload.restype = ctypes.c_bool

    ch347dll.CH347GPIO_Get.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(ctypes.c_ubyte),
        ctypes.POINTER(ctypes.c_ubyte),
    ]
    ch347dll.CH347GPIO_Get.restype = ctypes.c_bool

    ch347dll.CH347GPIO_Set.argtypes = [
        wintypes.ULONG,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
    ]
    ch347dll.CH347GPIO_Set.restype = ctypes.c_bool

    ch347dll.CH347SetIntRoutine.argtypes = [
        wintypes.ULONG,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
        ctypes.c_ubyte,
        ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_ubyte)),
    ]
    ch347dll.CH347SetIntRoutine.restype = ctypes.c_bool

    ch347dll.CH347ReadInter.argtypes = [
        wintypes.ULONG,
        ctypes.POINTER(ctypes.c_ubyte),
    ]
    ch347dll.CH347ReadInter.restype = ctypes.c_bool

    ch347dll.CH347AbortInter.argtypes = [
        wintypes.ULONG,
    ]
    ch347dll.CH347AbortInter.restype = ctypes.c_bool

    return ch347dll
