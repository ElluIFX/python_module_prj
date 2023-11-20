import json
import math
from io import BytesIO
from typing import Optional

import requests
from PIL import Image


class ZKW_Commands(object):
    SAVE_PARMS = "$PCAS00"
    BAUDRATE_4800 = "$PCAS01,0"
    BAUDRATE_9600 = "$PCAS01,1"
    BAUDRATE_19200 = "$PCAS01,2"
    BAUDRATE_38400 = "$PCAS01,3"
    BAUDRATE_57600 = "$PCAS01,4"
    BAUDRATE_115200 = "$PCAS01,5"
    AVAILABLE_BAUDRATES = (4800, 9600, 19200, 38400, 57600, 115200)
    OUTPUT_1HZ = "$PCAS02,1000"
    OUTPUT_2HZ = "$PCAS02,500"
    OUTPUT_4HZ = "$PCAS02,250"
    OUTPUT_5HZ = "$PCAS02,200"
    OUTPUT_10HZ = "$PCAS02,100"
    AVAILABLE_OUTPUT_RATES = (1, 2, 4, 5, 10)

    @staticmethod
    def SET_NEMA_ITEMS(
        GGA: bool = False,
        GLL: bool = False,
        GSA: bool = False,
        GSV: bool = False,
        RMC: bool = False,
        VTG: bool = False,
        ZDA: bool = False,
        TXT: bool = False,
    ) -> str:
        """
        Parameters meanings:
        GGA: Global Positioning System Fix Data
        GLL: Geographic Position - Latitude/Longitude
        GSA: GPS DOP and active satellites
        GSV: GPS Satellites in view
        RMC: Recommended Minimum Specific GNSS Data
        VTG: Course Over Ground and Ground Speed
        ZDA: Date & Time
        TXT: Text Transmission
        """
        return f"$PCAS03,{int(GGA)},{int(GLL)},{int(GSA)},{int(GSV)},{int(RMC)},{int(VTG)},{int(ZDA)},{int(TXT)},0,0"

    @staticmethod
    def SET_NEMA_TALKERS(GPS: bool, BEIDOU: bool, GLONASS: bool) -> str:
        return f"$PCAS04,{int(GPS) + 2*int(BEIDOU) + 4*int(GLONASS)}"

    NEMA4_0_PROTOCOL = "$PCAS05,5"
    NEMA4_1_PROTOCOL = "$PCAS05,2"
    NEMA4_2_PROTOCOL = "$PCAS05,9"

    BOOT_HOT = "$PCAS10,0"
    BOOT_WARM = "$PCAS10,1"
    BOOT_COLD = "$PCAS10,2"
    BOOT_FACTORY = "$PCAS10,3"

    MODE_PORTABLE = "$PCAS11,0"
    MODE_STATIC = "$PCAS11,1"
    MODE_WALKING = "$PCAS11,2"
    MODE_CAR = "$PCAS11,3"
    MODE_SEA = "$PCAS11,4"
    MODE_SEA_1G = "$PCAS11,5"
    MODE_SEA_2G = "$PCAS11,6"
    MODE_SEA_4G = "$PCAS11,7"


def add_nema_checksum(command):
    """
    Add checksum to a command
    """
    if command[0] == "$":
        command = command[1:]
    checksum = 0
    for i, char in enumerate(command):
        if char == "*":
            command = command[:i]
            break
        checksum ^= ord(char)
    return f"${command}*{checksum:02X}\r\n"


x_pi = 3.14159265358979324 * 3000.0 / 180.0
pi = 3.1415926535897932384626  # π
a = 6378245.0  # 长半轴
ee = 0.00669342162296594323  # 偏心率平方
API_KEY = (
    "ptxBqLe12hDKrURvGUnDVeMrBLubppI8"
)  # 百度地图API密钥 #https://lbsyun.baidu.com/apiconsole/key#/home


class Coord_Trans(object):
    @staticmethod
    def bd09_to_gcj02(bd_lon, bd_lat):
        """
        百度坐标系(BD-09)转火星坐标系(GCJ-02)
        """
        x = bd_lon - 0.0065
        y = bd_lat - 0.006
        z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * x_pi)
        theta = math.atan2(y, x) - 0.000003 * math.cos(x * x_pi)
        gg_lon = z * math.cos(theta)
        gg_lat = z * math.sin(theta)
        return [gg_lon, gg_lat]

    @staticmethod
    def gcj02_to_wgs84(lon, lat):
        """
        GCJ02转WGS84
        """
        dlat = Coord_Trans._transformlat(lon - 105.0, lat - 35.0)
        dlon = Coord_Trans._transformlon(lon - 105.0, lat - 35.0)
        radlat = lat / 180.0 * pi
        magic = math.sin(radlat)
        magic = 1 - ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
        dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
        mglat = lat + dlat
        mglon = lon + dlon
        return [lon * 2 - mglon, lat * 2 - mglat]

    @staticmethod
    def gcj02_to_bd09(lon, lat):
        """
        火星坐标系(GCJ-02)转百度坐标系(BD-09)
        """
        z = math.sqrt(lon * lon + lat * lat) + 0.00002 * math.sin(lat * x_pi)
        theta = math.atan2(lat, lon) + 0.000003 * math.cos(lon * x_pi)
        bd_lon = z * math.cos(theta) + 0.0065
        bd_lat = z * math.sin(theta) + 0.006
        return [bd_lon, bd_lat]

    @staticmethod
    def wgs84_to_gcj02(lon, lat):
        """
        WGS84转GCJ02(火星坐标系)
        """
        dlat = Coord_Trans._transformlat(lon - 105.0, lat - 35.0)
        dlon = Coord_Trans._transformlon(lon - 105.0, lat - 35.0)
        radlat = lat / 180.0 * pi
        magic = math.sin(radlat)
        magic = 1 - ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
        dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
        mglat = lat + dlat
        mglon = lon + dlon
        return [mglon, mglat]

    @staticmethod
    def wgs84_to_bd09(lon, lat):
        """
        WGS84转百度坐标系(BD-09)
        """
        lon, lat = Coord_Trans.wgs84_to_gcj02(lon, lat)
        return Coord_Trans.gcj02_to_bd09(lon, lat)

    @staticmethod
    def bd09_to_wgs84(bd_lon, bd_lat):
        """
        百度坐标系(BD-09)转WGS84
        """
        lon, lat = Coord_Trans.bd09_to_gcj02(bd_lon, bd_lat)
        return Coord_Trans.gcj02_to_wgs84(lon, lat)

    @staticmethod
    def out_of_china(lon, lat):
        """
        判断是否在国内
        """
        return not (lon > 73.66 and lon < 135.05 and lat > 3.86 and lat < 53.55)

    @staticmethod
    def _transformlon(lon, lat):
        ret = (
            300.0
            + lon
            + 2.0 * lat
            + 0.1 * lon * lon
            + 0.1 * lon * lat
            + 0.1 * math.sqrt(math.fabs(lon))
        )
        ret += (
            (20.0 * math.sin(6.0 * lon * pi) + 20.0 * math.sin(2.0 * lon * pi))
            * 2.0
            / 3.0
        )
        ret += (20.0 * math.sin(lon * pi) + 40.0 * math.sin(lon / 3.0 * pi)) * 2.0 / 3.0
        ret += (
            (150.0 * math.sin(lon / 12.0 * pi) + 300.0 * math.sin(lon / 30.0 * pi))
            * 2.0
            / 3.0
        )
        return ret

    @staticmethod
    def _transformlat(lon, lat):
        ret = (
            -100.0
            + 2.0 * lon
            + 3.0 * lat
            + 0.2 * lat * lat
            + 0.1 * lon * lat
            + 0.2 * math.sqrt(math.fabs(lon))
        )
        ret += (
            (20.0 * math.sin(6.0 * lon * pi) + 20.0 * math.sin(2.0 * lon * pi))
            * 2.0
            / 3.0
        )
        ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
        ret += (
            (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0))
            * 2.0
            / 3.0
        )
        return ret

    @staticmethod
    def api_trans(lon, lat, type=5):
        """
        使用百度api进行坐标转换(从GPS)
        type: 3:GaoDe/TX, 5:Baidu
        """
        API_ADDR = (
            "https://api.map.baidu.com/geoconv/v1/?coords={},{}&from=1&to={}&ak={}"
        )
        url = API_ADDR.format(lon, lat, type, API_KEY)
        response = requests.get(url)
        if response.status_code == 200:
            data = json.loads(response.text)
            if data["status"] == 0:
                return data["result"][0]["x"], data["result"][0]["y"]
        return Coord_Trans.wgs84_to_bd09(lon, lat)


def get_bd_map_image(
    lon, lat, zoom=18, width=640, height=640, highdpi=False
) -> Optional[Image.Image]:
    """
    获取百度地图图片
    """
    API_ADDR = (
        "http://api.map.baidu.com/staticimage/v2?ak={}&mcode={}"
        "&center={},{}&width={}&height={}&zoom={}&copyright=1"
    )
    if highdpi:
        API_ADDR += "&dpiType=ph&scale=2"
        width /= 2
        height /= 2
        zoom -= 1
    zoom = max(3, zoom)
    zoom = min(18 if highdpi else 19, zoom)
    url = API_ADDR.format(
        API_KEY,
        666666,
        lon,
        lat,
        int(width),
        int(height),
        int(zoom),
    )
    response = requests.get(url)
    if response.status_code == 200:
        try:
            return Image.open(BytesIO(response.content))
        except Exception:
            print(f"地图API错误: {response}")
            return None
    return None
