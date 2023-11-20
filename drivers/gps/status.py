import datetime

from pynmeagps import NMEAMessage


class RMC_Status(object):
    """
    RMC: Recommended Minimum Specific GNSS Data
    """

    vaild: bool = False
    lat: float = 0
    NS: str = ""
    lon: float = 0
    EW: str = ""
    speed: float = 0
    heading: float = 0
    UTCtime: datetime.time = None
    UTCdate: datetime.date = None

    def __str__(self) -> str:
        return f"RMC_Status(vaild={self.vaild}, lat={self.lat}, NS={self.NS}, lon={self.lon}, EW={self.EW}, speed={self.speed}, heading={self.heading}, UTCtime={self.UTCtime}, UTCdate={self.UTCdate})"

    def __repr__(self) -> str:
        return self.__str__()

    def update(self, pack: NMEAMessage):
        if not pack.status == "A":
            self.vaild = False
        else:
            self.vaild = True
            self.lat = pack.lat
            self.NS = pack.NS
            self.lon = pack.lon
            self.EW = pack.EW
            self.speed = pack.spd
            self.heading = pack.cog
            self.UTCtime = pack.time
            self.UTCdate = pack.date


class VTG_Status(object):
    """
    VTG: Ground Speed and Heading information
    """

    vaild: bool = False
    cogt: float = 0  # ground heading, true north, unit: degrees
    cogm: float = 0  # ground heading, magnetic north, unit: degrees
    sogn: float = 0  # speed over ground, unit: knots
    sogk: float = 0  # speed over ground, unit: km/h

    def __str__(self) -> str:
        return f"VTG_Status(vaild={self.vaild}, cogt={self.cogt}, cogm={self.cogm}, sogn={self.sogn}, sogk={self.sogk})"

    def __repr__(self) -> str:
        return self.__str__()

    def update(self, pack: NMEAMessage):
        if not pack.posMode == "A":
            self.vaild = False
        else:
            self.vaild = True
            self.cogt = pack.cogt
            self.cogm = pack.cogm
            self.sogn = pack.sogn
            self.sogk = pack.sogk


class GGA_Status(object):
    """
    GGA: Global Positioning System Fix Data
    """

    vaild: bool = False
    UTCtime: datetime.time = None
    lat: float = 0
    NS: str = ""
    lon: float = 0
    EW: str = ""
    fix: int = 0  # 0: no fix, 1: GPS fix, 6: DGPS fix
    num_sat: int = 0  # number of satellites in use
    hdop: float = 0  # horizontal dilution of precision
    alt: float = 0  # altitude above mean sea level, unit: m
    geoid_sep: float = 0  # geoid separation, unit: m
    dgps_age: float = 0
    dgps_sid: int = 0

    def __str__(self) -> str:
        return f"GGA_Status(vaild={self.vaild}, lat={self.lat}, NS={self.NS}, lon={self.lon}, EW={self.EW}, fix={self.fix}, num_sat={self.num_sat}, hdop={self.hdop}, alt={self.alt}, geoid_sep={self.geoid_sep}, dgps_age={self.dgps_age}, dgps_sid={self.dgps_sid}, UTCtime={self.UTCtime})"

    def __repr__(self) -> str:
        return self.__str__()

    def update(self, pack: NMEAMessage):
        if pack.quality == 0:
            self.vaild = False
        else:
            self.vaild = True
            self.lat = pack.lat
            self.NS = pack.NS
            self.lon = pack.lon
            self.EW = pack.EW
            self.UTCtime = pack.time
            self.alt = pack.alt
            self.geoid_sep = pack.sep
            self.dgps_age = pack.diffAge
            self.dgps_sid = pack.diffStation
        self.fix = pack.quality
        self.num_sat = pack.numSV
        self.hdop = pack.HDOP
