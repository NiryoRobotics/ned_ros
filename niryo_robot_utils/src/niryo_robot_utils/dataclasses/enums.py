from enum import Enum, auto


class TcpVersion(Enum):
    LEGACY = auto()
    DH_CONVENTION = auto()


class LengthUnit(Enum):
    METERS = auto()
    MILLIMETERS = auto()
