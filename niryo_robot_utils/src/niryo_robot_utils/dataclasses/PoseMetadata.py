from typing import Dict, Any

from .ABCSerializable import ABCSerializable
from .enums import TcpVersion, LengthUnit


class PoseMetadata(ABCSerializable):
    __DEFAULT_TCP_VERSION = TcpVersion.DH_CONVENTION
    __DEFAULT_LENGTH_UNIT = LengthUnit.METERS
    __DEFAULT_FRAME = ''

    def __init__(self,
                 version: int,
                 tcp_version: TcpVersion,
                 frame: str = __DEFAULT_FRAME,
                 length_unit: LengthUnit = __DEFAULT_LENGTH_UNIT):
        self.version = version
        self.tcp_version = tcp_version
        self.frame = frame
        self.length_unit = length_unit

    def to_dict(self):
        return {
            'version': self.version,
            'tcp_version': self.tcp_version.name,
            'frame': self.frame,
            'length_unit': self.length_unit.name
        }

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'PoseMetadata':
        if d['version'] == 1:
            return cls.v1(d['frame'])
        elif d['version'] == 2:
            return cls.v2(TcpVersion[d['tcp_version']], d['frame'], LengthUnit[d['length_unit']])

    @classmethod
    def v1(cls, frame: str = __DEFAULT_FRAME) -> 'PoseMetadata':
        return cls(1, TcpVersion.LEGACY, frame=frame)

    @classmethod
    def v2(cls,
           tcp_version: TcpVersion = __DEFAULT_TCP_VERSION,
           frame: str = __DEFAULT_FRAME,
           length_unit: LengthUnit = __DEFAULT_LENGTH_UNIT) -> 'PoseMetadata':
        return cls(2, tcp_version=tcp_version, frame=frame, length_unit=length_unit)
