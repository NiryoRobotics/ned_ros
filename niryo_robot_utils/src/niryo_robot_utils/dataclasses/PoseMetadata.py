from typing import Dict, Any

from .ABCSerializable import ABCSerializable
from .enums import LengthUnit


class PoseMetadata(ABCSerializable):
    __DEFAULT_LENGTH_UNIT = LengthUnit.METERS
    __DEFAULT_FRAME = ''

    def __init__(self, version: int, frame: str = __DEFAULT_FRAME, length_unit: LengthUnit = __DEFAULT_LENGTH_UNIT):
        self.version = version
        self.frame = frame
        self.length_unit = length_unit

    def to_dict(self):
        return {'version': self.version, 'frame': self.frame, 'length_unit': self.length_unit.name}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'PoseMetadata':
        if d['version'] == 1:
            return cls.v1(d['frame'])
        elif d['version'] == 2:
            return cls.v2(d['frame'], LengthUnit[d['length_unit']])

    @classmethod
    def v1(cls, frame: str = __DEFAULT_FRAME) -> 'PoseMetadata':
        return cls(1, frame=frame)

    @classmethod
    def v2(cls, frame: str = __DEFAULT_FRAME, length_unit: LengthUnit = __DEFAULT_LENGTH_UNIT) -> 'PoseMetadata':
        return cls(2, frame=frame, length_unit=length_unit)
