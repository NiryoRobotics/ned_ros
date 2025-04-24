from typing import Dict, Any

from .ABCSerializable import ABCSerializable
from .enums import LengthUnit


class PoseMetadata(ABCSerializable):
    __DEFAULT_LENGTH_UNIT = LengthUnit.METERS
    __DEFAULT_FRAME = ''

    def __init__(self,
                 version: int = 2,
                 *,
                 frame: str = __DEFAULT_FRAME,
                 length_unit: LengthUnit = __DEFAULT_LENGTH_UNIT):
        """
        :param version: Not used anymore, but kept for backward compatibility
        :param frame: The frame of the pose. It can be empty, in which case the default frame is used.
        :param length_unit: The unit of length used in the pose. If empty, the default unit is used.
        """
        self.version = version
        self.frame = frame
        self.length_unit = length_unit

    def to_dict(self):
        return {'version': self.version, 'frame': self.frame, 'length_unit': self.length_unit.name, 'tcp_version': 1}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'PoseMetadata':
        if d['version'] == 1:
            return cls.v1(frame=d['frame'])
        elif d['version'] == 2:
            return cls.v2(frame=d['frame'], length_unit=LengthUnit[d['length_unit']])

    @classmethod
    def v1(cls, *, frame: str = __DEFAULT_FRAME) -> 'PoseMetadata':
        """
        This method is kept for backward compatibility. It creates a PoseMetadata object with version 1.
        :param frame: The frame of the pose. It can be empty, in which case the default frame is used.
        :return: A PoseMetadata object with version 1.
        """
        return cls(1, frame=frame)

    @classmethod
    def v2(cls, *, frame: str = __DEFAULT_FRAME, length_unit: LengthUnit = __DEFAULT_LENGTH_UNIT) -> 'PoseMetadata':
        """
        This method is kept for backward compatibility. It creates a PoseMetadata object with version 2.
        :param frame: The frame of the pose. It can be empty, in which case the default frame is used.
        :param length_unit: The unit of length used in the pose. If empty, the default unit is used.
        :return: A PoseMetadata object with version 2.
        """
        return cls(2, frame=frame, length_unit=length_unit)
