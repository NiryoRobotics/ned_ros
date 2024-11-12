from typing import Generator, Dict, List, Tuple
from niryo_robot_poses_handlers.transform_functions import convert_legacy_rpy_to_dh_convention, quaternion_from_euler

from .ABCSerializable import ABCSerializable
from .enums import TcpVersion, LengthUnit
from .PoseMetadata import PoseMetadata


class Pose(ABCSerializable):

    def __init__(self,
                 x: float,
                 y: float,
                 z: float,
                 roll: float,
                 pitch: float,
                 yaw: float,
                 metadata: PoseMetadata = None):
        if metadata is None:
            metadata = PoseMetadata.v2()
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.metadata = metadata

    def __iter__(self) -> Generator[float, None, None]:
        for attr in self.to_list():
            yield attr

    def __getitem__(self, value: int) -> float:
        return self.to_list()[value]

    def __len__(self) -> int:
        return 6

    def to_dict(self) -> Dict:
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'metadata': self.metadata.to_dict()
        }

    @classmethod
    def from_dict(cls, d: Dict) -> 'Pose':
        args = [d['x'], d['y'], d['z'], d['roll'], d['pitch'], d['yaw']]
        if 'metadata' in d:
            args.append(PoseMetadata.from_dict(d['metadata']))
        return cls(*args)

    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

    def quaternion(self) -> Tuple[float, float, float, float]:
        return quaternion_from_euler(self.roll, self.pitch, self.yaw)

    def convert_to_dh_convention(self) -> None:
        if self.metadata.tcp_version == TcpVersion.DH_CONVENTION:
            return
        roll, pitch, yaw = convert_legacy_rpy_to_dh_convention(self.roll, self.pitch, self.yaw)
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.metadata.tcp_version = TcpVersion.DH_CONVENTION

    def convert_to_meters(self) -> None:
        if self.metadata.length_unit == LengthUnit.METERS:
            return
        self.x /= 1000
        self.y /= 1000
        self.z /= 1000
        self.metadata.length_unit = LengthUnit.METERS

    def normalize(self) -> None:
        self.convert_to_meters()
        self.convert_to_dh_convention()
        self.metadata.version = 2
