from typing import Generator, Dict, List, Tuple, Union
from niryo_robot_poses_handlers.transform_functions import quaternion_from_euler

from .ABCSerializable import ABCSerializable
from .enums import LengthUnit
from .PoseMetadata import PoseMetadata


class Pose(ABCSerializable):
    __ATTRIBUTES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')

    def __init__(self,
                 x: float,
                 y: float,
                 z: float,
                 roll: float,
                 pitch: float,
                 yaw: float,
                 metadata: PoseMetadata = None):
        if metadata is None:
            metadata = PoseMetadata()
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.metadata = metadata

    def __iter__(self) -> Generator[float, None, None]:
        for attr in self.__ATTRIBUTES:
            yield getattr(self, attr)

    def __get_attr_name(self, key: Union[int, str]):
        if key in self.__ATTRIBUTES:
            return key
        elif isinstance(key, int) and 0 <= key < len(self.__ATTRIBUTES):
            return self.__ATTRIBUTES[key]
        elif isinstance(key, int):
            raise IndexError(f"Index out of range: {key}. Expected between 0 and {len(self.__ATTRIBUTES) - 1}.")
        else:
            raise KeyError(f"Invalid key: {key}. Expected int or str.")

    def __getitem__(self, value: Union[int, str]) -> float:
        return getattr(self, self.__get_attr_name(value))

    def __setitem__(self, key: Union[int, str], value: float) -> None:
        setattr(self, self.__get_attr_name(key), value)

    def __len__(self) -> int:
        return len(self.__ATTRIBUTES)

    def to_dict(self) -> Dict:
        return {**{attr: getattr(self, attr) for attr in self.__ATTRIBUTES}, 'metadata': self.metadata.to_dict()}

    @classmethod
    def from_dict(cls, d: Dict) -> 'Pose':
        args = [d[arg] for arg in cls.__ATTRIBUTES]
        if 'metadata' in d:
            args.append(PoseMetadata.from_dict(d['metadata']))
        return cls(*args)

    def to_list(self) -> List[float]:
        return [getattr(self, attr) for attr in self.__ATTRIBUTES]

    def quaternion(self) -> Tuple[float, float, float, float]:
        return quaternion_from_euler(self.roll, self.pitch, self.yaw)

    def convert_to_meters(self) -> None:
        if self.metadata.length_unit == LengthUnit.METERS:
            return
        self.x /= 1000
        self.y /= 1000
        self.z /= 1000
        self.metadata.length_unit = LengthUnit.METERS

    def normalize(self) -> None:
        self.convert_to_meters()
        self.metadata.version = 2
