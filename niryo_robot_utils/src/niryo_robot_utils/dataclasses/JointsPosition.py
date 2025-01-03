import re
from typing import Dict, Any, List

from .ABCSerializable import ABCSerializable
from .JointsPositionMetadata import JointsPositionMetadata


class JointsPosition(ABCSerializable):

    def __init__(self, *joints: float, metadata: JointsPositionMetadata = None):
        self.__joints = list(joints)
        if metadata is None:
            metadata = JointsPositionMetadata.v1()
        self.metadata = metadata

    def __iter__(self):
        return iter(self.__joints)

    def __getitem__(self, item) -> float:
        return self.__joints[item]

    def __setitem__(self, key: int, value: float):
        self.__joints[key] = value

    def __delitem__(self, index: int):
        del self.__joints[index]

    def __len__(self) -> int:
        return len(self.__joints)

    def insert(self, index: int, value: float):
        self.__joints.insert(index, value)

    def __repr__(self) -> str:
        args = [str(joint) for joint in self.__joints]
        args += [f'{name}={repr(value)}' for name, value in self.__dict__.items() if value != self.__joints]
        repr_str = f'{self.__class__.__name__}({", ".join(args)})'
        return repr_str

    def __eq__(self, other) -> bool:
        return self.__joints == other.__joints

    def to_list(self) -> List[int]:
        return list(self.__joints)

    def to_dict(self) -> Dict[str, Any]:
        d = {f'joint_{n}': joint for n, joint in enumerate(self.__joints)}
        d['metadata'] = self.metadata.to_dict()
        return d

    @classmethod
    def from_dict(cls, d) -> 'JointsPosition':
        """
        Creates a new JointsPosition object from a dictionary representing the object.

        :param d: A dictionary representing the object.
        :type d: dict
        """
        joints = []
        other_args = {}
        for name, value in d.items():
            if re.match(r'^joint_\d+$', name):
                joints.append(value)
        if 'metadata' in d:
            other_args['metadata'] = JointsPositionMetadata.from_dict(d['metadata'])
        return cls(*joints, **other_args)
