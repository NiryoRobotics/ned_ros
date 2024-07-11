import re
from typing import Dict, Any

from .ABCSerializable import ABCSerializable
from .JointsPositionMetadata import JointsPositionMetadata


class JointsPosition(ABCSerializable):

    def __init__(self, *joints: float, metadata: JointsPositionMetadata = JointsPositionMetadata.v1()):
        self.__joints = joints
        self.metadata = metadata

    def __iter__(self):
        return iter(self.__joints)

    def __getitem__(self, item) -> float:
        return self.__joints[item]

    def __len__(self) -> int:
        return len(self.__joints)

    def __repr__(self):
        args = [str(joint) for joint in self.__joints]
        args += [f'{name}={repr(value)}' for name, value in self.__dict__.items() if value != self.__joints]
        repr_str = f'{self.__class__.__name__}({", ".join(args)})'
        return repr_str

    def to_dict(self) -> Dict[str, Any]:
        d = {f'joint_{n}': joint for n, joint in enumerate(self.__joints)}
        d['metadata'] = self.metadata.to_dict()
        return d

    @classmethod
    def from_dict(cls, d) -> 'JointsPosition':
        joints = []
        other_args = {}
        for name, value in d.items():
            if re.match(r'^joint_\d+$', name):
                joints.append(value)
            else:
                other_args[name] = value
        return cls(*joints, **other_args)
