from typing import Dict, Any

from .ABCSerializable import ABCSerializable


class JointsPositionMetadata(ABCSerializable):

    def __init__(self, version: int):
        self.version = version

    def to_dict(self) -> Dict[str, Any]:
        return {'version': self.version}

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> 'JointsPositionMetadata':
        return cls(d['version'])

    @classmethod
    def v1(cls) -> 'JointsPositionMetadata':
        return cls(version=1)
