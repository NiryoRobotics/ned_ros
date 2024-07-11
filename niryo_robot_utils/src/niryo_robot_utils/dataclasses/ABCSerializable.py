from abc import abstractmethod, ABC
from typing import Dict, Any


class ABCSerializable(ABC):

    @abstractmethod
    def to_dict(self) -> Dict[str, Any]:
        raise NotImplementedError

    @classmethod
    @abstractmethod
    def from_dict(cls, d: Dict[str, Any]):
        raise NotImplementedError

    def copy(self) -> 'ABCSerializable':
        return self.from_dict(self.to_dict())

    def __repr__(self) -> str:
        args = [f'{name}={repr(value)}' for name, value in self.__dict__.items()]
        repr_str = f'{self.__class__.__name__}({", ".join(args)})'
        return repr_str
