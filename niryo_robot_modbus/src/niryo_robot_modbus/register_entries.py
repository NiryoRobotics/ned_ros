from __future__ import annotations
from abc import ABC, abstractmethod

from niryo_robot_python_ros_wrapper import NiryoRosWrapper


class ABCRegisterEntry(ABC):

    @staticmethod
    @abstractmethod
    def get_starting_address() -> int:
        raise NotImplementedError()

    @staticmethod
    @abstractmethod
    def get_data_type() -> type:
        pass

    @classmethod
    def is_read_only(cls: ABCRegisterEntry) -> bool:
        return getattr(cls.write, '__isabstractmethod__', False)

    def __init__(self, ros_wrapper: NiryoRosWrapper):
        self._ros_wrapper = ros_wrapper

    @property
    def address(self) -> int:
        return self.get_starting_address()

    @abstractmethod
    def read(self):
        pass

    @abstractmethod
    def write(self, value):
        pass


class ABCRegisterEntries(ABCRegisterEntry):

    @staticmethod
    @abstractmethod
    def get_address_count(ros_wrapper: NiryoRosWrapper) -> int:
        pass

    def __init__(self, ros_wrapper: NiryoRosWrapper, ix: int):
        super().__init__(ros_wrapper)
        self._index = ix

    @property
    def address(self) -> int:
        return self.get_starting_address() + self._index
