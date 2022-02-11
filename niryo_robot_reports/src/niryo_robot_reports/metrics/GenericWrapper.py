from abc import ABCMeta, abstractmethod


class GenericWrapper:
    __metaclass__ = ABCMeta

    def __init__(self):
        self._data = []

    @classmethod
    def get_data(cls):
        inst = cls()
        inst._fetch_datas()
        return inst._data

    @abstractmethod
    def _fetch_datas(self):
        pass
