from abc import ABCMeta, abstractmethod


class GenericWrapper:
    __metaclass__ = ABCMeta

    def __init__(self):
        self._data = {}

    def get_data(self):
        self._fetch_datas()
        return self._data

    @abstractmethod
    def _fetch_datas(self):
        pass
