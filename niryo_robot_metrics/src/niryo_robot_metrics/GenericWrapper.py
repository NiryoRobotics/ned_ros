from abc import ABCMeta, abstractmethod


class GenericWrapper:
    __metaclass__ = ABCMeta
    __available_metrics__ = []

    def __init__(self):
        self._data = {}

    @property
    def data(self):
        self._fetch_datas()
        return self._data

    @property
    def available_metrics(self):
        return self.__available_metrics__

    @abstractmethod
    def _fetch_datas(self):
        pass

    def __getitem__(self, item):
        self._fetch_datas()
        if item not in self.__available_metrics__:
            raise ValueError(f'{item} is not a valid metric name')
        return self._data[item]
