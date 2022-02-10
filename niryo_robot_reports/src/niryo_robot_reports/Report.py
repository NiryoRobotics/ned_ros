import os
import json


class Report:
    DEFAULT_VALUE = {}

    def __init__(self, path):
        self.__path = ''
        self.__deleted = False
        self.set_path(path)

    def delete(self):
        os.remove(self.__path)
        self.__deleted = True

    def __del__(self):
        if not self.__deleted:
            self._write()

    def set_path(self, path):
        self.__path = path
        if not os.path.isfile(self.__path):
            open(self.__path, 'w+').close()
        self._read()

    def _read(self):
        with open(self.__path, 'r') as f:
            try:
                self.content = json.load(f)
            except ValueError:
                self.content = self.DEFAULT_VALUE

    def _write(self):
        with open(self.__path, 'w') as f:
            json.dump(self.content, f)
