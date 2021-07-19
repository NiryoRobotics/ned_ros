import os

class ApiKey:
    def __init__(self, api_key_file_path):
        self.__file_path = api_key_file_path

    def __file_exists(self):
        return os.path.exists(self.__file_path)

    def read_key(self, attempt=3):
        if not self.__file_exists():
            return None

        if attempt <= 0:
            return None
        try:
            with open(self.__file_path, 'r') as f:
                return f.read().rstrip()
        except IOError:
            return self.read_key(attempt - 1)

    def write_key(self, key):
        try:
            with open(self.__file_path, 'w') as f:
                f.write(key)
                return True
        except IOError:
            return False
