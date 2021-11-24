import uuid
from datetime import datetime

class UnknownFilePathException(Exception):
    pass


class FilePath:
    def __init__(self, dao):
        self.__dao = dao

    def exists(self, name):
        query = 'SELECT id FROM file_path WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            return False, None
        else:
            return True, result['id']

    def get_path_from_name(self, name):
        query = 'SELECT path FROM file_path WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            raise UnknownFilePathException()
        return result['path']

    def add_file_path(self, file_type, name, path):
        file_path_exists, file_path_id = self.exists(name)

        params = {
            'type': file_type,
            'name': name,
            'date': datetime.now().isoformat(),
            'path': path,
        }

        if file_path_exists:
            query = 'UPDATE file_path SET type = :type, name = :name, date = :date, path = :path WHERE name = :name'
        else:
            query = 'INSERT INTO file_path VALUES (:id, :type, :name, :date, :path)'
            params['id'] = str(uuid.uuid4())

        self.__dao.execute(query, params)
