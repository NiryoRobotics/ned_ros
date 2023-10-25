import uuid
from datetime import date


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

    def get_all_by_type(self, file_type):
        query = 'SELECT id, type, name, date, path FROM file_path WHERE type = :type'
        result = self.__dao.execute(query, {'type': file_type}).fetchall()
        return [dict(r) for r in result]

    def add_file_path(self, file_type, name, path):
        file_path_exists, file_path_id = self.exists(name)

        params = {
            'type': file_type,
            'name': name,
            'date': str(date.today()),
            'path': path,
        }

        if file_path_exists:
            query = 'UPDATE file_path SET type = :type, name = :name, date = :date, path = :path WHERE name = :name'
        else:
            query = 'INSERT INTO file_path VALUES (:id, :type, :name, :date, :path)'
            params['id'] = str(uuid.uuid4())

        self.__dao.execute(query, params)
        return self.__dao.last_row_id

    def rm_file_path(self, file_id):
        query = 'DELETE FROM file_path WHERE id = :id'
        params = {'id': file_id}
        self.__dao.execute(query, params)
