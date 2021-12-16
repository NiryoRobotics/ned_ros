import uuid
from datetime import datetime


class UnknownMetricException(Exception):
    pass


class Metrics:
    def __init__(self, dao):
        self.__dao = dao

    def exists(self, name):
        query = 'SELECT id FROM metric WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            return False, None
        else:
            return True, result['id']

    def get(self, name):
        query = 'SELECT id, name, value, update_date FROM metric WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            raise UnknownMetricException()
        return result

    def get_all(self):
        query = 'SELECT id, name, value, update_date FROM metric'
        result = self.__dao.execute(query).fetchall()
        return result

    def set(self, name, value):
        settings_exists, settings_id = self.exists(name)
        update_date = datetime.now().isoformat()

        params = {
            'value': value,
            'update_date': update_date,
            'name': name,
        }
        if settings_exists:
            query = 'UPDATE metric SET value = :value, update_date = :update_date WHERE name = :name'
        else:
            query = 'INSERT INTO metric VALUES (:id, :name, :value, :update_date)'
            params['id'] = str(uuid.uuid4())

        self.__dao.execute(query, params)
