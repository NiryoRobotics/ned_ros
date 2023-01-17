import uuid
from pydoc import locate


class UnknownSettingsException(Exception):
    pass


class Settings:
    def __init__(self, dao):
        self.__dao = dao

    def exists(self, name):
        query = 'SELECT id FROM settings WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            return False, None
        else:
            return True, result['id']

    def get(self, name):
        query = 'SELECT value, type FROM settings WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            raise UnknownSettingsException()

        if result['type'] is None or isinstance(result['type'], str):
            value_type = result['type']
        else:
            value_type = result['type'].decode()

        if result['value'] is None or isinstance(result['value'], str):
            value = result['value']
        else:
            value = result['value'].decode()

        return value, value_type

    def set(self, name, value, value_type):
        try:
            if value_type == 'bool' and value not in [
                    'True', 'true', 'False', 'false'
            ]:
                raise ValueError()
            else:
                locate(value_type)(value)
        except ValueError:
            raise TypeError('{} is not of type {}'.format(value, value_type))

        settings_exists, settings_id = self.exists(name)

        params = {
            'value': value,
            'type': value_type,
            'name': name,
        }

        if settings_exists:
            query = 'UPDATE settings SET value = :value, type = :type WHERE name = :name'
        else:
            query = 'INSERT INTO settings VALUES (:id, :name, :value, :type)'
            params['id'] = str(uuid.uuid4())

        self.__dao.execute(query, params)
