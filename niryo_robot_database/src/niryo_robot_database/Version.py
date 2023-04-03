class UnknownVersionException(Exception):
    pass


class Version:

    def __init__(self, dao):
        self.__dao = dao

    def exists(self, name):
        query = 'SELECT id FROM version WHERE name = :name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            return False, None
        else:
            return True, result['id']

    def get(self, name):
        query = 'SELECT id, name, version, version_name, update_date FROM version WHERE name=:name'
        result = self.__dao.execute(query, {'name': name}).fetchone()
        if result is None:
            raise UnknownVersionException(f'"{name}" is not a valid version name')

        return result['version']

    def set(self, name, version, version_name=''):
        query = ('UPDATE version '
                 'SET version=:version, version_name=:version_name, update_date=CURRENT_TIMESTAMP WHERE name=:name')
        self.__dao.execute(query, {'version': version, 'version_name': version_name, 'name': name})
