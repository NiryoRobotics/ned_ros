import uuid


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
            raise UnknownVersionException(
                f'"{name}" is not a valid version name')

        return result['version']

    def set(self, name, version, version_name=''):

        params = {
            'version': version,
            'version_name': version_name,
            'name': name,
        }

        exists, id = self.exists(name)

        if exists:
            query = (
                'UPDATE version '
                'SET version=:version, version_name=:version_name, update_date=CURRENT_TIMESTAMP WHERE name=:name'
            )
        else:
            query = (
                'INSERT INTO version (id, name, version, version_name, update_date)'
                'VALUES (:id, :name, :version, :version_name, CURRENT_TIMESTAMP)'
            )
            params['id'] = str(uuid.uuid4())

        self.__dao.execute(query, params)
