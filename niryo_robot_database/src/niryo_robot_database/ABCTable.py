from abc import ABC, abstractmethod


class RowNotFoundError(Exception):
    pass


class ABCTable(ABC):
    _schema = ""
    _default = []

    def __init__(self, dao):
        self._dao = dao
        self._table_name = type(self).__name__.lower()
        query = 'SELECT name FROM sqlite_master WHERE type=="table" AND name==:name'
        result = self._dao.execute(query, {'name': self._table_name}).fetchone()
        if result is None:
            self.__create_table()
            self.__populate_table()

    def __create_table(self):
        query = f'CREATE TABLE IF NOT EXISTS {self._table_name} ({self.schema})'
        self._dao.execute(query)

    def __populate_table(self):
        for default_args in self._default:
            self.insert(*default_args)

    def get_all(self):
        query = f'SELECT * FROM {self._table_name}'
        result = self._dao.execute(query).fetchall()
        return result

    def get_by_id(self, id_):
        query = f'SELECT * FROM {self._table_name} WHERE id = :id'
        result = self._dao.execute(query, {'id': id_}).fetchone()
        return result

    def exists(self, id_):
        return self.get_by_id(id_) is not None

    def delete(self, id_):
        if not self.exists(id_):
            raise RowNotFoundError(f'id "{id_}" does not exist in {self._table_name}.')

        query = 'DELETE FROM {self._table_name} WHERE id = :id'
        self._dao.execute(query, {'id': id_})

    @abstractmethod
    def insert(self, *args, **kwargs):
        raise NotImplementedError()

    @abstractmethod
    def update(self, *args, **kwargs):
        raise NotImplementedError()
