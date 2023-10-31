from abc import ABC


class ABCTable(ABC):
    schema = ""

    def __init__(self, dao):
        self._dao = dao
        self._table_name = type(self).__name__.lower()
        query = 'SELECT name FROM sqlite_master WHERE type=="table" AND name==:name'
        result = self._dao.execute(query, {'name': self._table_name}).fetchone()
        if result is None:
            query = f'CREATE TABLE IF NOT EXISTS {self._table_name} ({self.schema})'
            self._dao.execute(query)
