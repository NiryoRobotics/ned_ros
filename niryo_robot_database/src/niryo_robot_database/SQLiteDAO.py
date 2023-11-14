import sqlite3
from threading import Lock


class SQLiteDAO:

    def __init__(self, db_path):
        self.__lock = Lock()
        self.__con = sqlite3.connect(db_path, check_same_thread=False)
        self.__con.row_factory = sqlite3.Row
        self.__cursor = self.__con.cursor()

    def __del__(self):
        try:
            self.__con.close()
        except AttributeError:
            pass

    @property
    def last_row_id(self):
        return self.__cursor.lastrowid

    def execute(self, *args, **kwargs):
        with self.__lock:
            exec_result = self.__cursor.execute(*args, **kwargs)
            self.__con.commit()
        return exec_result
