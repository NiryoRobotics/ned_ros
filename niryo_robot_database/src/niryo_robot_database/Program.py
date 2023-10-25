from .ABCTable import ABCTable
from datetime import datetime


class UnknownProgramException(Exception):
    pass


class Program(ABCTable):
    schema = """
id TEXT PRIMARY KEY,
name TEXT,
description TEXT,
saved_at DATE,
has_blockly BOOLEAN
"""

    def exists(self, id_):
        query = 'SELECT id FROM program WHERE id = :id'
        result = self._dao.execute(query, {'id': id_}).fetchone()
        return result is not None

    def get_all(self):
        query = 'SELECT * FROM program'
        result = self._dao.execute(query).fetchall()
        return result

    def get_by_id(self, id_):
        query = 'SELECT * FROM program WHERE id=:id'
        result = self._dao.execute(query, {'id': id_}).fetchone()
        return result

    def update_program(self, id_, name=None, description=None, has_blockly=None):
        if not self.exists(id_):
            raise UnknownProgramException(f"Program with ID {id_} does not exist.")

        update_data = {}

        if name is not None:
            update_data['name'] = name
        if description is not None:
            update_data['description'] = description
        if has_blockly is not None:
            update_data['has_blockly'] = has_blockly
        update_data['saved_at'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        query_args = ', '.join([f'{key}:={key}' for key in update_data])

        update_data['id'] = id_
        query = f'UPDATE program SET {query_args} WHERE id=:id'
        self._dao.execute(query, update_data)

        return self.get_by_id(id_)

    def insert_program(self, id_, name, description, has_blockly):
        if self.exists(id_):
            raise ValueError(f"Program with ID {id_} already exists.")

        saved_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        program = {
            'id': id_, 'name': name, 'description': description, 'saved_at': saved_at, 'has_blockly': has_blockly
        }

        query = ('INSERT INTO program (id, name, description, saved_at, has_blockly) '
                 'VALUES (:id, :name, :description, :saved_at, :has_blockly)')
        self._dao.execute(query, program)
        return program

    def delete_program(self, id_):
        if not self.exists(id_):
            raise UnknownProgramException(f"Program with ID {id_} does not exist.")

        query = 'DELETE FROM program WHERE id = :id'
        self._dao.execute(query, {'id': id_})
