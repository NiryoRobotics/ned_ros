from .ABCTable import ABCTable
from datetime import datetime
from uuid import uuid4


class UnknownProgramException(Exception):
    pass


class Program(ABCTable):
    _schema = """
id TEXT PRIMARY KEY,
name TEXT,
description TEXT,
saved_at DATE,
has_blockly BOOLEAN
"""
    _default = [
        (str(uuid4()), 'Ned2_demo', 'A small demonstration program to show the Ned2 capabilities', False),
    ]

    def update(self, id_, name=None, description=None, has_blockly=None):
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

    def insert(self, id_, name, description, has_blockly):
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
