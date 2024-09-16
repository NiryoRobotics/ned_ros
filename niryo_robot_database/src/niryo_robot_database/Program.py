from .ABCTable import ABCTable
from datetime import datetime


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
        ('Ned2_demo', 'Ned2_demo', 'A small demonstration program to show the Ned2 capabilities', False),
        ('Ned3Pro_demo', 'Ned3Pro_demo', 'A small demonstration program to show the Ned3Pro capabilities', False),
    ]

    def update(self, id_, values):
        values['saved_at'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return super().update(id_, values)

    def insert(self, id_, name, description, has_blockly):
        self.check_not_exists(id_)

        saved_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        program = {
            'id': id_, 'name': name, 'description': description, 'saved_at': saved_at, 'has_blockly': has_blockly
        }

        query = ('INSERT INTO program (id, name, description, saved_at, has_blockly) '
                 'VALUES (:id, :name, :description, :saved_at, :has_blockly)')
        self._dao.execute(query, program)
        return program
