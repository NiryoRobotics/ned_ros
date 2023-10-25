import os
from uuid import uuid4
from tempfile import NamedTemporaryFile

import rospy
from niryo_robot_database.Program import Program
from niryo_robot_database.SQLiteDAO import SQLiteDAO

from .ProgramsFileManager import ProgramsFileManager


class ProgramsManager:

    def __init__(self, db_path, programs_path):
        self.__programs_path = programs_path
        if not os.path.isdir(self.__programs_path):
            os.makedirs(self.__programs_path)

        self.__database = Program(SQLiteDAO(db_path))
        self.__python_manager = ProgramsFileManager(f'{self.__programs_path}/python3', '.py', True, 'python3')
        self.__blockly_manager = ProgramsFileManager(f'{self.__programs_path}/blockly', '.xml')

        self.__sync_db_with_system()
        self.__programs = {program['id']: program for program in self.__database.get_all()}

    def __sync_db_with_system(self):
        """
        Do various checks to ensure the database is synced with the real programs files
        """
        db_programs = self.__database.get_all()
        python_programs = self.__python_manager.get_all_names()

        for db_program in db_programs:
            if db_program['id'] not in python_programs:
                rospy.logwarn(f'Removing program "{db_program["name"]}" from database since it has no file')
                self.__database.delete_program(db_program['id'])

    # - Programs handling

    def create_program(self, name, description, python_code, blockly_code=''):
        has_blockly = blockly_code != ''
        program_id = str(uuid4())
        new_program = self.__database.insert_program(program_id, name, description, has_blockly)
        self.__programs[program_id] = new_program
        self.__python_manager.create(program_id, python_code)
        if has_blockly:
            self.__blockly_manager.create(program_id, blockly_code)
        return program_id

    def delete_program(self, program_id):
        self.__database.delete_program(program_id)
        self.__python_manager.remove(program_id)
        program = self.__programs.pop(program_id)
        if program['has_blockly']:
            self.__blockly_manager.remove(program_id)

    def get(self, id_):
        db_program = dict(self.__programs[id_])
        db_program.update(self.get_program_code(id_))
        return db_program

    def get_all(self):
        return [self.get(id_) for id_ in self.__programs]

    def get_program_code(self, id_):
        return {'python_code': self.__python_manager.read(id_), 'blockly_code': self.__blockly_manager.read(id_)}

    # - Program executions

    @property
    def is_executing_program(self):
        return self.__python_manager.is_running

    def execute_program(self, id_, output_stream):
        self.__python_manager.execute_from_name(id_, output_stream)

    def execute_from_code(self, python_code, output_stream):
        with NamedTemporaryFile(prefix='tmp_program', mode='w') as program_file:
            program_file.write(python_code)
            self.__python_manager.execute_from_path(program_file.name, output_stream)

    def stop_execution(self):
        self.__python_manager.stop_execution()
