import os
from threading import Thread
from typing import TypedDict, List
from uuid import uuid4

import rospy
from niryo_robot_database.Program import Program
from niryo_robot_database.SQLiteDAO import SQLiteDAO

from .ProgramsFileManager import ProgramsFileManager
from .PythonRunner import PythonRunner

ProgramDict = TypedDict(
    'ProgramDict',
    {
        'id': str,
        'name': str,
        'description': str,
        'saved_at': str,
        'has_blockly': bool,
        'python_code': str,
        'blockly_code': str
    })


class ProgramsManager:

    def __init__(self, db_path: str, programs_path: str):
        self.__programs_path = programs_path
        if not os.path.isdir(self.__programs_path):
            os.makedirs(self.__programs_path)

        self.__database = Program(SQLiteDAO(db_path))
        self.__python_manager = ProgramsFileManager(f'{self.__programs_path}/python3', '.py')
        self.__blockly_manager = ProgramsFileManager(f'{self.__programs_path}/blockly', '.xml')

        self.__sync_db_with_system()
        self.__programs = {program['id']: program for program in self.__database.get_all()}
        self.__python_runner = PythonRunner()

    def __sync_db_with_system(self) -> None:
        """
        Do various checks to ensure the database is synced with the files
        """
        db_programs = self.__database.get_all()
        python_programs = self.__python_manager.get_all_names()
        blockly_programs = self.__blockly_manager.get_all_names()

        for db_program in db_programs:
            if db_program['id'] not in python_programs:
                rospy.logwarn(f'Removing program "{db_program["id"]}" from database since it has no file')
                self.__database.delete_program(db_program['id'])
            if db_program['has_blockly'] and db_program['id'] not in blockly_programs:
                rospy.logwarn(f'No blockly file found for "{db_program["id"]}". Setting "has_blockly" to False')
                self.__database.update_program(db_program['id'], has_blockly=False)

        db_programs_ids = [p['id'] for p in db_programs]
        for python_program in python_programs:
            if python_program not in db_programs_ids:
                rospy.logwarn(f'Found python program "{python_program}" not referenced in database')

    # - Programs handling

    def create_program(self, name: str, description: str, python_code: str, blockly_code: str = '') -> str:
        has_blockly = blockly_code != ''
        program_id = str(uuid4())
        new_program = self.__database.insert_program(program_id, name, description, has_blockly)
        self.__programs[program_id] = new_program
        self.__python_manager.create(program_id, python_code)
        if has_blockly:
            self.__blockly_manager.create(program_id, blockly_code)
        return program_id

    def delete_program(self, program_id: str) -> None:
        self.__database.delete_program(program_id)
        self.__python_manager.remove(program_id)
        program = self.__programs.pop(program_id)
        if program['has_blockly']:
            self.__blockly_manager.remove(program_id)

    def update_program(self,
                       program_id: str,
                       name: str,
                       description: str,
                       python_code: str,
                       blockly_code: str = '') -> None:
        has_blockly = blockly_code != ''
        self.__database.update_program(program_id, name, description, has_blockly)
        self.__python_manager.edit(program_id, python_code)
        if has_blockly:
            self.__blockly_manager.edit(program_id, blockly_code)

    def exists(self, program_id: str) -> bool:
        return self.__database.exists(program_id) and self.__python_manager.exists(program_id)

    def get(self, program_id: str) -> ProgramDict:
        db_program = dict(self.__programs[program_id])
        db_program['python_code'] = self.__python_manager.read(program_id)
        db_program['blockly_code'] = self.__blockly_manager.read(program_id) if db_program['has_blockly'] else ''
        return db_program

    def get_all(self) -> List[ProgramDict]:
        return [self.get(program_id) for program_id in self.__programs]

    # - Program executions

    @property
    def execution_output(self) -> str:
        return self.__python_runner.output

    @property
    def execution_is_running(self) -> bool:
        return self.__python_runner.is_running

    @property
    def execution_is_success(self) -> bool:
        return self.__python_runner.exit_status == 0

    def __execute(self, path: str) -> None:
        self.__execution_thread = Thread(target=self.__python_runner.start, args=[path], daemon=True)
        self.__execution_thread.start()

    def execute_from_id(self, program_id: str) -> None:
        self.__execute(self.__python_manager.get_file_path(program_id))

    def execute_from_code(self, python_code: str) -> None:
        with self.__python_manager.temporary_file(python_code) as tmp_file:
            self.__execute(tmp_file)

    def stop_execution(self) -> None:
        self.__python_runner.stop()
