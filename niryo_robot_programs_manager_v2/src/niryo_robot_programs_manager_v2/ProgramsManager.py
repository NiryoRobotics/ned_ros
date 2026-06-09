import os
from queue import Queue
from threading import Thread, Event
from typing import TypedDict, List, Callable
from uuid import uuid4

import rospy
from niryo_robot_system_api_client import system_api_client

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

    def __init__(self, programs_path: str):
        self.__programs_path = programs_path
        if not os.path.isdir(self.__programs_path):
            os.makedirs(self.__programs_path)

        self.__python_manager = ProgramsFileManager(f'{self.__programs_path}/python3', '.py')
        self.__blockly_manager = ProgramsFileManager(f'{self.__programs_path}/blockly', '.xml')

        self.__sync_db_with_system()
        self.__programs = {db_program['id']: db_program for db_program in self.get_all()}

        self.__python_runner = PythonRunner()

    def __sync_db_with_system(self) -> None:
        """
        Do various checks to ensure the database is synced with the files
        """
        try:
            system_api_client.wait_for_api()
        except TimeoutError:
            rospy.logerr(f'Failed to get programs information: API is not available')
            return

        response = system_api_client.get_programs()
        if not response.success:
            rospy.logerr(f'Failed to get programs information: {response.detail} ({response.code})')
            return

        python_programs = self.__python_manager.get_all_names()
        blockly_programs = self.__blockly_manager.get_all_names()

        db_programs_ids = []
        for db_program in response.data:
            db_programs_ids.append(db_program['id'])
            if db_program['id'] not in python_programs:
                rospy.logwarn(f'Removing program "{db_program["id"]}" from database since it has no file')
                delete_response = system_api_client.delete_program(db_program['id'])
                if not delete_response.success:
                    rospy.logerr(f'Failed to delete program "{db_program["id"]}": {delete_response.detail}')
            elif db_program['has_blockly'] and db_program['id'] not in blockly_programs:
                rospy.logwarn(f'No blockly file found for "{db_program["id"]}". Setting "has_blockly" to False')
                update_response = system_api_client.update_program(db_program['id'], has_blockly=False)
                if not update_response.success:
                    rospy.logerr(f'Failed to update program "{db_program["id"]}": {update_response.detail}')

        for python_program in python_programs:
            if python_program not in db_programs_ids:
                rospy.logwarn(f'Found python program "{python_program}" not referenced in database')
                add_response = system_api_client.add_program(id_=python_program,
                                                             name=python_program,
                                                             description='Unknown program',
                                                             has_blockly=python_program in blockly_programs)
                if not add_response.success:
                    rospy.logerr(f'Failed to add program "{python_program}": {add_response.detail}')

    # - Programs handling

    def create_program(self, name: str, description: str, python_code: str, blockly_code: str = '') -> str:
        has_blockly = blockly_code != ''
        program_id = str(uuid4())
        response = system_api_client.add_program(id_=program_id,
                                                 name=name,
                                                 description=description,
                                                 has_blockly=has_blockly)
        if not response.success:
            raise RuntimeError(f'Failed to create program "{name}": {response.detail}')
        self.__python_manager.write(program_id, python_code)
        if has_blockly:
            self.__blockly_manager.write(program_id, blockly_code)

        self.__programs[program_id] = self.get(program_id)
        return program_id

    def delete_program(self, program_id: str) -> None:
        program = self.__programs.get(program_id)
        response = system_api_client.delete_program(program_id)
        if not response.success:
            raise RuntimeError(f'Failed to delete program "{program_id}": {response.detail}')
        self.__python_manager.remove(program_id)
        self.__programs.pop(program_id)
        if program and program['has_blockly']:
            self.__blockly_manager.remove(program_id)

    def update_program(self, program_id: str, name: str, description: str, python_code: str, blockly_code: str) -> None:
        has_blockly = blockly_code != ''
        response = system_api_client.update_program(program_id,
                                                    name=name,
                                                    description=description,
                                                    has_blockly=has_blockly)
        if not response.success:
            raise RuntimeError(f'Failed to update program "{program_id}": {response.detail}')
        self.__python_manager.write(program_id, python_code, overwrite_allowed=True)
        if has_blockly:
            self.__blockly_manager.write(program_id, blockly_code, overwrite_allowed=True)
        self.__programs.pop(program_id)
        self.__programs[program_id] = self.get(program_id)

    def exists(self, program_id: str) -> bool:
        response = system_api_client.program_exists(program_id)
        if not response.success:
            return False
        return response.data.get('exists', False) and self.__python_manager.exists(program_id)

    def get(self, program_id: str) -> ProgramDict:
        """
        Retrieve a program by doing a request to the database and by reading the files on the system
        :param program_id: the wanted program's id
        :type program_id: str
        :return: The program information
        :rtype: ProgramDict
        """
        response = system_api_client.get_program(program_id)
        if not response.success:
            raise RuntimeError(f'Failed to get program "{program_id}": {response.detail}')
        db_program = response.data
        db_program['python_code'] = self.__python_manager.read(program_id)
        db_program['blockly_code'] = self.__blockly_manager.read(program_id) if db_program['has_blockly'] else ''
        return db_program

    def get_all(self) -> List[ProgramDict]:
        """
        Returns all the programs.
        This method is costly as it calls the database and the system files multiple times.
        Prefer using the programs property
        :return: The programs information
        :rtype: List[ProgramDict]
        """
        response = system_api_client.get_programs()
        if not response.success:
            rospy.logerr(f'Failed to get all programs: {response.detail}')
            return []
        return [self.get(db_program['id']) for db_program in response.data]

    @property
    def programs(self) -> List[ProgramDict]:
        """
        Returns the cached programs.
        This method is more efficient than get_all as it do not directly use the database nor the file
        :return: The programs information
        :rtype: List[ProgramDict]
        """
        return list(self.__programs.values())

    # - Program executions

    @property
    def output_queue(self) -> Queue:
        return self.__python_runner.output_queue

    @property
    def execution_is_running(self) -> bool:
        return self.__python_runner.is_running

    @property
    def execution_is_success(self) -> bool:
        return self.__python_runner.exit_status == 0

    def execute_from_id(self, program_id: str, in_thread: bool = True) -> None:

        def execute_program(*args, **kwargs) -> None:
            file_path = self.__python_manager.get_file_path(program_id)
            self.__python_runner.start(file_path, *args, **kwargs)

        if in_thread:
            self.__execute_in_thread(execute_program)
        else:
            execute_program()

    def execute_from_code(self, python_code: str, in_thread: bool = True) -> None:

        def execute_program(*args, **kwargs) -> None:
            with self.__python_manager.temporary_file(python_code) as tmp_file_path:
                self.__python_runner.start(tmp_file_path, *args, **kwargs)

        if in_thread:
            self.__execute_in_thread(execute_program)
        else:
            execute_program()

    @staticmethod
    def __execute_in_thread(method: Callable, *args, **kwargs) -> None:
        execution_started_event = Event()
        kwargs['execution_started_event'] = execution_started_event
        Thread(target=method, args=args, kwargs=kwargs, daemon=True).start()
        execution_started_event.wait(5)

    def stop_execution(self) -> None:
        self.__python_runner.stop()
