#!/usr/bin/env python

import rospy
import rostest
import os
import unittest
import shutil

from niryo_robot_programs_manager.msg import ProgramLanguage, ProgramLanguageList
from niryo_robot_programs_manager.srv import ExecuteProgram, ExecuteProgramRequest
from niryo_robot_programs_manager.srv import GetProgram, GetProgramRequest
from niryo_robot_programs_manager.srv import GetProgramAutorunInfos
from niryo_robot_programs_manager.srv import GetProgramList, GetProgramListRequest
from niryo_robot_programs_manager.srv import ManageProgram, ManageProgramRequest
from niryo_robot_programs_manager.srv import SetProgramAutorun, SetProgramAutorunRequest

python_language_msg = ProgramLanguage(ProgramLanguage.PYTHON2)
blockly_language_msg = ProgramLanguage(ProgramLanguage.BLOCKLY)


def call_service(service_name, service_msg_type, *args):
    service_timeout = 3
    # Connect to service
    rospy.wait_for_service(service_name, service_timeout)
    # Call service
    service = rospy.ServiceProxy(service_name, service_msg_type)
    response = service(*args)
    return response


def clean_folders():
    folders_list_raw = [python2_dir, blockly_dir]
    folders_list = [os.path.expanduser(folder) for folder in folders_list_raw]
    for folder in folders_list:
        if os.path.isdir(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)


def create_python2_code(name):
    TestPython2Manager.save_program(name, "niryo_robot.calib()", "")


class TestProgramManagerAbstract(unittest.TestCase):
    language = None
    code = None
    description = None

    def setUp(self):
        clean_folders()

    def tearDown(self):
        clean_folders()

    def assertStatus(self, ret):
        self.assertIsNotNone(ret, msg="Assert Status cannot operate on None ret")
        self.assertTrue(ret.status > 0, msg="status : {} - message : {}".format(ret.status, ret.message))

    def assertNotStatus(self, ret):
        self.assertIsNotNone(ret, msg="AssertNotStatus cannot operate on None ret")
        self.assertTrue(ret.status < 0, msg="status : {} - message : {}".format(ret.status, ret.message))

    @classmethod
    def save_program(cls, name, code, description, allow_overwrite=False):
        req = ManageProgramRequest()
        req.cmd = ManageProgramRequest.SAVE
        req.name = name
        req.language = cls.language
        req.code = code
        req.description = description
        req.allow_overwrite = allow_overwrite
        return call_service('/niryo_robot_programs_manager/manage_program', ManageProgram, req)

    @classmethod
    def delete_program(cls, name):
        req = ManageProgramRequest()
        req.cmd = ManageProgramRequest.DELETE
        req.name = name
        req.language = cls.language
        return call_service('/niryo_robot_programs_manager/manage_program', ManageProgram, req)

    @classmethod
    def get_program(cls, name):
        req = GetProgramRequest()
        req.name = name
        req.language = cls.language
        return call_service('/niryo_robot_programs_manager/get_program', GetProgram, req)

    @classmethod
    def get_program_list(cls):
        req = GetProgramListRequest()
        req.language = cls.language
        return call_service('/niryo_robot_programs_manager/get_program_list', GetProgramList, req)

    @classmethod
    def get_program_name_list(cls):
        return cls.get_program_list().programs_names

    @classmethod
    def execute_program_from_name(cls, name):
        return cls.execute_program(execute_from_string=False, name=name, code_string="")

    @classmethod
    def execute_program_from_string(cls, string_):
        return cls.execute_program(execute_from_string=True, name="", code_string=string_)

    @classmethod
    def execute_program(cls, execute_from_string, name, code_string):
        req = ExecuteProgramRequest()
        req.execute_from_string = execute_from_string
        req.name = name
        req.code_string = code_string
        req.language = cls.language
        return call_service('/niryo_robot_programs_manager/execute_program', ExecuteProgram, req)

    # - Abstract test methods

    def abstract_no_prog(self):
        self.assertEqual(self.get_program_name_list(), [])
        self.assertNotStatus(self.get_program("DontExist"))  # Do not exist
        self.assertNotStatus(self.delete_program("DontExist"))  # Do not exist

    def abstract_creation_delete_prog(self):
        self.assertEqual(self.get_program_name_list(), [])
        list_names = []
        for i in range(5):
            name = "Test{}".format(i)
            self.assertStatus(self.save_program(name, py_code, py_description))
            ret_get_prog = self.get_program(name)
            self.assertStatus(ret_get_prog)

            # self.assertEqual(code, ret_get_prog.code)
            self.assertEqual(self.description, ret_get_prog.description)

            list_names.append(name)
            self.assertEqual(self.get_program_name_list(), list_names, "Read All Failed")
        for name in list_names:
            self.assertStatus(self.delete_program(name))
        self.assertEqual(self.get_program_name_list(), [], "Read All Failed")

    def abstract_overwrite(self):
        name = "Test_execute"
        self.assertStatus(self.save_program(name, py_code, py_description))

        self.assertNotStatus(self.save_program(name, py_code, py_description, allow_overwrite=False))

        self.assertStatus(self.save_program(name, py_code, py_description, allow_overwrite=True))

        self.assertStatus(self.delete_program(name))

    def abstract_execute_prog(self):
        name = "Test_execute"
        self.assertStatus(self.save_program(name, py_code, py_description))

        self.assertStatus(self.execute_program_from_name(name))

        self.assertStatus(self.delete_program(name))

        self.assertStatus(self.execute_program_from_string(py_code))


py_code = """
import math
print math.pi
"""
py_description = "Display PI"


class TestPython2Manager(TestProgramManagerAbstract):
    language = python_language_msg
    code = py_code
    description = py_description

    def test_python2_no_prog(self):
        self.abstract_no_prog()

    def test_python2_creation_delete_prog(self):
        self.abstract_creation_delete_prog()

    def test_python2_overwrite(self):
        self.abstract_overwrite()

    def test_python2_execute_prog(self):
        self.abstract_execute_prog()


class TestBlocklyManager(TestProgramManagerAbstract):
    language = blockly_language_msg
    code = "<some_blockly_code>"
    description = ""

    def test_blockly_no_prog(self):
        self.abstract_no_prog()

    def test_blockly_creation_delete_prog(self):
        self.abstract_creation_delete_prog()

    def test_blockly_overwrite(self):
        self.abstract_overwrite()


class TestAllManager(TestProgramManagerAbstract):
    language = ProgramLanguage(ProgramLanguage.ALL)

    def test_all_no_prog(self):
        self.abstract_no_prog()


class TestAutorun(TestProgramManagerAbstract):
    test_python2_code_name = "test_code_python2"

    @staticmethod
    def set_autorun_program_from_req(req):
        return call_service('/niryo_robot_programs_manager/set_program_autorun', SetProgramAutorun, req)

    @staticmethod
    def get_program_autorun():
        return call_service('/niryo_robot_programs_manager/get_program_autorun_infos', GetProgramAutorunInfos)

    def test_set_autorun(self):
        create_python2_code(self.test_python2_code_name)
        req = SetProgramAutorunRequest()
        req.language = blockly_language_msg
        req.name = "unknown_name"
        req.mode = SetProgramAutorunRequest.LOOP

        self.assertNotStatus(self.set_autorun_program_from_req(req))
        req.name = self.test_python2_code_name
        self.assertNotStatus(self.set_autorun_program_from_req(req))

        req.language = python_language_msg
        self.assertStatus(self.set_autorun_program_from_req(req))

    def test_get_autorun(self):
        create_python2_code(self.test_python2_code_name)
        req = SetProgramAutorunRequest()
        req.name = self.test_python2_code_name
        req.language = python_language_msg
        req.mode = SetProgramAutorunRequest.ONE_SHOT
        self.assertStatus(self.set_autorun_program_from_req(req))

        infos = self.get_program_autorun()
        self.assertStatus(infos)

        self.assertEqual(infos.language, python_language_msg)
        self.assertEqual(infos.name, self.test_python2_code_name)
        self.assertEqual(infos.mode, SetProgramAutorunRequest.ONE_SHOT)


if __name__ == '__main__':
    while not rospy.has_param('/niryo_robot_programs_manager/initialized'):
        rospy.sleep(0.10)

    programs_dir = os.path.expanduser(rospy.get_param('/niryo_robot_programs_manager/programs_dir'))
    python2_dir = os.path.join(programs_dir, "python2")
    blockly_dir = os.path.join(programs_dir, "blockly")
    # Going to execute all unittest.TestCase subclasses in the file -> Import are also concerned
    rostest.rosrun("programs_manager", "test_programs_manager", __name__)
