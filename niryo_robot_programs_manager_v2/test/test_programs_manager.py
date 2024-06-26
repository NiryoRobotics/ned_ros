#!/usr/bin/env python
import rospy
import rostest
import os
import unittest
import shutil

from niryo_robot_programs_manager_v2.srv import GetProgram, GetProgramRequest, CreateProgramRequest, CreateProgram, \
    DeleteProgramRequest, DeleteProgram
from niryo_robot_programs_manager_v2.srv import GetProgramAutorunInfos
from niryo_robot_programs_manager_v2.srv import SetProgramAutorun


def call_service(service_name, service_msg_type, *args):
    service_timeout = 3
    # Connect to service
    rospy.wait_for_service(service_name, service_timeout)
    # Call service
    service = rospy.ServiceProxy(service_name, service_msg_type)
    response = service(*args)
    return response


def clean_folders():
    folders_list_raw = [python3_dir, blockly_dir]
    folders_list = [os.path.expanduser(folder) for folder in folders_list_raw]
    for folder in folders_list:
        if os.path.isdir(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)


def create_python3_code(name):
    TestPython3Manager.save_program(name, "niryo_robot.calib()", "")


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
    def save_program(cls, name, code, description):
        req = CreateProgramRequest()
        req.name = name
        req.python_code = code
        req.description = description
        return call_service('/niryo_robot_programs_manager_v2/create_program', CreateProgram, req)

    @classmethod
    def delete_program(cls, id_):
        req = DeleteProgramRequest()
        req.program_id = id_
        return call_service('/niryo_robot_programs_manager_v2/delete_program', DeleteProgram, req)

    @classmethod
    def get_program(cls, id_):
        req = GetProgramRequest()
        req.program_id = id_
        return call_service('/niryo_robot_programs_manager_v2/get_program', GetProgram, req)

    # - Abstract test methods

    def abstract_creation_delete_prog(self):
        list_names = []
        for i in range(5):
            name = "Test{}".format(i)
            self.assertStatus(self.save_program(name, py_code, py_description))
            ret_get_prog = self.get_program(name)
            self.assertStatus(ret_get_prog)

            # self.assertEqual(code, ret_get_prog.code)
            self.assertEqual(self.description, ret_get_prog.description)

            list_names.append(name)
        for name in list_names:
            self.assertStatus(self.delete_program(name))

    def abstract_overwrite(self):
        name = "Test_execute"
        self.assertStatus(self.save_program(name, py_code, py_description))

        self.assertNotStatus(self.save_program(name, py_code, py_description))

        self.assertStatus(self.save_program(name, py_code, py_description))

        self.assertStatus(self.delete_program(name))


py_code = """
import math
print(math.pi)
"""
py_description = "Display PI"


class TestPython3Manager(TestProgramManagerAbstract):
    code = py_code
    description = py_description

    def test_python3_overwrite(self):
        self.abstract_overwrite()


class TestBlocklyManager(TestProgramManagerAbstract):
    code = "<some_blockly_code>"
    description = ""

    def test_blockly_creation_delete_prog(self):
        self.abstract_creation_delete_prog()

    def test_blockly_overwrite(self):
        self.abstract_overwrite()


if __name__ == '__main__':
    while not rospy.has_param('/niryo_robot_programs_manager_v2/initialized'):
        rospy.sleep(0.10)

    programs_dir = os.path.expanduser(rospy.get_param('/niryo_robot_programs_manager_v2/programs_dir'))
    python3_dir = os.path.join(programs_dir, "python3")
    blockly_dir = os.path.join(programs_dir, "blockly")
    # Going to execute all unittest.TestCase subclasses in the file -> Import are also concerned
    rostest.rosrun("programs_manager", "test_programs_manager", __name__)
