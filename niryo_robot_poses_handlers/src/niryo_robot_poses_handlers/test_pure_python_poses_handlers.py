#!/usr/bin/env python

import unittest
import os
import shutil

from niryo_robot_poses_handlers.file_manager import NiryoRobotFileException

import niryo_robot_poses_handlers.pose_manager as pose_manager
import niryo_robot_poses_handlers.trajectory_manager as trajectory_manager
import niryo_robot_poses_handlers.workspace_manager as workspace_manager

test_folder = "~/Tests_Niryo_Ned/"
test_folder = os.path.expanduser(test_folder)

points = [[0.278, 0.104, 0.058], [0.275, -0.036, 0.058],
          [0.125, -0.040, 0.059], [0.127, 0.109, 0.059]]
robot_poses = [[[0.281, 0.102, 0.083], [2.200, 1.439, 2.367]],
               [[0.276, -0.03, 0.083], [2.058, 1.473, 1.704]],
               [[0.123, -0.04, 0.084], [0.132, 1.499, -0.12]],
               [[0.121, 0.110, 0.083], [-0.19, 1.339, -0.19]]]


def create_test_folder():
    if not os.path.isdir(test_folder):
        os.makedirs(test_folder)


def delete_test_folder():
    if os.path.isdir(test_folder):
        shutil.rmtree(test_folder)


def check_get_read_remove(obj):
    obj.assertEqual(obj.manager.get_all_names(), [])
    obj.assertFalse(obj.manager.exists("NotExist"))
    with obj.assertRaises(NiryoRobotFileException):
        obj.manager.read("NoOne")
    with obj.assertRaises(NiryoRobotFileException):
        obj.manager.remove("NoOne")


class TestPoseMethods(unittest.TestCase):

    def setUp(self):
        create_test_folder()
        self.path = os.path.join(test_folder, "positions")
        self.manager = pose_manager.PoseManager(position_dir=self.path)

    def tearDown(self):
        shutil.rmtree(self.path)
        delete_test_folder()

    def test_python_get_read_remove_pos(self):
        check_get_read_remove(self)

    def test_python_creation_delete_pos(self):
        self.assertEqual(self.manager.get_all_names(), [])
        list_names = []
        pose = [[0., 1., 2., 3., 4., 5.], [1, 2, 3], [1, 2, 3], [1, 2, 3, 4]]
        for i in range(5):
            name = "Test{}".format(i)
            self.manager.create(name, *pose)
            self.assertEqual(self.manager.read(name).get_value(), pose)

            list_names.append(name)
            self.assertEqual(self.manager.get_all_names(), list_names)
        for name in list_names:
            self.manager.remove(name)
        self.assertEqual(self.manager.get_all_names(), [])


class TestTrajectoryMethods(unittest.TestCase):

    def setUp(self):
        create_test_folder()
        self.path = os.path.join(test_folder, "trajectories")
        self.manager = trajectory_manager.TrajectoryManager(trajectory_dir=self.path)

    def tearDown(self):
        shutil.rmtree(self.path)
        delete_test_folder()

    def test_python_get_read_remove_trajectories(self):
        check_get_read_remove(self)

    def test_python_creation_delete_traj(self):
        self.assertEqual(self.manager.get_all_names(), [])
        list_names = []
        poses = [[[1, 2, 3], [1, 0, 0, 0]],
                 [[1, 2, 3], [1, 0, 0, 0]]]
        for i in range(5):
            name = "Test{}".format(i)
            self.manager.create(name, poses)
            self.assertEqual(self.manager.read(name).get_value(), poses)

            list_names.append(name)
            self.assertEqual(self.manager.get_all_names(), list_names)
        for name in list_names:
            self.manager.remove(name)
        self.assertEqual(self.manager.get_all_names(), [])


class TestWorkspaceMethods(unittest.TestCase):

    def setUp(self):
        create_test_folder()
        self.path = os.path.join(test_folder, "workspaces")
        self.manager = workspace_manager.WorkspaceManager(self.path)

    def tearDown(self):
        shutil.rmtree(self.path)
        delete_test_folder()

    def test_python_get_read_remove_workspaces(self):
        check_get_read_remove(self)

    def test_python_creation_delete_workspace(self):
        self.assertEqual(self.manager.get_all_names(), [])
        list_names = []
        for i in range(3):
            name = "Test{}".format(i)
            self.manager.create(name, points, robot_poses=None)
            self.assertEqual(self.manager.read(name).get_value(), tuple([points, []]))
            self.manager.remove(name)
            self.manager.create(name, points, robot_poses)
            self.assertEqual(self.manager.read(name).get_value(), tuple([points, robot_poses]))

            list_names.append(name)
            self.assertEqual(self.manager.get_all_names(), list_names)
        for name in list_names:
            self.manager.remove(name)
        self.assertEqual(self.manager.get_all_names(), [])


def process_py():
    unittest.main()


if __name__ == '__main__':
    process_py()
