#!/usr/bin/env python

import rospy

import rostest
import unittest

from niryo_robot_python_ros_wrapper import *

import os
import shutil
import numpy as np
import math

# Pins
list_pins = [PinID.GPIO_1A, PinID.GPIO_1B, PinID.GPIO_1C, PinID.GPIO_2A, PinID.GPIO_2B, PinID.GPIO_2C]

# Some Poses
pose_1 = [0.25, -0.05, 0.2, 0.0, 0.7, 0.0]
pose_2 = [0.17, 0.15, 0.25, 0.0, 0.7, 0.0]
pose_3 = [0.15, 0.10, 0.2, 0.0, 0.7, 0.0]

pose_q_1 = [0.25, 0.0, 0.2, 0, math.sqrt(2) / 2, 0, math.sqrt(2) / 2]
pose_q_2 = [0.17, 0.15, 0.2, 0, math.sqrt(2) / 2, 0, math.sqrt(2) / 2]
pose_q_3 = [0.15, 0.10, 0.2, 0, math.sqrt(2) / 2, 0, math.sqrt(2) / 2]

position_dir = trajectory_dir = workspace_dir = None
simulation_mode = use_gripper_n_camera = None


def clean_folders():
    folders_list_raw = [position_dir, trajectory_dir, workspace_dir]
    folders_list = [os.path.expanduser(folder) for folder in folders_list_raw]
    for folder in folders_list:
        if os.path.isdir(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)


class TestPythonWrapper(unittest.TestCase):
    """
    This class contains functions that are available whether you are on simulation
    or on reality, with or without camera/gripper
    """

    @staticmethod
    def assertAlmostEqualVector(a, b, decimal=1):
        np.testing.assert_almost_equal(a, b, decimal)

    def assertStatus(self, ret):
        self.assertIsNotNone(ret, msg="Assert Status cannot operate on None ret")
        self.assertTrue(ret[0] > 0, msg="status : {} - message : {}".format(ret[0], ret[1]))

    def assertNotStatus(self, ret):
        self.assertIsNotNone(ret, msg="AssertNotStatus cannot operate on None ret")
        self.assertTrue(ret[0] < 0, msg="status : {} - message : {}".format(ret[0], ret[1]))

    def setUp(self):
        clean_folders()
        self.niryo_robot = NiryoRosWrapper()

    def tearDown(self):
        del self.niryo_robot
        clean_folders()

    # -- Move

    # - Main Purpose
    def test_calibrate(self):
        if simulation_mode:
            self.assertStatus(self.niryo_robot.calibrate_manual())
        self.assertStatus(self.niryo_robot.calibrate_auto())

    def test_learning_mode(self):
        self.assertStatus(self.niryo_robot.set_learning_mode(True))
        self.assertTrue(self.niryo_robot.get_learning_mode())
        self.assertStatus(self.niryo_robot.set_learning_mode(False))
        self.assertFalse(self.niryo_robot.get_learning_mode())

    def test_set_arm_max_velocity(self):
        self.assertStatus(self.niryo_robot.set_arm_max_velocity(58))
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.set_arm_max_velocity(200)

    # - Pose

    def test_joints(self):
        self.assertStatus(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqualVector(self.niryo_robot.get_joints(), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertStatus(self.niryo_robot.move_to_sleep_pose())

    def test_pose(self):
        target_pose = [0.15, 0.0, 0.15, 0.0, 1.0, 0.0]
        self.assertStatus(self.niryo_robot.move_pose(*target_pose))
        new_pose = self.niryo_robot.get_pose_as_list()
        self.assertAlmostEqualVector(new_pose[:3], target_pose[:3], decimal=1)
        self.assertAlmostEqualVector(new_pose[3:], target_pose[3:], decimal=1)
        self.assertStatus(self.niryo_robot.move_to_sleep_pose())

    def test_linear_pose(self):
        target_pose = [0.3, 0.09, 0.36, 0.0, 1.0, 0.0]
        self.assertStatus(self.niryo_robot.move_linear_pose(*target_pose))
        new_pose = self.niryo_robot.get_pose_as_list()
        self.assertAlmostEqualVector(new_pose[:3], target_pose[:3], decimal=1)
        self.assertAlmostEqualVector(new_pose[3:], target_pose[3:], decimal=1)
        self.assertStatus(self.niryo_robot.shift_linear_pose(ShiftPose.AXIS_Y, -0.04))
        self.assertStatus(self.niryo_robot.move_to_sleep_pose())

    def test_kinematics(self):
        self.assertStatus(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        initial_pose = self.niryo_robot.get_pose_as_list()
        # Forward Kinematics
        joints_target = [0.2, 0.0, -0.4, 0.0, 0.0, 0.0]
        pose_target = self.niryo_robot.forward_kinematics(*joints_target)
        self.assertStatus(self.niryo_robot.move_pose(*pose_target))
        joints_reached = self.niryo_robot.get_joints()
        self.assertAlmostEqualVector(joints_target, joints_reached)
        # Inverse Kinematics
        joints_target_to_initial_pose = self.niryo_robot.inverse_kinematics(*initial_pose)
        self.assertStatus(self.niryo_robot.move_joints(*joints_target_to_initial_pose))
        pose_reached = self.niryo_robot.get_pose_as_list()
        self.assertAlmostEqualVector(initial_pose, pose_reached)

    def test_pose_saved(self):
        self.assertEqual(self.niryo_robot.get_saved_pose_list(), [])
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.delete_pose("NotExistent")  # Do not exist
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.get_pose_saved("NotExistent")  # Do not exist
        list_names = []
        for i in range(3):
            name = "Test{}".format(i)
            self.assertStatus(self.niryo_robot.save_pose(name, *pose_1))
            self.assertEqual(self.niryo_robot.get_pose_saved(name), tuple(pose_1))
            list_names.append(name)
            self.assertEqual(self.niryo_robot.get_saved_pose_list(), list_names)
        self.assertStatus(self.niryo_robot.move_pose_saved("Test0"))
        for name in list_names:
            self.assertStatus(self.niryo_robot.delete_pose(name))
        self.assertEqual(self.niryo_robot.get_saved_pose_list(), [])

    def test_trajectory(self):
        self.assertStatus(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.assertStatus(self.niryo_robot.shift_pose(ShiftPose.AXIS_Z, -0.04))
        self.assertStatus(self.niryo_robot.move_pose(*pose_1))
        self.assertStatus(self.niryo_robot.move_linear_pose(*pose_2))
        new_pose = self.niryo_robot.get_pose_as_list()
        self.assertAlmostEqualVector(new_pose[:3], pose_2[:3])
        new_pose = self.niryo_robot.get_pose_as_list()
        self.assertAlmostEqualVector(new_pose[:3], pose_3[:3])
        self.assertStatus(self.niryo_robot.execute_trajectory_from_poses([pose_1, pose_2, pose_3]))
        self.assertAlmostEqualVector(new_pose[:3], pose_3[:3])

    def test_trajectory_saved(self):
        self.assertEqual(self.niryo_robot.get_saved_trajectory_list(), [])
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.delete_trajectory("NotExistent")  # Do not exist
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.get_trajectory_saved("NotExistent")  # Do not exist
        list_names = []
        for i in range(3):
            name = "Test{}".format(i)
            self.assertStatus(self.niryo_robot.save_trajectory(name, [pose_q_1, pose_q_2, pose_q_3]))
            self.assertEqual(self.niryo_robot.get_trajectory_saved(name), [pose_q_1, pose_q_2, pose_q_3])
            list_names.append(name)
            self.assertEqual(self.niryo_robot.get_saved_trajectory_list(), list_names)
        for name in list_names:
            self.assertStatus(self.niryo_robot.delete_trajectory(name))
        self.assertEqual(self.niryo_robot.get_saved_trajectory_list(), [])

    def test_jog_controller(self):
        self.assertStatus(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        rospy.sleep(.4)  # Making sure the previous command stopped
        self.assertStatus(self.niryo_robot.set_jog_use_state(True))
        self.assertStatus(self.niryo_robot.set_jog_use_state(False))
        self.assertStatus(self.niryo_robot.set_jog_use_state(True))

        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.move_joints(0.01, 0.0, 0.0, 0.0, 0.0, 0.0)  # Should not be allowed

        self.assertStatus(self.niryo_robot.jog_pose_shift([0.02, 0.0, 0.0, 0.0, 0.0, 0.0]))
        rospy.sleep(1)
        self.assertStatus(self.niryo_robot.jog_pose_shift([0.00, -0.02, -0.01, 0.0, 0.0, 0.0]))
        rospy.sleep(1)
        self.assertStatus(self.niryo_robot.jog_joints_shift([-0.01, -0.02, 0.0, 0.0, 0.0, 0.0]))
        rospy.sleep(1)
        self.assertStatus(self.niryo_robot.set_jog_use_state(False))

    # - Workspace
    def test_workspace(self):
        self.assertEqual(self.niryo_robot.get_workspace_list(), [])
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.delete_workspace("NotExistent")  # Do not exist
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.get_workspace_poses("NotExistent")  # Do not exist
        points = [[0.278, 0.104, 0.058], [0.275, -0.036, 0.058],
                  [0.125, -0.040, 0.059], [0.127, 0.109, 0.059]]
        robot_poses = [[0.281, 0.102, 0.083, 2.200, 1.439, 2.367],
                       [0.276, -0.03, 0.083, 2.058, 1.473, 1.704],
                       [0.123, -0.04, 0.084, 0.132, 1.499, -0.12],
                       [0.121, 0.110, 0.083, -0.19, 1.339, -0.19]]
        list_names = []
        for i in range(3):
            name = "Test{}".format(2 * i)
            self.assertStatus(self.niryo_robot.save_workspace_from_poses(name, robot_poses))
            self.assertEqual(self.niryo_robot.get_workspace_poses(name), robot_poses)
            name2 = "Test{}".format(2 * i + 1)
            self.assertStatus(self.niryo_robot.save_workspace_from_points(name2, points))

            list_names.append(name)
            list_names.append(name2)
            self.assertEqual(self.niryo_robot.get_workspace_list(), list_names)

        for name in list_names:
            self.assertStatus(self.niryo_robot.delete_workspace(name))
        self.assertEqual(self.niryo_robot.get_workspace_list(), [])


class TestPythonWrapperSimulation(TestPythonWrapper):
    """
    Simulation test -> used on Rviz
    """
    pass


# noinspection PyTypeChecker
class TestPythonWrapperSimulationWithGripperAndCamera(TestPythonWrapperSimulation):
    """
    Simulation test with gripper and tool -> Used on Gazebo
    """

    def test_tool(self):
        self.assertStatus(self.niryo_robot.update_tool())
        self.assertStatus(self.niryo_robot.grasp_with_tool())
        self.assertStatus(self.niryo_robot.release_with_tool())

    # - Vision
    def test_vision(self):
        self.assertIsNotNone(self.niryo_robot.get_camera_intrinsics())
        workspace_name = "gazebo_1"
        workspace_h = 0.001
        point_1 = [0.3369, 0.087, workspace_h]
        point_2 = [point_1[0], -point_1[1], workspace_h]
        point_3 = [0.163, -point_1[1], workspace_h]
        point_4 = [point_3[0], point_1[1], workspace_h]
        self.assertStatus(self.niryo_robot.save_workspace_from_points(
            workspace_name, [point_1, point_2, point_3, point_4]))

        self.assertStatus(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0))
        self.assertStatus(self.niryo_robot.update_tool())
        self.assertIsNotNone(self.niryo_robot.get_target_pose_from_rel(workspace_name, 0.1, 0.5, 0.5, 0.0))
        self.assertIsNotNone(self.niryo_robot.get_target_pose_from_cam(
            workspace_name, 0.1, ObjectShape.ANY, ObjectColor.ANY))
        self.assertIsNotNone(self.niryo_robot.detect_object(workspace_name, ObjectShape.ANY, ObjectColor.RED))

        self.assertIsNotNone(self.niryo_robot.move_to_object(workspace_name, 0.03, ObjectShape.ANY, ObjectColor.GREEN))

        self.assertStatus(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0))
        self.assertIsNotNone(self.niryo_robot.vision_pick(workspace_name, 0.002, ObjectShape.ANY, ObjectColor.BLUE))


class TestPythonWrapperRasp(TestPythonWrapper):
    """
    Test on Raspberry
    """

    def setUp(self):
        super(TestPythonWrapperRasp, self).setUp()
        self.assertStatus(self.niryo_robot.calibrate_auto())
        rospy.sleep(0.5)

    def tearDown(self):
        self.assertStatus(self.niryo_robot.set_learning_mode(True))
        super(TestPythonWrapperRasp, self).tearDown()

    # - Hardware
    def test_hardware(self):
        for pin in list_pins:
            self.assertStatus(self.niryo_robot.set_pin_mode(pin, PinMode.OUTPUT))
            self.assertStatus(self.niryo_robot.digital_write(pin, PinState.LOW))
            self.assertEqual(self.niryo_robot.digital_read(pin), PinState.LOW)
            self.assertStatus(self.niryo_robot.digital_write(pin, PinState.HIGH))
            self.assertEqual(self.niryo_robot.digital_read(pin), PinState.HIGH)

            self.assertStatus(self.niryo_robot.set_pin_mode(pin, PinMode.INPUT))
            with self.assertRaises(NiryoRosWrapperException):
                self.niryo_robot.digital_write(pin, PinState.LOW)


# noinspection PyTypeChecker
class TestPythonWrapperRaspWithConveyor(TestPythonWrapperRasp):
    """
    Test on Raspberry with conveyor
    """

    def test_conveyor(self):
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.unset_conveyor(ConveyorID.ID_1)
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.unset_conveyor(-1)
        with self.assertRaises(NiryoRosWrapperException):
            self.niryo_robot.control_conveyor(ConveyorID.ID_1, True, speed=50,
                                              direction=ConveyorDirection.FORWARD)

        conveyor_id = self.niryo_robot.set_conveyor()
        assert conveyor_id in [ConveyorID.ID_1, ConveyorID.ID_2]
        self.assertStatus(self.niryo_robot.control_conveyor(conveyor_id, bool_control_on=True, speed=50,
                                                            direction=ConveyorDirection.FORWARD))
        self.assertStatus(self.niryo_robot.control_conveyor(conveyor_id, bool_control_on=False, speed=50,
                                                            direction=ConveyorDirection.BACKWARD))
        self.assertStatus(self.niryo_robot.unset_conveyor(conveyor_id))


if __name__ == '__main__':
    rospy.init_node('niryo_robot_test_python_ros_wrapper')
    simulation_mode = rospy.get_param('~simulation_mode')
    use_gripper_n_camera = rospy.get_param('~gripper_n_camera')
    use_conveyor = rospy.get_param('~conveyor')

    NiryoRosWrapper.wait_for_nodes_initialization(simulation_mode)

    position_dir = rospy.get_param('/niryo_robot_poses_handlers/poses_dir')
    trajectory_dir = rospy.get_param('/niryo_robot_poses_handlers/trajectories_dir')
    workspace_dir = rospy.get_param('/niryo_robot_poses_handlers/workspace_dir')

    if simulation_mode:
        if use_gripper_n_camera:
            rostest.rosrun("python_ros_wrapper", "test_python_ros_wrapper_simulation",
                           TestPythonWrapperSimulationWithGripperAndCamera)
        else:
            rostest.rosrun("python_ros_wrapper", "test_python_ros_wrapper_simulation", TestPythonWrapperSimulation)
    else:
        if use_conveyor:
            rostest.rosrun("python_ros_wrapper", "test_python_ros_wrapper_Rasp", TestPythonWrapperRaspWithConveyor)
        else:
            rostest.rosrun("python_ros_wrapper", "test_python_ros_wrapper_Rasp", TestPythonWrapperRasp)
