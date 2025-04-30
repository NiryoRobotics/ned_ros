#!/usr/bin/env python3
# Lib
import functools
import warnings
from typing import Union, List, overload
from threading import Thread

from niryo_robot_poses_handlers.msg import NiryoPose

from niryo_robot_utils.dataclasses.Pose import Pose
from niryo_robot_utils.dataclasses.PoseMetadata import PoseMetadata
from niryo_robot_utils.dataclasses.JointsPosition import JointsPosition
from niryo_robot_utils.dataclasses.types import RobotPosition

# Enums
from niryo_robot_tools_commander.api import ToolsRosWrapper

import rospy
# Actions
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from conveyor_interface.msg import ConveyorFeedbackArray
# Messages
from geometry_msgs.msg import Pose as RosPose, Point, Quaternion

from niryo_robot_arm_commander.msg import ArmMoveCommand, RobotMoveGoal, RobotMoveAction
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tools_interface.msg import Tool

# Command Status
from niryo_robot_msgs.msg import CommandStatus, SoftwareVersion
from niryo_robot_msgs.msg import HardwareStatus, RobotState, RPY
from niryo_robot_msgs.msg import BasicObjectArray

# Services
from niryo_robot_msgs.srv import SetBool, SetInt, Trigger, Ping, SetFloat, GetFloatList, SetFloatList
from niryo_robot_rpi.msg import DigitalIOState, AnalogIOState, StorageStatus
from niryo_robot_status.msg import RobotStatus
from niryo_robot_utils import NiryoRosWrapperException, NiryoActionClient, NiryoTopicValue, AbstractNiryoRosWrapper
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from std_msgs.msg import Bool, Int32, String
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from .CollisionPolicy import CollisionPolicy

# Can't remove this star import because blockly generated programs use
# from niryo_robot_python_ros_wrapper.ros_wrapper import *
# to import all the package
from .ros_wrapper_enums import *


def move_command(move_function):

    @functools.wraps(move_function)
    def wrapper(self, *args, **kwargs):
        # check if a collision happened previously
        if self._collision_detected:
            rospy.logerr(f'{move_function.__name__} called with collision_detected still raised')
            raise NiryoRosWrapperException(
                'clear_collision_detected() must be called before attempting a new move command')

        result = move_function(self, *args, **kwargs)

        # check if a collision happened during the move
        if self._collision_detected and self._collision_policy == CollisionPolicy.HARD:
            _, message = result
            raise NiryoRosWrapperException(message)
        return result

    return wrapper


class NiryoRosWrapper(AbstractNiryoRosWrapper):

    def __init__(self, collision_policy=CollisionPolicy.HARD):
        super(NiryoRosWrapper, self).__init__()
        self._collision_detected = False
        self._collision_policy = collision_policy

        # - Getting ROS parameters
        self.__service_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/service_timeout")
        self.__simulation_mode = rospy.get_param("/niryo_robot/simulation_mode")
        self.__hardware_version = rospy.get_param("/niryo_robot/hardware_version")

        if self.__hardware_version in ['ned', 'ned2', 'ned3pro']:
            self.__node_name = rospy.get_name()
            rospy.Service("~/ping", Trigger, self.__ping_ros_wrapper_callback)
            rospy.wait_for_service("/niryo_robot_status/ping_ros_wrapper", timeout=5)
            self.__advertise_ros_wrapper_srv = rospy.ServiceProxy("/niryo_robot_status/ping_ros_wrapper", Ping)
            self.__advertise_ros_wrapper_srv(self.__node_name, True)
            rospy.on_shutdown(self.__advertise_stop)

        # - Publishers
        # Highlight publisher (to highlight blocks in Blockly interface)
        self.__highlight_block_publisher = rospy.Publisher('/niryo_robot_blockly/highlight_block',
                                                           String,
                                                           queue_size=10)

        # Break point publisher (for break point blocks in Blockly interface)
        self.__break_point_publisher = rospy.Publisher('/niryo_robot_blockly/break_point', Int32, queue_size=10)

        # -- Subscribers

        rospy.Subscriber('/niryo_robot/collision_detected', Bool, self.__callback_collision_detected)

        # - Pose
        self.__joints_ntv = NiryoTopicValue('/joint_states', JointState)
        self.__pose_ntv = NiryoTopicValue('/niryo_robot/robot_state', RobotState, timeout=100)

        # - Hardware
        self.__learning_mode_on_ntv = NiryoTopicValue('/niryo_robot/learning_mode/state', Bool)
        self.__hw_status_ntv = NiryoTopicValue('/niryo_robot_hardware_interface/hardware_status', HardwareStatus)
        self.__digital_io_state_ntv = NiryoTopicValue('/niryo_robot_rpi/digital_io_state', DigitalIOState)
        self.__analog_io_state_ntv = NiryoTopicValue('/niryo_robot_rpi/analog_io_state', AnalogIOState)
        self.__max_velocity_scaling_factor_ntv = NiryoTopicValue('/niryo_robot/max_velocity_scaling_factor', Int32)
        self.__storage_status_ntv = NiryoTopicValue('/niryo_robot_rpi/storage_status', StorageStatus)
        self.__tool_motor_state_ntv = NiryoTopicValue('/niryo_robot_hardware/tools/motor', Tool)

        # - Vision
        self.__compressed_image_message_ntv = NiryoTopicValue('/niryo_robot_vision/compressed_video_stream',
                                                              CompressedImage,
                                                              queue_size=1)
        self.__camera_intrinsics_message_ntv = NiryoTopicValue('/niryo_robot_vision/camera_intrinsics',
                                                               CameraInfo,
                                                               queue_size=1)
        # - Conveyor
        self.__conveyors_feedback_ntv = NiryoTopicValue('/niryo_robot/conveyor/feedback', ConveyorFeedbackArray)

        self.__conveyor_id_to_number = {}
        for conveyor_protocol_enum in [ConveyorTTL, ConveyorCan]:
            self.__conveyor_id_to_number[conveyor_protocol_enum.ID_1.value] = ConveyorID.ID_1
            self.__conveyor_id_to_number[conveyor_protocol_enum.ID_2.value] = ConveyorID.ID_2

        # Software
        self.__software_version_ntv = NiryoTopicValue('/niryo_robot_hardware_interface/software_version',
                                                      SoftwareVersion,
                                                      queue_size=1)

        # - Action server
        # Robot action
        self.__robot_action_nac = NiryoActionClient('/niryo_robot_arm_commander/robot_action',
                                                    RobotMoveAction,
                                                    RobotMoveGoal)

        self.__follow_joint_traj_nac = NiryoActionClient(
            rospy.get_param("/niryo_robot_arm_commander/joint_controller_name") + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            FollowJointTrajectoryGoal)

        self.__tools = ToolsRosWrapper(self.__service_timeout)

        # database
        from niryo_robot_database.api import DatabaseRosWrapper
        self.__database = DatabaseRosWrapper(self.__service_timeout)

        from niryo_robot_status.api import RobotStatusRosWrapper
        self.__robot_status = RobotStatusRosWrapper(self.__service_timeout)

        if self.__hardware_version in ['ned2', 'ned3pro']:
            from niryo_robot_python_ros_wrapper.custom_button_ros_wrapper import CustomButtonRosWrapper
            from niryo_robot_led_ring.api import LedRingRosWrapper
            from niryo_robot_sound.api import SoundRosWrapper

            # Led Ring
            self.__led_ring = LedRingRosWrapper(self.__hardware_version, self.__service_timeout)
            # Sound
            self.__sound = SoundRosWrapper(self.__hardware_version, self.__service_timeout)
            # - Custom button
            self.__custom_button = CustomButtonRosWrapper(self.__hardware_version)
        else:
            self.__led_ring = self.__sound = self.__custom_button = None

        rospy.loginfo("Python ROS Wrapper ready")

    @classmethod
    def init_with_node(cls, *args, **kwargs):
        """
        Initialize a ros node before returning a new instance of the class
        """
        rospy.init_node(cls.__name__, anonymous=True)
        return cls(*args, **kwargs)

    @property
    def collision_detected(self):
        return self._collision_detected

    def clear_collision_detected(self):
        self._collision_detected = False

    def __callback_collision_detected(self, msg):
        if msg.data:
            self._collision_detected = True

    def __advertise_stop(self):
        if self.__hardware_version in ['ned', 'ned2', 'ned3pro']:
            try:
                self.__advertise_ros_wrapper_srv(self.__node_name, False)
            except (rospy.ServiceException, rospy.ROSException):
                pass

    def __ping_ros_wrapper_callback(self):
        return CommandStatus.SUCCESS, self.__node_name

    @classmethod
    def wait_for_node_initialization(cls, node_name, timeout=0, sleep_time=0.1):
        param_name = f'/{node_name}/initialized'
        t_start = rospy.Time.now()
        while not rospy.has_param(param_name):
            rospy.loginfo_once(f'Niryo ROS Wrapper - Waiting for {node_name} initialization')
            if ((timeout > 0) and (rospy.Time.now() - t_start).to_sec() > timeout):
                raise NiryoRosWrapperException(f'Timeout exceeded while waiting for node {node_name} initialization')
            rospy.sleep(sleep_time)

    @classmethod
    def wait_for_nodes_initialization(cls, simulation_mode=False):
        nodes_to_check = [
            'niryo_robot_poses_handlers',
            'niryo_robot_arm_commander',
            'niryo_robot_rpi',
            'niryo_robot_tools_commander',
        ]
        for node in nodes_to_check:
            cls.wait_for_node_initialization(node, sleep_time=1 if simulation_mode else 0.1)

    # -- Publishers
    # Blockly
    def highlight_block(self, block_id):
        msg = String()
        msg.data = block_id
        self.__highlight_block_publisher.publish(msg)

    def break_point(self):
        import sys

        msg = Int32()
        msg.data = 1
        self.__break_point_publisher.publish(msg)

        # Close program
        sys.exit()

    # -- Main Purpose
    def request_new_calibration(self):
        """
        Calls service to set the request calibration value. If failed, raises NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        try:
            return self._call_service('/niryo_robot/joints_interface/request_new_calibration', Trigger)
        except rospy.ROSException as e:
            raise NiryoRosWrapperException(str(e))

    def calibrate_auto(self):
        """
        Calls service to calibrate motors then waits for its end. If failed, raises NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        return self.__calibrate(calib_type_int=1)

    def calibrate_manual(self):
        """
        Calls service to calibrate motors then waits for its end. If failed, raises NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        return self.__calibrate(calib_type_int=2)

    def __calibrate(self, calib_type_int):
        """
        Call service to calibrate motors then waits for its end. If failed, raises NiryoRosWrapperException

        :param calib_type_int: 1 for auto-calibration & 2 for manual calibration
        :return: status, message
        :rtype: (int, str)
        """
        hw_status = self.__hw_status_ntv.wait_for_message()
        if not hw_status.calibration_needed:
            return self.return_success("Calibration not needed")

        result = self._call_service('/niryo_robot/joints_interface/calibrate_motors', SetInt, calib_type_int)
        self._check_result_status(result)
        # Wait until calibration start
        rospy.sleep(0.2)
        calibration_finished = False
        while not calibration_finished:
            try:
                hw_status = self.__hw_status_ntv.wait_for_message()
                if not (hw_status.calibration_needed or hw_status.calibration_in_progress):
                    calibration_finished = True
                else:
                    rospy.sleep(0.1)
            except rospy.ROSException as e:
                raise NiryoRosWrapperException(str(e))
        # Little delay to be sure calibration is over
        rospy.sleep(0.5)
        return result.status, result.message

    def get_learning_mode(self):
        """
        Uses /niryo_robot/learning_mode/state topic subscriber to get learning mode status

        :return: ``True`` if activate else ``False``
        :rtype: bool
        """
        return self.__learning_mode_on_ntv.value.data

    def set_learning_mode(self, set_bool):
        """
        Calsl service to set_learning_mode according to set_bool. If failed, raises NiryoRosWrapperException

        :param set_bool: ``True`` to activate, ``False`` to deactivate
        :type set_bool: bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot/learning_mode/activate', SetBool, set_bool)
        rospy.sleep(0.1)
        return self._classic_return_w_check(result)

    def get_max_velocity_scaling_factor(self):
        """
        Gets the max velocity scaling factor
        :return: max velocity scaling factor
        :rtype: Int32
        """
        return self.__max_velocity_scaling_factor_ntv.value

    def set_arm_max_velocity(self, percentage):
        """
        Sets relative max velocity (in %)

        :param percentage: Percentage of max velocity
        :type percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_arm_commander/set_max_velocity_scaling_factor', SetInt, percentage)
        return self._classic_return_w_check(result)

    def set_arm_max_acceleration(self, percentage):
        """
        Sets relative max acceleration (in %)

        :param percentage: Percentage of max acceleration
        :type percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_arm_commander/set_acceleration_factor', SetInt, percentage)
        return self._classic_return_w_check(result)

    # - Useful functions
    @staticmethod
    def wait(time_sleep):
        rospy.sleep(time_sleep)

    # -- Move

    # - Pose

    def get_joints(self):
        """
        Uses /joint_states topic to get joints status

        :return: A JointsPosition object containing the joints values
        :rtype: JointsPosition
        """
        return JointsPosition(*self.__joints_ntv.value.position)

    def get_joint_names(self):
        """
        Uses /joint_states topic to get the name of the joints

        :return: list of the name of the joints
        :rtype: list[string]
        """
        return list(self.__joints_ntv.value.name)

    def get_pose(self):
        """
        Return the legacy pose of the robot (i.e the legacy TCP orientation)
        Uses /niryo_robot/robot_state topic to get pose status

        :return: RobotState object (position.x/y/z && rpy.roll/pitch/yaw && orientation.x/y/z/w)
        :rtype: RobotState
        """
        return self.__pose_ntv.value

    def get_pose_as_list(self):
        """
        Return the legacy pose of the robot (i.e the legacy TCP orientation) as a list
        Uses /niryo_robot/robot_state topic to get pose status

        :return: list corresponding to [x, y, z, roll, pitch, yaw]
        :rtype: list[float]
        """
        p_msg = self.get_pose()
        pose = self.pose_from_msg(p_msg)
        return pose

    @overload
    def move(self, robot_position: Pose, move_cmd: int = ArmMoveCommand.POSE, blocking=True):
        ...

    @overload
    def move(self, robot_position: JointsPosition, move_cmd: int = ArmMoveCommand.JOINTS, blocking=True):
        ...

    @move_command
    def move(self, robot_position, move_cmd: int = None, blocking=True):
        """
        Moves the robot end effector to the given goal position. The goal position can be either a joint position or a
        pose

        :param robot_position: Position of the goal position
        :type robot_position: Union[Pose, JointsPosition]
        :param move_cmd: Command used to move the robot. If not provided, the command will be the basic move (either
            joint or pose depending on the robot_position type)
        :type move_cmd: int
        :param blocking: Whether the function should wait for the end of the movement
        :type blocking: bool
        :return: status, message
        :rtype: (int, str)
        """
        if move_cmd is None:
            move_cmd = ArmMoveCommand.JOINTS if isinstance(robot_position, JointsPosition) else ArmMoveCommand.POSE

        move_goal = ArmMoveCommand(cmd_type=move_cmd)
        if move_cmd == ArmMoveCommand.JOINTS:
            if not isinstance(robot_position, JointsPosition):
                raise NiryoRosWrapperException("goal must be a JointsPosition in order to do a move joints")
            move_goal.joints = list(robot_position)
        else:
            if not isinstance(robot_position, Pose):
                raise NiryoRosWrapperException("goal must be a Pose in order to do a move pose")
            robot_position.normalize()
            if robot_position.metadata.frame != '':
                robot_position = self.__calculate_transform_in_frame(robot_position)
            move_goal.position.x = robot_position.x
            move_goal.position.y = robot_position.y
            move_goal.position.z = robot_position.z
            move_goal.rpy.roll = robot_position.roll
            move_goal.rpy.pitch = robot_position.pitch
            move_goal.rpy.yaw = robot_position.yaw

        goal = RobotMoveGoal(cmd=move_goal)
        return self.__robot_action_nac.execute(goal, wait_for_result=blocking)

    @move_command
    def __move_sequence(self, sequence, blocking=True, **kwargs):
        """
        Executes a sequence of actions in a blocking or non-blocking way
        :param sequence: a function that contains the sequence of actions to execute
        :type sequence: Callable[..., (int, str)]
        :param kwargs: any keyword argument taken by the move function (except blocking)
        :return: the return value of sequence if blocking, else None
        """

        def sequence_wrapper():
            try:
                sequence(**kwargs)
            except NiryoRosWrapperException as e:
                rospy.logerr(f'An error occurred while running sequence: {e}')

        if blocking:
            return sequence(**kwargs)
        else:
            Thread(target=sequence_wrapper).start()

    @move_command
    def move_joints(self, j1, j2, j3, j4, j5, j6, **kwargs):
        """
        .. deprecated:: 5.5.0
           You should use :func:`move` with a `JointsPosition` object.

        Executes Move joints action

        :param j1:
        :type j1: float
        :param j2:
        :type j2: float
        :param j3:
        :type j3: float
        :param j4:
        :type j4: float
        :param j5:
        :type j5: float
        :param j6:
        :type j6: float
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use move with a JointsPosition object.", DeprecationWarning)
        return self.move(JointsPosition(j1, j2, j3, j4, j5, j6), **kwargs)

    @move_command
    def move_to_sleep_pose(self, **kwargs):
        """
        Moves the robot to a predefined home position

        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_joints_interface/get_home_position', GetFloatList)
        self._classic_return_w_check(result)
        joints_position = JointsPosition(*result.values)
        return self.move(joints_position, **kwargs)

    def get_sleep_pose(self) -> JointsPosition:
        """
        Get current robot's home position

        :return: the sleep pose
        :rtype: JointsPosition
        """
        result = self._call_service('/niryo_robot_joints_interface/get_home_position', GetFloatList)
        self._classic_return_w_check(result)
        joints_position = JointsPosition(*result.values)
        return joints_position

    def set_sleep_pose(self, joints_position: JointsPosition):
        """
        Set user defined robot's home position in the database.
        Raises NiryoRosWrapperException if the number of joint values does not match the robot's joints number
        or if failed.

        :param joints_position: joint position of the sleep pose
        :type joints_position: JointsPosition

        :return: status, message
        :rtype: (int, str)
        """
        if len(joints_position) != 6:
            raise NiryoRosWrapperException(f'Pose requires 6 values. Currently {len(joints_position)} given')
        result = self._call_service('/niryo_robot_joints_interface/set_home_position',
                                    SetFloatList,
                                    list(joints_position))
        return self._classic_return_w_check(result)

    def reset_sleep_pose(self):
        """
        Reset robot's home position to factory default

        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_joints_interface/reset_home_position', Trigger)
        return self._classic_return_w_check(result)

    @move_command
    def move_pose(self, x, y, z, roll, pitch, yaw, frame='', **kwargs):
        """
        .. deprecated:: 5.5.0
           You should use :func:`move` with a `Pose` object.

        Moves robot end effector pose to a (x, y, z, roll, pitch, yaw) pose,
        in a particular frame if defined

        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param roll:
        :type roll: float
        :param pitch:
        :type pitch: float
        :param yaw:
        :type yaw: float
        :param frame:
        :type frame: str
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use move with a Pose object.", DeprecationWarning)
        return self.move(Pose(x, y, z, roll, pitch, yaw, PoseMetadata(frame=frame)), **kwargs)

    def move_circle(self, x, y, z, **kwargs):
        """
        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return:
        """
        return self.move(Pose(x, y, z, 0, 0, 0), ArmMoveCommand.DRAW_CIRCLE, **kwargs)

    def move_pose_saved(self, pose_name, **kwargs):
        """
        Moves robot end effector pose to a pose saved

        :param pose_name:
        :type pose_name: str
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        return self.move(self.get_pose_saved(pose_name), **kwargs)

    @move_command
    def shift_pose(self, axis, value, linear=False):
        """
        Executes Shift pose action

        :param axis: Value of RobotAxis enum corresponding to where the shift happens
        :type axis: ShiftPose
        :param value: shift value
        :type value: float
        :param linear: Whether the movement has to be linear or not
        :type linear: bool
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.msg import ShiftPose
        move_cmd = ArmMoveCommand.SHIFT_LINEAR_POSE if linear else ArmMoveCommand.SHIFT_POSE
        cmd = ArmMoveCommand(cmd_type=move_cmd, shift=ShiftPose(axis_number=axis, value=value))
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)

    @move_command
    def shift_linear_pose(self, axis, value):
        """
        .. deprecated:: 5.5.0
           You should use :func:`shift_pose` with linear=True.

        Executes Shift pose action with a linear trajectory

        :param axis: Value of RobotAxis enum corresponding to where the shift happens
        :type axis: ShiftPose
        :param value: shift value
        :type value: float
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use shift_linear with linear=True.", DeprecationWarning)
        return self.shift_pose(axis, value, linear=True)

    @move_command
    def move_linear_pose(self, x, y, z, roll, pitch, yaw, frame='', **kwargs):
        """
        .. deprecated:: 5.5.0
           You should use :func:`move` with a `Pose` object and `move_cmd=ArmMoveCommand.LINEAR_POSE`

        Moves robot end effector pose to a (x, y, z, roll, pitch, yaw) pose, with a linear trajectory,
        in a particular frame if defined

        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param roll:
        :type roll: float
        :param pitch:
        :type pitch: float
        :param yaw:
        :type yaw: float
        :param frame:
        :type frame: str
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use move with a Pose object and move_cmd=ArmMoveCommand.LINEAR_POSE",
                      DeprecationWarning)
        pose = Pose(x, y, z, roll, pitch, yaw, metadata=PoseMetadata(frame=frame))
        return self.move(pose, move_cmd=ArmMoveCommand.LINEAR_POSE, **kwargs)

    @move_command
    def move_spiral(self, radius=0.2, angle_step=5, nb_steps=72, plan=1):
        """
        Calls robot action service to draw a spiral trajectory

        :param radius: maximum distance between the spiral and the starting point
        :param angle_step: rotation between each waypoint creation
        :param nb_steps: number of waypoints from the beginning to the end of the spiral
        :param plan: xyz plan of the spiral:  1 = yz plan, 2 = xz plan, 3 = xy plan
        :type plan: int
        :return: status, message
        :rtype: (int, str)
        """
        cmd = ArmMoveCommand(cmd_type=ArmMoveCommand.DRAW_SPIRAL, args=[radius, angle_step, nb_steps, plan])
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)

    def move_without_moveit(self, joints_target, duration):
        goal = self._create_goal(joints_target, duration)
        self.__follow_joint_traj_nac.action_server.wait_for_server()

        # When to start the trajectory: 0.1s from now
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.1)
        self.__follow_joint_traj_nac.action_server.send_goal(goal)
        self.__follow_joint_traj_nac.action_server.wait_for_result(timeout=rospy.Duration(2 * duration + 0.1))

        result = self.__follow_joint_traj_nac.action_server.get_result()
        if not result:
            raise NiryoRosWrapperException("Follow joint trajectory goal has reached timeout limit")

        msg_dict = {
            result.SUCCESSFUL: "Successful",
            result.INVALID_GOAL: "Invalid goal",
            result.INVALID_JOINTS: "Invalid joints",
            result.OLD_HEADER_TIMESTAMP: "Old header timestamp",
            result.PATH_TOLERANCE_VIOLATED: "Path tolerance violated",
            result.GOAL_TOLERANCE_VIOLATED: "Goal tolerance violated"
        }

        return result.error_code, msg_dict[result.error_code]

    def _create_goal(self, joints_position, duration):
        trajectory_point = JointTrajectoryPoint(positions=joints_position,
                                                velocities=[0.0] * len(joints_position),
                                                time_from_start=rospy.Duration(duration))

        trajectory = JointTrajectory(joint_names=self.get_joint_names(), points=[trajectory_point])
        goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        return goal

    def stop_move(self):
        """
        Stops the robot movement

        :return: list of joints value
        :rtype: list[float]
        """
        result = self._call_service('/niryo_robot_arm_commander/stop_command', Trigger)
        return self._classic_return_w_check(result)

    def set_jog_use_state(self, state):
        """
        Turns jog controller On or Off

        :param state: ``True`` to turn on, else ``False``
        :type state: bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot/jog_interface/enable', SetBool, state)
        return self._classic_return_w_check(result)

    def jog_joints_shift(self, shift_values):
        """
        .. deprecated:: 5.5.0
           You should use :func:`jog_shift` with a `JointsPosition` object.

        Makes a Jog on joints position

        :param shift_values: list corresponding to the shift to be applied to each joint
        :type shift_values: list[float]
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use jog_shift with a JointsPosition object.", DeprecationWarning)
        return self.jog_shift(JointsPosition(*shift_values))

    def jog_pose_shift(self, shift_values):
        """
        .. deprecated:: 5.5.0
           You should use :func:`jog_shift` with a `Pose` object.

        Makes a Jog on end-effector position

        :param shift_values: list corresponding to the shift to be applied to the position
        :type shift_values: list[float]
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use jog_shift with a Pose object.", DeprecationWarning)
        return self.jog_shift(Pose(*shift_values))

    def jog_shift(self, shift_values: RobotPosition):
        """
        Makes a Jog oof the robot position

        :param shift_values: robot position corresponding to the shift to be applied to the current position
        :type shift_values: RobotPosition
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import JogShiftRequest, JogShift

        if isinstance(shift_values, JointsPosition):
            shift_request = JogShiftRequest.JOINTS_SHIFT
        else:
            shift_request = JogShiftRequest.POSE_SHIFT
            shift_values.normalize()
        result = self._call_service('/niryo_robot/jog_interface/jog_shift_commander',
                                    JogShift,
                                    shift_request,
                                    list(shift_values))
        return self._classic_return_w_check(result)

    @staticmethod
    def __get_obj_from_args(obj_class: type, kw: str, args: tuple, kwargs: dict) -> RobotPosition:
        """
        Helper function for the overloaded methods. Its goal is to return the desired object no matter the input.

        Example: ::

            __get_obj_from_args(Pose, 'pose', (1, 2, 3, 4, 5, 6), {}) -> Pose(1, 2, 3, 4, 5, 6)
            __get_obj_from_args(Pose, 'pose', (), {'pose': Pose(1, 2, 3, 4, 5, 6)}) -> Pose(1, 2, 3, 4, 5, 6)
            __get_obj_from_args(Pose, 'pose', (Pose(1, 2, 3, 4, 5, 6)), {}) -> Pose(1, 2, 3, 4, 5, 6)

        :param obj_class: class of the object to retrieve
        :type obj_class: type
        :param kw: keyword to search for in kwargs
        :type kw: str
        :param args: the positional arguments
        :type args: tuple
        :param kwargs: the keyword arguments
        :type kwargs: dict
        :return: the object found in the arguments
        :rtype: object
        """
        if len(args) + len(kwargs) == 6:
            obj = obj_class(*args, **kwargs)
        elif kw in kwargs and isinstance(kwargs[kw], obj_class):
            obj = kwargs[kw]
        elif len(args) > 0 and isinstance(args[0], obj_class):
            obj = args[0]
        else:
            raise NiryoRosWrapperException(
                f'Invalid arguments. '
                f'Parameters must be either 6 floats or a {obj_class.__name__} instance. Got: {args}, {kwargs}')
        return obj

    @overload
    def forward_kinematics(self, j1: float, j2: float, j3: float, j4: float, j5: float, j6: float) -> Pose:
        ...

    @overload
    def forward_kinematics(self, joints_position: JointsPosition) -> Pose:
        ...

    def forward_kinematics(self, *args, **kwargs) -> Pose:
        """
        Computes the forward kinematics given joint positions.
        The end effector pose is given for an end effector frame following the right hand rule and with the x axis
        pointing forward.

        This function is overloaded to accept multiple forms of input:

        :param args: Variable-length positional arguments. This can be either individual joint angles
                     (j1, j2, j3, j4, j5, j6) or a single JointsPosition object.
        :type args: tuple
        :param kwargs: Arbitrary keyword arguments.
        :type kwargs: dict

        :returns: The pose of the end effector in the robot's workspace.
        :rtype: Pose
        """
        from niryo_robot_arm_commander.srv import GetFK

        joints_position = self.__get_obj_from_args(JointsPosition, 'joints_position', args, kwargs)
        joints = list(joints_position)
        result = self._call_service('/niryo_robot/kinematics/forward', GetFK, joints)
        pose = self.pose_from_msg(result.pose)
        return pose

    @overload
    def inverse_kinematics(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> JointsPosition:
        ...

    @overload
    def inverse_kinematics(self, pose: Pose) -> JointsPosition:
        ...

    def inverse_kinematics(self, *args, **kwargs) -> JointsPosition:
        """
        Computes the inverse kinematics given a pose.

        This function is overloaded to accept multiple forms of input:

        :param args: Variable-length positional arguments. This can be either individual pose components
                     (x, y, z, roll, pitch, yaw) or a single Pose object.
        :type args: tuple
        :param kwargs: Arbitrary keyword arguments.
        :type kwargs: dict

        :returns: The joint position of the robot.
        :rtype: JointsPosition
        """
        from niryo_robot_arm_commander.srv import GetIK

        pose = self.__get_obj_from_args(Pose, 'pose', args, kwargs)

        result = self._call_service('/niryo_robot/kinematics/inverse', GetIK, self.msg_from_pose(pose, normalize=False))
        self._check_result_status(result)

        return JointsPosition(*result.joints)

    # - Saved Pose

    def get_pose_saved(self, pose_name: str) -> Pose:
        """
        Gets saved pose from robot intern storage
        Will raise error if position does not exist

        :param pose_name: Pose Name
        :type pose_name: str
        :return: x, y, z, roll, pitch, yaw
        :rtype: Pose
        """
        from niryo_robot_poses_handlers.srv import GetPose

        result = self._call_service('/niryo_robot_poses_handlers/get_pose', GetPose, pose_name)
        self._check_result_status(result)
        pose = self.pose_from_msg(result.pose)
        return pose

    @overload
    def save_pose(self, name: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float):
        ...

    @overload
    def save_pose(self, name: str, pose: Pose):
        ...

    def save_pose(self, *args, **kwargs):
        """
        Saves pose in robot's memory
        """
        from niryo_robot_poses_handlers.srv import ManagePose, ManagePoseRequest

        if 'name' in kwargs:
            name = kwargs.pop('name')
        else:
            name = args[0]
            args = args[1:]

        pose = self.__get_obj_from_args(Pose, 'pose', args, kwargs)

        if len(args) + len(kwargs) > 2:
            pose.metadata = PoseMetadata(frame=pose.metadata.frame)
        pose.normalize()

        req = ManagePoseRequest()
        req.cmd = ManagePoseRequest.SAVE
        req.pose = self.msg_from_pose(pose, NiryoPose)
        req.pose.name = name
        req.pose.pose_version = pose.metadata.version

        result = self._call_service('/niryo_robot_poses_handlers/manage_pose', ManagePose, req)
        return self._classic_return_w_check(result)

    def delete_pose(self, name):
        """
        Sends delete command to the pose manager service

        :param name:
        :type name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManagePose, ManagePoseRequest

        req = ManagePoseRequest()
        req.cmd = ManagePoseRequest.DELETE
        req.pose.name = name
        result = self._call_service('/niryo_robot_poses_handlers/manage_pose', ManagePose, req)
        return self._classic_return_w_check(result)

    @staticmethod
    def get_saved_pose_list(with_desc=False):
        """
        Asks the pose manager service which positions are available

        :param with_desc: If True it returns the poses descriptions
        :type with_desc: bool
        :return: list of positions name
        :rtype: list[str]
        """
        pose_list = rospy.wait_for_message('/niryo_robot_poses_handlers/pose_list', BasicObjectArray, 2)
        names = [pose.name for pose in pose_list.objects]
        if with_desc:
            descriptions = [pose.description for pose in pose_list.objects]
            return names, descriptions
        return names

    # - Pick/Place

    def __get_high_pose(self, robot_position: RobotPosition, z_offset=0.05):
        if isinstance(robot_position, JointsPosition):
            high_pick_pose = self.forward_kinematics(*robot_position)
        else:
            high_pick_pose = robot_position.copy()
        high_pick_pose.z += z_offset
        return high_pick_pose

    def pick(self, robot_position: RobotPosition, **kwargs):
        """
        Picks an object at a given position.
        :param robot_position: The position of the object to pick
        :type robot_position: RobotPosition
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return:
        """

        high_pick_pose = self.__get_high_pose(robot_position)

        def move_sequence(**kwargs_):
            self.release_with_tool()
            self.execute_trajectory([high_pick_pose, robot_position])
            self.grasp_with_tool()
            return self.move(high_pick_pose, **kwargs_)

        return self.__move_sequence(move_sequence, **kwargs)

    def place(self, robot_position: RobotPosition, **kwargs):
        """
        Places an object at a given position.
        :param robot_position: The position of the object to place
        :type robot_position: RobotPosition
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        """
        high_pick_pose = self.__get_high_pose(robot_position)

        def move_sequence(**kwargs_):
            self.execute_trajectory([high_pick_pose, robot_position])
            self.release_with_tool()
            self.move(high_pick_pose, **kwargs_)

        return self.__move_sequence(move_sequence, **kwargs)

    def pick_from_pose(self, x, y, z, roll, pitch, yaw, **kwargs):
        """
        .. deprecated:: 5.5.0
           You should use :func:`pick` with a `Pose` or `JointsPosition` object instead

        Executes a picking from a position. If an error happens during the movement, error will be raised
        A picking is described as :
        - going over the object
        - going down until height = z
        - grasping with tool
        - going back over the object

        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param roll:
        :type roll: float
        :param pitch:
        :type pitch: float
        :param yaw:
        :type yaw: float
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
       """
        warnings.warn("You should use pick with a Pose or JointsPosition object instead", DeprecationWarning)

        def move_sequence(**kwargs_):
            self.release_with_tool()

            self.move(Pose(x, y, z + 0.05, roll, pitch, yaw), **kwargs_)
            self.move(Pose(x, y, z, roll, pitch, yaw), **kwargs_)

            self.grasp_with_tool()

            self.move(Pose(x, y, z + 0.05, roll, pitch, yaw), **kwargs_)

        return self.__move_sequence(move_sequence, **kwargs)

    def place_from_pose(self, x, y, z, roll, pitch, yaw, **kwargs):
        """
        .. deprecated:: 5.5.0
           You should use :func:`place` with a `Pose` or `JointsPosition` object instead

        Executes a placing from a position. If an error happens during the movement, error will be raised
        A placing is described as :
        - going over the place
        - going down until height = z
        - releasing the object with tool
        - going back over the place

        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param roll:
        :type roll: float
        :param pitch:
        :type pitch: float
        :param yaw:
        :type yaw: float
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use place with a Pose or JointsPosition object instead", DeprecationWarning)

        def move_sequence(**kwargs_):
            self.move(Pose(x, y, z + 0.05, roll, pitch, yaw), **kwargs_)
            self.move(Pose(x, y, z, roll, pitch, yaw), **kwargs_)

            self.release_with_tool()

            self.move(Pose(x, y, z + 0.05, roll, pitch, yaw), **kwargs_)

        return self.__move_sequence(move_sequence, **kwargs)

    @overload
    def pick_and_place(self, pick_pose: list, place_pose: list, dist_smoothing=0.0, **kwargs):
        ...

    @overload
    def pick_and_place(self, pick_pose: RobotPosition, place_pose: RobotPosition, dist_smoothing=0.0, **kwargs):
        ...

    def pick_and_place(self, pick_pose, place_pose, dist_smoothing=0.0, **kwargs):
        """
        Executes a pick and place. If an error happens during the movement, error will be raised
        -> Args param is for development purposes

        :param pick_pose: The position of the object to pick
        :type pick_pose: list[float] | RobotPosition
        :param place_pose: The position of the object to place
        :type place_pose: list[float] | RobotPosition
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        if isinstance(pick_pose, list):
            pick_pose = Pose(*pick_pose)
        if isinstance(place_pose, list):
            place_pose = Pose(*place_pose)

        pick_pose_high = self.__get_high_pose(pick_pose)
        place_pose_high = self.__get_high_pose(place_pose)

        def move_sequence(**kwargs_):
            self.release_with_tool()
            self.execute_trajectory([pick_pose_high, pick_pose], dist_smoothing=dist_smoothing)
            self.grasp_with_tool()
            self.execute_trajectory([pick_pose_high, place_pose_high, place_pose], dist_smoothing=dist_smoothing)
            self.release_with_tool()
            self.move(place_pose_high, **kwargs_)

        return self.__move_sequence(move_sequence, **kwargs)

    # - Trajectories

    def __ros_poses_from_robot_positions(self, robot_positions: List[RobotPosition]) -> List[RosPose]:
        """
        Converts a list of robot positions to a list of ROS poses. Also convert the poses to DH conventions if needed.
        :param robot_positions: a list of objects
        :type robot_positions: list[RobotPosition]
        :return: a list of ROS poses
        :rtype: list[RosPose]
        """
        list_poses = []
        for pose in robot_positions:
            if isinstance(pose, JointsPosition):
                pose = self.forward_kinematics(pose)
            pose.normalize()

            point = Point(x=pose.x, y=pose.y, z=pose.z)
            quaternion = pose.quaternion()
            orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
            list_poses.append(RosPose(position=point, orientation=orientation))
        return list_poses

    @staticmethod
    def __robot_positions_from_raw_poses(raw_poses: List[List[float]], list_type=None) -> List[RobotPosition]:
        if list_type is None or len(list_type) == 1 and list_type[0] == 'pose':
            list_type = ['pose'] * len(raw_poses)
        elif len(list_type) == 1 and list_type[0] == 'joints':
            list_type = ['joints'] * len(raw_poses)

        robot_position = []
        for raw_pose, pose_type in zip(raw_poses, list_type):
            if pose_type == 'pose':
                if len(raw_pose) == 7:  # quaternion
                    raw_pose = raw_pose[:3] + list(euler_from_quaternion(raw_pose[3:]))
                robot_position.append(Pose(*raw_pose))
            elif pose_type == 'joints':
                robot_position.append(JointsPosition(*raw_pose))
            else:
                raise NiryoRosWrapperException(f'Invalid type "{pose_type}". Only "pose" and "joints" are supported')
        return robot_position

    #   - Compute

    def compute_trajectory(self, robot_positions: List[RobotPosition], dist_smoothing=0.0):
        """
        Generate a trajectory from a list of poses and joints.
        :param robot_positions: a list of poses and / or joints
        :type robot_positions: list[RobotPosition]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float

        :return: a trajectory
        :rtype: JointTrajectory
        """
        from niryo_robot_arm_commander.srv import ComputeTrajectory

        if len(robot_positions) < 2:
            return "Give me at least 2 points"
        list_poses = self.__ros_poses_from_robot_positions(robot_positions)

        result = self._call_service("/niryo_robot_arm_commander/compute_waypointed_trajectory",
                                    ComputeTrajectory,
                                    list_poses,
                                    dist_smoothing)
        self._classic_return_w_check(result)
        return result.trajectory

    def compute_trajectory_from_poses_and_joints(self, list_pose_joints, list_type=None, dist_smoothing=0.0):
        """
        .. deprecated:: 5.5.0
           You should use :func:`compute_trajectory` with `JointsPosition` and `Pose` objects.

        Generate a trajectory from a list of poses and joints.
        :param list_pose_joints: a list of poses and / or joints
        :type list_pose_joints: list[list[float]]
        :param list_type: a list indicating if the corresponding element is a pose or joints
        :type list_type: list[str]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float

        :return: a trajectory
        :rtype: JointTrajectory
        """
        warnings.warn("You should use compute_trajectory with JointsPosition and Pose objects.", DeprecationWarning)
        robot_positions = self.__robot_positions_from_raw_poses(list_pose_joints, list_type)
        return self.compute_trajectory(robot_positions, dist_smoothing)

    def compute_trajectory_from_poses(self, list_poses_raw, dist_smoothing=0.0):
        """
        .. deprecated:: 5.5.0
           You should use :func:`compute_trajectory` with `Pose` objects.

        Generate a trajectory from a list of robot positions.
        :param list_poses_raw: a list of poses
        :type list_poses_raw: list[list[float]]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float

        :return: a trajectory
        :rtype: JointTrajectory
        """
        warnings.warn("You should use compute_trajectory with Pose objects.", DeprecationWarning)
        robot_positions = self.__robot_positions_from_raw_poses(list_poses_raw, list_type=['pose'])
        return self.compute_trajectory(robot_positions, dist_smoothing)

    #   - Execute

    @move_command
    def execute_trajectory(self, robot_positions: List[RobotPosition], dist_smoothing=0.0):
        """
        Executes trajectory from list of poses and joints

        :param robot_positions: List of poses and / or joints
        :type robot_positions: list[RobotPosition]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float
        :return: status, message
        :rtype: (int, str)
        """
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.EXECUTE_TRAJ
        goal.cmd.list_poses = self.__ros_poses_from_robot_positions(robot_positions)
        goal.cmd.dist_smoothing = dist_smoothing
        return self.__robot_action_nac.execute(goal)

    def execute_trajectory_from_poses_and_joints(self, list_pose_joints, list_type=None, dist_smoothing=0.0):
        """
        .. deprecated:: 5.5.0
           You should use :func:`execute_trajectory` with `JointsPosition` and `Pose` objects.

        Executes trajectory from list of poses and joints

        :param list_pose_joints: List of [x,y,z,qx,qy,qz,qw]
            or list of [x,y,z,roll,pitch,yaw] or a list of [j1,j2,j3,j4,j5,j6]
        :type list_pose_joints: list[list[float]]
        :param list_type: List of string 'pose' or 'joint', or ['pose'] (if poses only) or ['joint'] (if joints only).
            If None, it is assumed there are only poses in the list.
        :type list_type: list[string]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use execute_trajectory with JointsPosition and Pose objects.", DeprecationWarning)
        robot_positions = self.__robot_positions_from_raw_poses(list_pose_joints, list_type)
        return self.execute_trajectory(robot_positions, dist_smoothing)

    def execute_trajectory_from_poses(self, list_poses_raw: List[Union[List[float], Pose]], dist_smoothing=0.0):
        """
        .. deprecated:: 5.5.0
           You should use :func:`execute_trajectory` with `Pose` objects.

        Executes trajectory from a list of pose

        :param list_poses_raw: list of [x, y, z, qx, qy, qz, qw] or list of [x, y, z, roll, pitch, yaw]
        :type list_poses_raw: List[Union[List[float], Pose]]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use execute_trajectory with Pose objects.", DeprecationWarning)
        robot_positions = self.__robot_positions_from_raw_poses(list_poses_raw, list_type=['pose'])
        return self.execute_trajectory(robot_positions, dist_smoothing)

    @move_command
    def execute_registered_trajectory(self, trajectory_name: str):
        """
        Sends execution command to the trajectory manager service

        :param trajectory_name: name
        :type trajectory_name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest
        req = ManageTrajectoryRequest(cmd=ManageTrajectoryRequest.EXECUTE_REGISTERED, name=trajectory_name)
        result = self._call_service('/niryo_robot_arm_commander/manage_trajectory', ManageTrajectory, req)
        return self._classic_return_w_check(result)

    @move_command
    def execute_moveit_robot_trajectory(self, moveit_robot_trajectory):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.EXECUTE_FULL_TRAJ
        goal.cmd.trajectory = moveit_robot_trajectory
        return self.__robot_action_nac.execute(goal)

    #   - Other

    def get_trajectory_saved(self, trajectory_name) -> List[JointsPosition]:
        """
        Gets saved trajectory from robot intern storage
        Will raise error if position does not exist

        :param trajectory_name:
        :type trajectory_name: str
        :raises NiryoRosWrapperException: If trajectory file doesn't exist
        :return: list of [x, y, z, qx, qy, qz, qw]
        :rtype: list[list[float]]
        """
        from niryo_robot_arm_commander.srv import GetTrajectory

        result = self._call_service('/niryo_robot_arm_commander/get_trajectory', GetTrajectory, trajectory_name)
        self._check_result_status(result)
        return [JointsPosition(*t.positions) for t in result.trajectory.points]

    @staticmethod
    def get_saved_trajectory_list() -> List[str]:
        """
        Asks the pose trajectory service which trajectories are available

        :return: list of trajectory name
        :rtype: list[str]
        """
        trajectories = rospy.wait_for_message('/niryo_robot_arm_commander/trajectory_list', BasicObjectArray, 2)
        return [trajectory.name for trajectory in trajectories.objects]

    def save_trajectory(self, trajectory_points, trajectory_name, trajectory_description):
        """
        Saves trajectory object and sends it to the trajectory manager service

        :param trajectory_points: list of waypoints that constitute the trajectory
        :type trajectory_points: Union[List[float], JointsPosition]
        :param trajectory_name: name which will have the trajectory
        :type trajectory_name: str
        :param trajectory_description: A short text describing the trajectory
        :type trajectory_description: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest
        from std_msgs.msg import Header
        req = ManageTrajectoryRequest()
        req.cmd = ManageTrajectoryRequest.SAVE
        req.name = trajectory_name

        req.trajectory = JointTrajectory(
            header=Header(stamp=rospy.Time.now()),
            joint_names=self.get_joint_names(),
            points=[JointTrajectoryPoint(positions=list(trajectory_point)) for trajectory_point in trajectory_points])

        req.description = trajectory_description
        result = self._call_service('/niryo_robot_arm_commander/manage_trajectory', ManageTrajectory, req)
        return self._classic_return_w_check(result)

    def save_last_learned_trajectory(self, trajectory_name, trajectory_description):
        """
        Saves trajectory object and sends it to the trajectory manager service

        :param trajectory_name: name which will have the trajectory
        :type trajectory_name: str
        :param trajectory_description: description which will have the trajectory
        :type trajectory_description: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest
        req = ManageTrajectoryRequest()
        req.cmd = ManageTrajectoryRequest.SAVE_LAST_LEARNED
        req.name = trajectory_name
        req.description = trajectory_description
        result = self._call_service('/niryo_robot_arm_commander/manage_trajectory', ManageTrajectory, req)
        return self._classic_return_w_check(result)

    def update_trajectory_infos(self, name, new_name, new_description):
        from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest
        req = ManageTrajectoryRequest(cmd=ManageTrajectoryRequest.UPDATE,
                                      name=name,
                                      new_name=new_name,
                                      description=new_description)
        result = self._call_service('/niryo_robot_arm_commander/manage_trajectory', ManageTrajectory, req)
        return self._classic_return_w_check(result)

    def delete_trajectory(self, trajectory_name):
        """
        Sends delete command to the trajectory manager service

        :param trajectory_name: name
        :type trajectory_name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest
        req = ManageTrajectoryRequest(cmd=ManageTrajectoryRequest.DELETE, name=trajectory_name)
        result = self._call_service('/niryo_robot_arm_commander/manage_trajectory', ManageTrajectory, req)
        return self._classic_return_w_check(result)

    def clean_trajectory_memory(self):
        """
        Sends delete all trajectory command to the trajectory manager service

        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest
        req = ManageTrajectoryRequest(cmd=ManageTrajectoryRequest.DELETE_ALL)
        result = self._call_service('/niryo_robot_arm_commander/manage_trajectory', ManageTrajectory, req)
        return self._classic_return_w_check(result)

    # - Dynamic frame

    def save_dynamic_frame_from_poses(self,
                                      frame_name,
                                      description,
                                      list_robot_poses: List[Union[Pose, List[float]]],
                                      belong_to_workspace=False):
        """
        Create a dynamic frame with 3 poses (origin, x, y)

        :param frame_name: name of the frame
        :type frame_name: str
        :param description: description of the frame
        :type description: str
        :param list_robot_poses: 3 poses needed to create the frame
        :type list_robot_poses: list[list[float]]
        :param belong_to_workspace: indicate if the frame belong to a workspace
        :type belong_to_workspace: boolean
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageDynamicFrame, ManageDynamicFrameRequest

        for ix, pose in enumerate(list_robot_poses):
            if isinstance(pose, list):
                list_robot_poses[ix] = Pose(*pose)
            list_robot_poses[ix].normalize()

        req = ManageDynamicFrameRequest()
        req.cmd = ManageDynamicFrameRequest.SAVE
        req.dynamic_frame.name = frame_name
        req.dynamic_frame.description = description
        req.dynamic_frame.poses = [self.msg_from_pose(pose) for pose in list_robot_poses]
        req.dynamic_frame.belong_to_workspace = belong_to_workspace

        result = self._call_service('/niryo_robot_poses_handlers/manage_dynamic_frame', ManageDynamicFrame, req)

        self._check_result_status(result)

        return self._classic_return_w_check(result)

    def save_dynamic_frame_from_points(self, frame_name, description, list_points, belong_to_workspace=False):
        """
        Create a dynamic frame with 3 points (origin, x, y)

        :param frame_name: name of the frame
        :type frame_name: str
        :param description: description of the frame
        :type description: str
        :param list_points: 3 points needed to create the frame
        :type list_points: list[list[float]]
        :param belong_to_workspace: indicate if the frame belong to a workspace
        :type belong_to_workspace: boolean
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageDynamicFrame, ManageDynamicFrameRequest

        points = [Point(*point) for point in list_points]

        req = ManageDynamicFrameRequest()
        req.cmd = ManageDynamicFrameRequest.SAVE_WITH_POINTS
        req.dynamic_frame.name = frame_name
        req.dynamic_frame.description = description
        req.dynamic_frame.points = points
        req.dynamic_frame.belong_to_workspace = belong_to_workspace

        result = self._call_service('/niryo_robot_poses_handlers/manage_dynamic_frame', ManageDynamicFrame, req)

        self._check_result_status(result)

        return self._classic_return_w_check(result)

    def edit_dynamic_frame(self, frame_name, new_frame_name, new_description):
        """
        Modify a dynamic frame

        :param frame_name: name of the frame
        :type frame_name: str
        :param new_frame_name: new name of the frame
        :type new_frame_name: str
        :param new_description: new description of the frame
        :type new_description: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageDynamicFrame, ManageDynamicFrameRequest

        req = ManageDynamicFrameRequest()
        req.cmd = ManageDynamicFrameRequest.EDIT
        req.dynamic_frame.name = frame_name
        req.dynamic_frame.new_name = new_frame_name
        req.dynamic_frame.description = new_description

        result = self._call_service('/niryo_robot_poses_handlers/manage_dynamic_frame', ManageDynamicFrame, req)

        self._check_result_status(result)

        return self._classic_return_w_check(result)

    def delete_dynamic_frame(self, frame_name, belong_to_workspace=False):
        """
        Delete a dynamic frame

        :param frame_name: name of the frame to remove
        :type frame_name: str
        :param belong_to_workspace: indicate if the frame belong to a workspace
        :type belong_to_workspace: boolean
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageDynamicFrame, ManageDynamicFrameRequest

        req = ManageDynamicFrameRequest()
        req.cmd = ManageDynamicFrameRequest.DELETE
        req.dynamic_frame.name = frame_name
        req.dynamic_frame.belong_to_workspace = belong_to_workspace

        result = self._call_service('/niryo_robot_poses_handlers/manage_dynamic_frame', ManageDynamicFrame, req)

        self._check_result_status(result)

        return self._classic_return_w_check(result)

    def get_saved_dynamic_frame(self, frame_name):
        """
        Get name, description and pose of a dynamic frame

        :param frame_name: name of the frame
        :type frame_name: str
        :return: name, description, position and orientation of a frame
        :rtype: list[str, str, list[float]]
        """
        from niryo_robot_poses_handlers.srv import GetDynamicFrame

        result = self._call_service('/niryo_robot_poses_handlers/get_dynamic_frame', GetDynamicFrame, frame_name)

        self._check_result_status(result)

        name = result.dynamic_frame.name
        description = result.dynamic_frame.description

        position = result.dynamic_frame.position
        rpy = result.dynamic_frame.rpy

        return [name, description, [position.x, position.y, position.z, rpy.roll, rpy.pitch, rpy.yaw]]

    @staticmethod
    def get_saved_dynamic_frame_list():
        """
        Get list of saved dynamic frames

        :return: list of dynamic frames name, list of description of dynamic frames
        :rtype: list[str], list[str]
        """
        dynamic_frame_list = rospy.wait_for_message('/niryo_robot_poses_handlers/dynamic_frame_list',
                                                    BasicObjectArray,
                                                    2)
        names = [dynamic_frame.name for dynamic_frame in dynamic_frame_list.objects]
        descriptions = [dynamic_frame.description for dynamic_frame in dynamic_frame_list.objects]
        return names, descriptions

    def __transform_pose(self, pose_local_frame, local_frame, source_frame):
        """
        Transform pose from a local frame to source frame

        :param pose_local_frame: pose in local frame
        :type pose_local_frame: geometry_msgs/Pose
        :param local_frame: name of local frame
        :type local_frame: str
        :param source_frame: name of local frame
        :type source_frame: str
        :return: transform_pose pose in frame source
        :rtype: geometry_msgs/PoseStamped
        """
        import tf2_ros
        import tf2_geometry_msgs

        if not hasattr(self, 'tf_listener') or not hasattr(self, 'tf_buffer'):
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get transform
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                local_frame,
                rospy.Time(0),
                rospy.Duration(4.0),
            )
        except rospy.ROSException as e:
            raise NiryoRosWrapperException(str(e))

        # Pose in local_frame
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = pose_local_frame
        pose_stamped.header.frame_id = local_frame
        pose_stamped.header.stamp = rospy.Time.now()

        # Calculate pose in world frame
        transform_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        return transform_pose

    def __calculate_transform_in_frame(self, pose: Pose):
        """
        Calculate the pose (x,y,z,roll,pitch,yaw) in the frame (frame_name)

        :param pose: a pose
        :type pose : Pose
        :return: the transformed frame
        :rtype: Pose
        """

        ros_pose = RosPose()
        ros_pose.position = Point(pose.x, pose.y, pose.z)
        quaternion = quaternion_from_euler(pose.roll, pose.pitch, pose.yaw)
        ros_pose.orientation = Quaternion(*quaternion)

        world_pose = self.__transform_pose(ros_pose, pose.metadata.frame, "world")

        point = world_pose.pose.position
        quaternion = world_pose.pose.orientation
        euler = euler_from_quaternion(self.quaternion_to_list(quaternion))

        return Pose(point.x, point.y, point.z, euler[0], euler[1], euler[2])

    @move_command
    def move_relative(self, offset, frame="world", **kwargs):
        """
        Move the end effector of the robot by an offset in a frame

        :param offset: list which contains offset of x, y, z, roll, pitch, yaw
        :type offset: list[float]
        :param frame: name of local frame
        :type frame: str
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        if len(offset) != 6:
            raise NiryoRosWrapperException("Offset must be a list of 6 elements")

        local_pose = self.pose_from_msg(self.__transform_pose(self.get_pose(), 'world', frame))
        for ix, o in enumerate(offset):
            local_pose[ix] += o

        return self.move(local_pose, **kwargs)

    @move_command
    def move_linear_relative(self, offset, frame="world", **kwargs):
        """
        .. deprecated:: 5.5.0
           You should use move with a frame in the pose metadata and linear=True.

        Move robot end of an offset by a linear movement in a frame

        :param offset: list which contains offset of x, y, z, roll, pitch, yaw
        :type offset: list[float]
        :param frame: name of local frame
        :type frame: str
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: status, message
        :rtype: (int, str)
        """
        warnings.warn("You should use move with a frame in the pose metadata and linear=True.", DeprecationWarning)
        pose = Pose(*offset, metadata=PoseMetadata(frame=frame))
        return self.move(pose, move_cmd=ArmMoveCommand.LINEAR_POSE, **kwargs)

    # - Useful Pose functions

    @staticmethod
    def quaternion_to_list(quaternion):
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

    @staticmethod
    def point_to_list(point):
        return [point.x, point.y, point.z]

    @staticmethod
    def msg_from_pose(pose: Pose, msg_type=RobotState, normalize=True):
        """
        Convert a Pose object to a ROS message.

        :param pose: the pose to convert
        :type pose: Pose
        :param msg_type: the type of the ROS message to create
        :type msg_type: any ROS message which has position and rpy attributes
        :param normalize: whether to normalize the pose before converting it or not. Default is True.
        :type normalize: bool
        :return: ROS message
        :rtype: RobotState
        """
        if normalize:
            pose.normalize()
        msg = msg_type()

        for attr in ['position', 'rpy']:
            if not hasattr(msg, attr):
                raise TypeError(f'"{msg_type.name}" object has no attribute "{attr}"')

        msg.position.x = pose.x
        msg.position.y = pose.y
        msg.position.z = pose.z
        msg.rpy.roll = pose.roll
        msg.rpy.pitch = pose.pitch
        msg.rpy.yaw = pose.yaw
        return msg

    def pose_from_msg(self, msg) -> Pose:
        if hasattr(msg, 'pose'):
            p_msg = msg.pose
        else:
            p_msg = msg

        pose = Pose(0, 0, 0, 0, 0, 0)

        if not hasattr(p_msg, 'position'):
            raise TypeError(f'{type(p_msg)} does not have a position attribute')

        pose.x = p_msg.position.x
        pose.y = p_msg.position.y
        pose.z = p_msg.position.z

        if not hasattr(p_msg, 'rpy') and not hasattr(p_msg, 'orientation'):
            raise TypeError(f'{type(p_msg)} does not have a rpy nor an orientation attribute')
        elif not hasattr(p_msg, 'rpy'):
            pose.roll, pose.pitch, pose.yaw = euler_from_quaternion(self.quaternion_to_list(p_msg.orientation))
        else:
            pose.roll = p_msg.rpy.roll
            pose.pitch = p_msg.rpy.pitch
            pose.yaw = p_msg.rpy.yaw

        if hasattr(msg, 'header') and hasattr(msg.header, 'frame_id'):
            pose.metadata.frame = msg.header.frame_id
        return pose

    # -- Tools

    def get_current_tool_id(self):
        """
        Uses /niryo_robot_tools_commander/current_id  topic to get current tool id

        :return: Tool Id
        :rtype: Union[ToolID, int]
        """
        return self.__tools.get_current_tool_id()

    def get_current_tool_state(self):
        """
        Return the hardware state of the tool
        :return: the hardware state
        :rtype: int
        """
        return self.__tool_motor_state_ntv.value.state

    def update_tool(self):
        """
        Calls service niryo_robot_tools_commander/update_tool to update tool

        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.update_tool()

    def grasp_with_tool(self, pin_id=""):
        """
        Grasps with the tool linked to tool_id
        This action corresponds to
        - Close gripper for Grippers
        - Pull Air for Vacuum pump
        - Activate for Electromagnet

        :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
        :type pin_id: PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.grasp_with_tool(pin_id)

    def release_with_tool(self, pin_id=""):
        """
        Releases with the tool associated to tool_id
        This action corresponds to
        - Open gripper for Grippers
        - Push Air for Vacuum pump
        - Deactivate for Electromagnet

        :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
        :type pin_id: PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.release_with_tool(pin_id)

    # - Gripper
    def open_gripper(self, speed=500, max_torque_percentage=100, hold_torque_percentage=20):
        """
        Opens gripper with a speed 'speed'

        :param speed: Default -> 500
        :type speed: int
        :param max_torque_percentage: Default -> 100
        :type max_torque_percentage: int
        :param hold_torque_percentage: Default -> 20
        :type hold_torque_percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.open_gripper(speed, max_torque_percentage, hold_torque_percentage)

    def close_gripper(self, speed=500, max_torque_percentage=100, hold_torque_percentage=50):
        """
        Closes gripper with a speed 'speed'

        :param speed: Default -> 500
        :type speed: int
        :param max_torque_percentage: Default -> 100
        :type max_torque_percentage: int
        :param hold_torque_percentage: Default -> 20
        :type hold_torque_percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.close_gripper(speed, max_torque_percentage, hold_torque_percentage)

    # - Vacuum
    def pull_air_vacuum_pump(self):
        """
        Pulls air

        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.pull_air_vacuum_pump()

    def push_air_vacuum_pump(self):
        """
        Pulls air

        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.push_air_vacuum_pump()

    # - Electromagnet
    def setup_electromagnet(self, pin_id):
        """
        Setups electromagnet on pin

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.setup_electromagnet(pin_id)

    def activate_electromagnet(self, pin_id):
        """
        Activates electromagnet associated to electromagnet_id on pin_id

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.activate_electromagnet(pin_id)

    def deactivate_electromagnet(self, pin_id):
        """
        Deactivates electromagnet associated to electromagnet_id on pin_id

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.deactivate_electromagnet(pin_id)

    # - TCP
    def enable_tcp(self, enable=True):
        """
        Enables or disables the TCP function (Tool Center Point).
        If activation is requested, the last recorded TCP value will be applied.
        The default value depends on the gripper equipped.
        If deactivation is requested, the TCP will be coincident with the tool_link

        :param enable: True to enable, False otherwise.
        :type enable: bool
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.enable_tcp(enable)

    def get_tcp(self, as_list=False):
        """
        Returns the TCP state
        :param as_list: True to return the tcp position as a list of float
        :type as_list: bool
        :return: the tcp (enabled, position and orientation)
        :rtype: Tool msg object
        """
        tcp = self.__tools.get_tcp()
        if as_list:
            return [tcp.position.x, tcp.position.y, tcp.position.z, tcp.rpy.roll, tcp.rpy.pitch, tcp.rpy.yaw]
        return tcp

    def set_tcp(self, x, y, z, roll, pitch, yaw):
        """
        Activates the TCP function (Tool Center Point)
        and defines the transformation between the tool_link frame and the TCP frame

        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param roll:
        :type roll: float
        :param pitch:
        :type pitch: float
        :param yaw:
        :type yaw: float
        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.set_tcp(x, y, z, roll, pitch, yaw)

    def reset_tcp(self):
        """
        Resets the TCP (Tool Center Point) transformation.
        The TCP will be reset according to the tool equipped

        :return: status, message
        :rtype: (int, str)
        """
        return self.__tools.reset_tcp()

    def tool_reboot(self):
        """
        Reboots the motor of the tool equipped. Useful when an Overload error occurs. (cf HardwareStatus)

        :return: success, message
        :rtype: (bool, str)
        """
        return self.__tools.tool_reboot()

    # - Hardware
    def get_simulation_mode(self):
        """
        The simulation mode
        """
        return self.__simulation_mode

    def set_pin_mode(self, pin_id, pin_mode):
        """
        Sets pin number pin_id to mode pin_mode

        :param pin_id:
        :type pin_id: str
        :param pin_mode:
        :type pin_mode: int
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_rpi.srv import SetIOMode
        result = self._call_service('/niryo_robot_rpi/set_digital_io_mode', SetIOMode, pin_id, pin_mode)
        return self._classic_return_w_check(result)

    def digital_write(self, pin_id, digital_state):
        """
        Sets pin_id state to pin_state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :param digital_state:
        :type digital_state: Union[ PinState, bool]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_rpi.srv import SetDigitalIO
        result = self._call_service('/niryo_robot_rpi/set_digital_io', SetDigitalIO, pin_id, digital_state)
        return self._classic_return_w_check(result)

    def analog_write(self, pin_id, analog_state):
        """
        Sets pin_id state to pin_state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :param analog_state:
        :type analog_state: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_rpi.srv import SetAnalogIO
        result = self._call_service('/niryo_robot_rpi/set_analog_io', SetAnalogIO, pin_id, analog_state)
        return self._classic_return_w_check(result)

    def digital_read(self, pin_id):
        """
        Reads pin number pin_id and returns its state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :return: state
        :rtype: PinState
        """
        from niryo_robot_rpi.srv import GetDigitalIO
        result = self._call_service('/niryo_robot_rpi/get_digital_io', GetDigitalIO, pin_id)
        self._check_result_status(result)
        return PinState.LOW if result.value == 0 else PinState.HIGH

    def analog_read(self, pin_id):
        """
        Reads pin number pin_id and returns its state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :return: pin voltage
        :rtype: float
        """
        from niryo_robot_rpi.srv import GetAnalogIO
        result = self._call_service('/niryo_robot_rpi/get_analog_io', GetAnalogIO, pin_id)
        self._check_result_status(result)
        return result.value

    def get_digital_io_state(self):
        """
        Gets Digital IO state : Names, modes, states

        :return: Infos contains in a IOsState object (see niryo_robot_msgs)
        :rtype: IOsState
        """
        return self.__digital_io_state_ntv.value

    def get_analog_io_state(self):
        return self.__analog_io_state_ntv.value

    def get_digital_io_mode(self, pin_id):
        """
        Get a digital IO mode

        :param pin_id: the pin id of a digital io
        :type pin_id: str
        :return: The mode of the digital io (Input or Output)
        :rtype: PinMode
        """
        dio_states = self.get_digital_io_state()
        if pin_id in [dio.name for dio in dio_states.digital_inputs]:
            return PinMode.INPUT
        return PinMode.OUTPUT

    def get_available_disk_size(self):
        """
        Get the RPI available space on the SD card
        :return: the number of MegaBytes available
        :rtype: int
        """
        self.__storage_status_ntv.wait_for_message()
        return self.__storage_status_ntv.value.available_disk_size

    def get_ros_logs_size(self):
        """
        Get the ros logs size on the SD card
        :return: the size of the ros logs in MB
        :rtype: int
        """
        self.__storage_status_ntv.wait_for_message()
        return self.__storage_status_ntv.value.log_size

    def get_hardware_version(self):
        """
        Gets the robot hardware version
        """
        return self.__hardware_version

    def get_hardware_status(self):
        """
        Gets hardware status : Temperature, Hardware version, motors names & types ...

        :return: Infos contains in a HardwareStatus object (see niryo_robot_msgs)
        :rtype: HardwareStatus
        """
        return self.__hw_status_ntv.value

    @staticmethod
    def get_robot_status():
        msg = rospy.wait_for_message('/niryo_robot_status/robot_status', RobotStatus, 2)
        return msg

    @staticmethod
    def get_axis_limits():
        """
        Returns the joints and positions min and max values

        :return: An object containing all the values
        :rtype: dict
        """
        path_pattern = '/niryo_robot/robot_command_validation/{}/{}/{}'
        axis_limits = {
            'joint_limits': {
                'joint_1': None,
                'joint_2': None,
                'joint_3': None,
                'joint_4': None,
                'joint_5': None,
                'joint_6': None,
            },
            'position_limits': {
                'x': None,
                'y': None,
                'z': None,
            },
            'rpy_limits': {
                'roll': None,
                'pitch': None,
                'yaw': None,
            }
        }

        for axis_group in axis_limits:
            for axis in axis_limits[axis_group]:
                try:
                    limits = {
                        'min': rospy.get_param(path_pattern.format(axis_group, axis, 'min')),
                        'max': rospy.get_param(path_pattern.format(axis_group, axis, 'max')),
                    }
                except KeyError as e:
                    return False, str(e)

                axis_limits[axis_group][axis] = limits
        return True, axis_limits

    def reboot_motors(self):
        """
        Reboots the robots motors

        :raises NiryoRosWrapperException:
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_hardware_interface/reboot_motors', Trigger)
        return self._classic_return_w_check(result)

    # - Conveyor

    def set_conveyor(self):
        """
        Scans for conveyor on can bus. If conveyor detected, returns the conveyor ID

        :raises NiryoRosWrapperException:
        :return: ID
        :rtype: ConveyorID
        """
        from conveyor_interface.srv import SetConveyor, SetConveyorRequest
        req = SetConveyorRequest(cmd=SetConveyorRequest.ADD)
        result = self._call_service('/niryo_robot/conveyor/ping_and_set_conveyor', SetConveyor, req)

        # If no new conveyor is detected, it should not crash
        if result.status in [CommandStatus.NO_CONVEYOR_LEFT, CommandStatus.NO_CONVEYOR_FOUND]:
            rospy.loginfo_throttle(1, 'ROS Wrapper - No new conveyor found')
            return ConveyorID.NONE

        self._check_result_status(result)

        return self.__conveyor_id_to_conveyor_number(result.id)

    def unset_conveyor(self, conveyor_id):
        """
        Removes specific conveyor

        :param conveyor_id: Basically, ConveyorID.ID_1 or ConveyorID.ID_2
        :type conveyor_id: ConveyorID
        :raises NiryoRosWrapperException:
        :return: status, message
        :rtype: (int, str)
        """
        from conveyor_interface.srv import SetConveyor, SetConveyorRequest

        real_conveyor_id = self.__conveyor_number_to_conveyor_id(conveyor_id)
        if real_conveyor_id == ConveyorID.NONE.value:
            raise NiryoRosWrapperException(f'Unable to retrieve the conveyor corresponding to {conveyor_id}')
        req = SetConveyorRequest(cmd=SetConveyorRequest.REMOVE, id=real_conveyor_id)
        result = self._call_service('/niryo_robot/conveyor/ping_and_set_conveyor', SetConveyor, req)
        return self._classic_return_w_check(result)

    def control_conveyor(self, conveyor_id, bool_control_on, speed, direction):
        """
        Controls conveyor associated to conveyor_id.
        Then stops it if bool_control_on is False, else refreshes it speed and direction

        :param conveyor_id: ConveyorID.ID_1 or ConveyorID.ID_2
        :type conveyor_id: ConveyorID
        :param bool_control_on: True for activate, False for deactivate
        :type bool_control_on: bool
        :param speed: target speed
        :type speed: int
        :param direction: Target direction
        :type direction: ConveyorDirection
        :return: status, message
        :rtype: (int, str)
        """
        from conveyor_interface.srv import ControlConveyor, ControlConveyorRequest

        real_conveyor_id = self.__conveyor_number_to_conveyor_id(conveyor_id)
        if real_conveyor_id == ConveyorID.NONE.value:
            raise NiryoRosWrapperException(f'Unable to retrieve the conveyor corresponding to {conveyor_id}')
        req = ControlConveyorRequest(id=real_conveyor_id, control_on=bool_control_on, speed=speed, direction=direction)
        result = self._call_service('/niryo_robot/conveyor/control_conveyor', ControlConveyor, req)
        return self._classic_return_w_check(result)

    def get_conveyors_feedback(self):
        """
        Gives conveyors feedback

        :return: for each conveyor, its id, connection_state, running, speed and direction
        :rtype: list[dict] with the following keys:
                conveyor_id: ConveyorID, connection_state: bool, speed: int, direction: ConveyorDirection
        """
        serialized_feedback = [{
            'conveyor_id': self.__conveyor_id_to_conveyor_number(f.conveyor_id),
            'connection_state': f.connection_state,
            'running': f.running,
            'speed': f.speed,
            'direction': f.direction
        } for f in self.__conveyors_feedback_ntv.value.conveyors]
        return serialized_feedback

    def get_conveyors_number(self):
        fb = self.__conveyors_feedback_ntv.value
        return [self.__conveyor_id_to_conveyor_number(conveyor.conveyor_id) for conveyor in fb.conveyors]

    def __conveyor_number_to_conveyor_id(self, conveyor_number):
        """
        Returns the conveyor ID associated to the conveyor number (an enum member)
        :param conveyor_number: a conveyor number
        :type conveyor_number: ConveyorID
        :return: the conveyor ID
        :rtype: int

        Example: ::

            self.__conveyor_number_to_conveyor_id(ConveyorID.ID_1) -> 9

        """
        ids = [conveyor.conveyor_id for conveyor in self.__conveyors_feedback_ntv.value.conveyors]
        try:
            conveyor_id = dict(zip([ConveyorID.ID_1, ConveyorID.ID_2], ids))[conveyor_number]
        except KeyError:
            conveyor_id = ConveyorID.NONE.value
        return conveyor_id

    def __conveyor_id_to_conveyor_number(self, conveyor_id):
        """
        Returns the conveyor number (an enum member) associated to the conveyor ID
        :param conveyor_id: a conveyor ID
        :type conveyor_id: int
        :return: the conveyor number
        :rtype: ConveyorID

        Example: ::

                self.__conveyor_id_to_conveyor_number(9) -> ConveyorID.ID_1
        """
        if conveyor_id not in self.__conveyor_id_to_number:
            raise ValueError(f'Conveyor ID {conveyor_id} is not a valid conveyor ID')
        return self.__conveyor_id_to_number[conveyor_id]

    # - Vision

    def control_video_stream(self, stream_on):
        """
        Control if the video stream should be activated or not
        :param stream_on: if True, activate the video stream. Deactivate it otherwise
        :return: None
        """
        result = self._call_service('/niryo_robot_vision/start_stop_video_streaming', SetBool, stream_on)
        self._check_result_status(result)

    def get_compressed_image(self, with_seq=False):
        """
        Gets last stream image in a compressed format

        :return: string containing a JPEG compressed image
        :rtype: str
        """
        compressed_img = self.__compressed_image_message_ntv.value

        if with_seq:
            return compressed_img.data, compressed_img.header.seq

        return compressed_img.data

    def set_brightness(self, brightness_factor):
        """
        Modifies image brightness

        :param brightness_factor: How much to adjust the brightness. 0.5 will
            give a darkened image, 1 will give the original image while
            2 will enhance the brightness by a factor of 2.
        :type brightness_factor: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_vision.srv import SetImageParameter
        result = self._call_service('/niryo_robot_vision/set_brightness', SetImageParameter, brightness_factor)
        return self._classic_return_w_check(result)

    def set_contrast(self, contrast_factor):
        """
        Modifies image contrast

        :param contrast_factor: While a factor of 1 gives original image.
            Making the factor towards 0 makes the image greyer, while factor>1 increases the contrast of the image.
        :type contrast_factor: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_vision.srv import SetImageParameter
        result = self._call_service('/niryo_robot_vision/set_contrast', SetImageParameter, contrast_factor)
        return self._classic_return_w_check(result)

    def set_saturation(self, saturation_factor):
        """
        Modifies image saturation

        :param saturation_factor: How much to adjust the saturation. 0 will
            give a black and white image, 1 will give the original image while
            2 will enhance the saturation by a factor of 2.
        :type saturation_factor: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_vision.srv import SetImageParameter
        result = self._call_service('/niryo_robot_vision/set_saturation', SetImageParameter, saturation_factor)
        return self._classic_return_w_check(result)

    @staticmethod
    def get_image_parameters():
        """
        Gets last stream image parameters: Brightness factor, Contrast factor, Saturation factor.

        Brightness factor: How much to adjust the brightness. 0.5 will give a darkened image,
        1 will give the original image while 2 will enhance the brightness by a factor of 2.

        Contrast factor: While a factor of 1 gives original image.
        Making the factor towards 0 makes the image greyer, while factor>1 increases the contrast of the image.

        Saturation factor: 0 will give a black and white image, 1 will give the original image while
        2 will enhance the saturation by a factor of 2.

        :return:  Brightness factor, Contrast factor, Saturation factor
        :rtype: float, float, float
        """
        from niryo_robot_vision.msg import ImageParameters

        try:
            img_param_msg = rospy.wait_for_message('/niryo_robot_vision/video_stream_parameters',
                                                   ImageParameters,
                                                   timeout=5)
        except rospy.ROSException:
            raise NiryoRosWrapperException(
                "Could not get image parameters on the {} topic".format('/niryo_robot_vision/video_stream_parameters'))

        return img_param_msg.brightness_factor, img_param_msg.contrast_factor, img_param_msg.saturation_factor

    def get_target_pose_from_rel(self, workspace_name, height_offset, x_rel, y_rel, yaw_rel, as_list=False):
        """
        Given a pose (x_rel, y_rel, yaw_rel) relative to a workspace, this function
        returns the robot pose in which the current tool will be able to pick an object at this pose.
        The height_offset argument (in m) defines how high the tool will hover over the workspace. If height_offset = 0,
        the tool will nearly touch the workspace.

        :param workspace_name: name of the workspace
        :type workspace_name: str
        :param height_offset: offset between the workspace and the target height
        :type height_offset: float
        :param x_rel:
        :type x_rel: float
        :param y_rel:
        :type y_rel: float
        :param yaw_rel:
        :type yaw_rel: float
        :param as_list: whether return the pose as a list or a Pose object
        :type as_list: bool
        :return: target_pose
        :rtype: RobotState
        """
        from niryo_robot_poses_handlers.srv import GetTargetPose

        result = self._call_service('/niryo_robot_poses_handlers/get_target_pose',
                                    GetTargetPose,
                                    workspace_name,
                                    height_offset,
                                    x_rel,
                                    y_rel,
                                    yaw_rel)
        self._check_result_status(result)
        if as_list:
            return [
                result.target_pose.position.x,
                result.target_pose.position.y,
                result.target_pose.position.z,
                result.target_pose.rpy.roll,
                result.target_pose.rpy.pitch,
                result.target_pose.rpy.yaw
            ]
        return result.target_pose

    def get_target_pose_from_cam(self, workspace_name, height_offset, shape, color, as_list=False):
        """
        First detects the specified object using the camera and then returns the robot pose in which the object can
        be picked with the current tool

        :param workspace_name: name of the workspace
        :type workspace_name: str
        :param height_offset: offset between the workspace and the target height
        :type height_offset: float
        :param shape: shape of the target
        :type shape: ObjectShape
        :param color: color of the target
        :type color: ObjectColor
        :param as_list: whether return the pose as a list or a Pose object
        :type as_list: bool
        :return: object_found, object_pose, object_shape, object_color
        :rtype: (bool, RobotState, str, str)
        """
        object_found, rel_pose, obj_shape, obj_color = self.detect_object(workspace_name, shape, color)
        if not object_found:
            return False, None, "", ""
        obj_pose = self.get_target_pose_from_rel(workspace_name,
                                                 height_offset,
                                                 rel_pose.x,
                                                 rel_pose.y,
                                                 rel_pose.yaw,
                                                 as_list=as_list)
        return True, obj_pose, obj_shape, obj_color

    def vision_pick_w_obs_joints(self,
                                 workspace_name,
                                 height_offset,
                                 shape,
                                 color,
                                 observation_joints: Union[list, JointsPosition]):
        """
        .. deprecated:: 5.5.0
           You should use :func:`vision_pick` by setting obs_pose with a `JointsPosition` object.

        Move Joints to observation_joints, then executes a vision pick
        """
        warnings.warn("You should use vision_pick by setting obs_pose with a JointsPosition object.",
                      DeprecationWarning)
        if isinstance(observation_joints, list):
            observation_joints = JointsPosition(*observation_joints)
        return self.vision_pick(workspace_name, height_offset, shape, color, obs_pose=observation_joints)

    def vision_pick_w_obs_pose(self,
                               workspace_name,
                               height_offset,
                               shape,
                               color,
                               observation_pose_list: Union[list, Pose]):
        """
        .. deprecated:: 5.5.0
           You should use :func:`vision_pick` by setting obs_pose with a `Pose` object.

        Move Pose to observation_pose, then executes a vision pick
        """
        warnings.warn("You should use vision_pick by setting obs_pose with a Pose object.", DeprecationWarning)
        if isinstance(observation_pose_list, list):
            observation_pose_list = Pose(*observation_pose_list)
        return self.vision_pick(workspace_name, height_offset, shape, color, obs_pose=observation_pose_list)

    def vision_pick(self, workspace_name, height_offset, shape, color, obs_pose: RobotPosition = None, **kwargs):
        """
        Picks the specified object from the workspace. This function has multiple phases:

        1. move to the observation pose (if specified)
        2. detects object using the camera
        3. prepares the current tool for picking
        4. approaches the object
        5. moves down to the correct picking pose
        6. actuates the current tool
        7. lifts the object

        :param workspace_name: name of the workspace
        :type workspace_name: str
        :param height_offset: offset between the workspace and the target height
        :type height_offset: float
        :param shape: shape of the target
        :type shape: ObjectShape
        :param color: color of the target
        :type color: ObjectColor
        :param obs_pose: The observation pose
        :type obs_pose: RobotPosition
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: object_found, object_shape, object_color
        :rtype: (bool, ObjectShape, ObjectColor)
        """

        def move_sequence(**kwargs_):
            if obs_pose is not None:
                self.move(obs_pose, **kwargs_)

            object_found, rel_pose, obj_shape, obj_color = self.detect_object(workspace_name, shape, color)
            if not object_found:
                return False, "", ""

            pick_pose = self.get_target_pose_from_rel(workspace_name,
                                                      height_offset,
                                                      rel_pose.x,
                                                      rel_pose.y,
                                                      rel_pose.yaw)
            approach_pose = self.get_target_pose_from_rel(workspace_name,
                                                          height_offset + 0.05,
                                                          rel_pose.x,
                                                          rel_pose.y,
                                                          rel_pose.yaw)

            self.release_with_tool()
            self.execute_trajectory([self.pose_from_msg(x) for x in (approach_pose, pick_pose)])
            self.grasp_with_tool()
            self.move(self.pose_from_msg(approach_pose), **kwargs_)
            return True, obj_shape, obj_color

        return self.__move_sequence(move_sequence, **kwargs)

    def move_to_object(self, workspace, height_offset, shape, color, **kwargs):
        """
        Same as `get_target_pose_from_cam` but directly moves to this position

        :param workspace: name of the workspace
        :type workspace: str
        :param height_offset: offset between the workspace and the target height
        :type height_offset: float
        :param shape: shape of the target
        :type shape: ObjectShape
        :param color: color of the target
        :type color: ObjectColor
        :param kwargs: any keyword argument taken by the move function
        :type kwargs: dict
        :return: object_found, object_shape, object_color
        :rtype: (bool, ObjectShape, ObjectColor)
        """
        obj_found, obj_pose, obj_shape, obj_color = self.get_target_pose_from_cam(
            workspace, height_offset, shape, color)
        if not obj_found:
            return False, "", ""
        self.move(self.pose_from_msg(obj_pose), **kwargs)
        return True, obj_shape, obj_color

    def detect_object(self, workspace_name, shape, color):
        """

        :param workspace_name: name of the workspace
        :type workspace_name: str
        :param shape: shape of the target
        :type shape: ObjectShape
        :param color: color of the target
        :type color: ObjectColor
        :return: object_found, object_pose, object_shape, object_color
        :rtype: (bool, ObjectPose, str, str)
        """
        from niryo_robot_vision.srv import ObjDetection

        ratio = self.get_workspace_ratio(workspace_name)
        response = self._call_service("/niryo_robot_vision/obj_detection_rel", ObjDetection, shape, color, ratio, False)

        if response.status == CommandStatus.MARKERS_NOT_FOUND:
            rospy.logwarn_throttle(1, 'ROS Wrapper - Markers Not Found')
        elif response.status == CommandStatus.VIDEO_STREAM_NOT_RUNNING:
            rospy.logwarn_throttle(1, 'Video Stream not running')

        object_detected = response.status == CommandStatus.SUCCESS
        return object_detected, response.obj_pose, response.obj_type, response.obj_color

    def get_camera_intrinsics(self):
        """
        Gets calibration object: camera intrinsics, distortions coefficients

        :return: raw camera intrinsics, distortions coefficients
        :rtype: (list, list)
        """

        camera_intrinsics = self.__camera_intrinsics_message_ntv.value
        return camera_intrinsics.K, camera_intrinsics.D

    def get_debug_markers(self):
        from niryo_robot_vision.srv import DebugMarkers, DebugMarkersRequest
        result = self._call_service('/niryo_robot_vision/debug_markers', DebugMarkers, DebugMarkersRequest())
        return result

    def get_debug_colors(self, color):
        from niryo_robot_vision.srv import DebugColorDetection, DebugColorDetectionRequest
        req = DebugColorDetectionRequest(color=color)
        result = self._call_service('/niryo_robot_vision/debug_colors', DebugColorDetection, req)
        return result

    # - Workspace
    def save_workspace_from_poses(self, name, list_poses_raw: List[Union[List[float], Pose]]):
        """
        Saves workspace by giving the poses of the robot to point its 4 corners
        with the calibration Tip. Corners should be in the good order

        :param name: workspace name, max 30 char.
        :type name: str
        :param list_poses_raw: list of 4 corners pose
        :type list_poses_raw: list[list]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageWorkspace, ManageWorkspaceRequest

        if len(list_poses_raw) > 0 and isinstance(list_poses_raw[0], list):
            list_poses_raw = [Pose(*pose) for pose in list_poses_raw]

        for pose in list_poses_raw:
            pose.normalize()

        list_poses = [self.msg_from_pose(pose) for pose in list_poses_raw]

        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.SAVE
        if len(name) > 30:
            rospy.logwarn('ROS Wrapper - Workspace name is too long, using : %s instead', name[:30])
        req.workspace.name = name[:30]
        req.workspace.poses = list_poses
        result = self._call_service('/niryo_robot_poses_handlers/manage_workspace', ManageWorkspace, req)
        return self._classic_return_w_check(result)

    def save_workspace_from_points(self, name, list_points_raw):
        """
        Saves workspace by giving the poses of its 4 corners in the good order

        :param name: workspace name, max 30 char.
        :type name: str
        :param list_points_raw: list of 4 corners [x, y, z]
        :type list_points_raw: list[list[float]]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageWorkspace, ManageWorkspaceRequest

        list_points = [Point(*point) for point in list_points_raw]
        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.SAVE_WITH_POINTS
        if len(name) > 30:
            rospy.logwarn('ROS Wrapper - Workspace name is too long, using : %s instead', name[:30])
        req.workspace.name = name[:30]
        req.workspace.points = list_points
        result = self._call_service('/niryo_robot_poses_handlers/manage_workspace', ManageWorkspace, req)
        return self._classic_return_w_check(result)

    def delete_workspace(self, name):
        """
        Calls workspace manager to delete a certain workspace

        :param name: workspace name
        :type name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageWorkspace, ManageWorkspaceRequest

        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.DELETE
        req.workspace.name = name
        result = self._call_service('/niryo_robot_poses_handlers/manage_workspace', ManageWorkspace, req)
        return self._classic_return_w_check(result)

    def get_workspace_poses(self, name):
        """
        Gets the 4 workspace poses of the workspace called 'name'

        :param name: workspace name
        :type name: str
        :return: List of the 4 workspace poses
        :rtype: list[list]
        """
        from niryo_robot_poses_handlers.srv import GetWorkspaceRobotPoses

        result = self._call_service('/niryo_robot_poses_handlers/get_workspace_poses', GetWorkspaceRobotPoses, name)
        self._check_result_status(result)

        list_p_raw = [self.pose_from_msg(p) for p in result.poses]
        return list_p_raw

    def get_workspace_ratio(self, name):
        """
        Gives the length over width ratio of a certain workspace

        :param name: workspace name
        :type name: str
        :return: ratio
        :rtype: float
        """
        from niryo_robot_poses_handlers.srv import GetWorkspaceRatio

        result = self._call_service('/niryo_robot_poses_handlers/get_workspace_ratio', GetWorkspaceRatio, name)
        self._check_result_status(result)
        return result.ratio

    @staticmethod
    def get_workspace_list(with_desc=False):
        """
        Asks the workspace manager service names of the available workspace

        :return: list of workspaces name
        :rtype: list[str]
        """
        workspace_list = rospy.wait_for_message('/niryo_robot_poses_handlers/workspace_list', BasicObjectArray, 2)
        names = [workspace.name for workspace in workspace_list.objects]
        if with_desc:
            descriptions = [workspace.description for workspace in workspace_list.objects]
            return names, descriptions
        return names

    # - Software

    def get_software_version(self):
        """
        Get the robot software version

        :return: rpi_image_version, ros_niryo_robot_version, motor_names, stepper_firmware_versions
        :rtype: (str, str, list[str], list[str])
        """
        value = self.__software_version_ntv.wait_for_message()
        return value

    # - Ned

    @property
    def database(self):
        return self.__database

    # - Ned 2

    @property
    def led_ring(self):
        """
        Manages the LED ring

        Example: ::

            from niryo_robot_python_ros_wrapper.ros_wrapper import *
            import rospy

            rospy.init_node('ros_wrapper_node')
            robot = NiryoRosWrapper()
            robot.led_ring.solid(color=[255, 255, 255])

        :return: LedRingRosWrapper API instance
        :rtype: LedRingRosWrapper
        """
        return self.__led_ring

    @property
    def sound(self):
        """
        Manages sound

        Example: ::

            from niryo_robot_python_ros_wrapper.ros_wrapper import *
            import rospy

            rospy.init_node('ros_wrapper_node')
            robot = NiryoRosWrapper()
            robot.sound.play(sound.sounds[0])

        :return: SoundRosWrapper API instance
        :rtype: SoundRosWrapper
        """
        return self.__sound

    @property
    def custom_button(self):
        """
        Manages the custom button

        Example: ::

            from niryo_robot_python_ros_wrapper.ros_wrapper import *
            import rospy

            rospy.init_node('ros_wrapper_node')
            robot = NiryoRosWrapper()
            print(robot.custom_button.state)

        :return: CustomButtonRosWrapper API instance
        :rtype:  CustomButtonRosWrapper
        """
        return self.__custom_button

    @property
    def robot_status(self):
        return self.__robot_status
