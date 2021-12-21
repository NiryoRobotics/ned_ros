#!/usr/bin/env python
# Lib
import rospy

# Command Status
from niryo_robot_msgs.msg import CommandStatus, SoftwareVersion

from niryo_robot_python_ros_wrapper.niryo_action_client import NiryoActionClient
from niryo_robot_python_ros_wrapper.niryo_topic_value import NiryoTopicValue

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

from conveyor_interface.msg import ConveyorFeedbackArray
from niryo_robot_msgs.msg import HardwareStatus, RobotState, RPY
from niryo_robot_rpi.msg import DigitalIO, DigitalIOState, AnalogIO, AnalogIOState
from niryo_robot_tools_commander.msg import ToolCommand
from niryo_robot_status.msg import RobotStatus

# Services
from niryo_robot_msgs.srv import GetNameDescriptionList, SetBool, SetInt, Trigger, Ping, SetFloat

# Actions
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from niryo_robot_tools_commander.msg import ToolGoal, ToolAction
from niryo_robot_arm_commander.msg import ArmMoveCommand, RobotMoveGoal, RobotMoveAction

# Enums
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import NiryoRosWrapperException
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import *


class NiryoRosWrapper:
    LOGS_LEVELS = {
        rospy.INFO: 'INFO',
        rospy.WARN: 'WARNING',
        rospy.ERROR: 'ERROR',
        rospy.FATAL: 'FATAL',
        rospy.DEBUG: 'DEBUG'
    }

    def __init__(self):
        # - Getting ROS parameters
        self.__service_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/service_timeout")
        self.__simulation_mode = rospy.get_param("/niryo_robot/simulation_mode")
        self.__hardware_version = rospy.get_param("/niryo_robot/hardware_version")

        if self.__hardware_version in ['ned', 'ned2']:
            self.__node_name = rospy.get_name()
            self.__ping_ros_wrapper_srv = rospy.Service("~/ping", Trigger, self.__ping_ros_wrapper_callback)
            rospy.wait_for_service("/niryo_robot_status/ping_ros_wrapper", timeout=5)
            self.__advertise_ros_wrapper_srv = rospy.ServiceProxy("/niryo_robot_status/ping_ros_wrapper", Ping)
            self.__advertise_ros_wrapper_srv(self.__node_name, True)
            rospy.on_shutdown(self.__advertise_stop)

        # - Publishers
        # Highlight publisher (to highlight blocks in Blockly interface)
        self.__highlight_block_publisher = rospy.Publisher('/niryo_robot_blockly/highlight_block', String,
                                                           queue_size=10)

        # Break point publisher (for break point blocks in Blockly interface)
        self.__break_point_publisher = rospy.Publisher('/niryo_robot_blockly/break_point', Int32, queue_size=10)

        # -- Subscribers
        # - Pose
        self.__joints_ntv = NiryoTopicValue('/joint_states', JointState)
        self.__pose_ntv = NiryoTopicValue('/niryo_robot/robot_state', RobotState)

        # - Hardware
        self.__learning_mode_on_ntv = NiryoTopicValue('/niryo_robot/learning_mode/state', Bool)
        self.__hw_status_ntv = NiryoTopicValue('/niryo_robot_hardware_interface/hardware_status', HardwareStatus)
        self.__digital_io_state_ntv = NiryoTopicValue('/niryo_robot_rpi/digital_io_state', DigitalIOState)
        self.__analog_io_state_ntv = NiryoTopicValue('/niryo_robot_rpi/analog_io_state', AnalogIOState)
        self.__current_tool_id_ntv = NiryoTopicValue('/niryo_robot_tools_commander/current_id', Int32)
        self.__max_velocity_scaling_factor_ntv = NiryoTopicValue('/niryo_robot/max_velocity_scaling_factor', Int32)

        # - Vision
        self.__compressed_image_message_ntv = NiryoTopicValue('/niryo_robot_vision/compressed_video_stream',
                                                              CompressedImage, queue_size=1)
        self.__camera_intrinsics_message_ntv = NiryoTopicValue('/niryo_robot_vision/camera_intrinsics', CameraInfo,
                                                               queue_size=1)
        # - Conveyor
        self.__conveyors_feedback_ntv = NiryoTopicValue('/niryo_robot/conveyor/feedback', ConveyorFeedbackArray)

        # Software
        self.__software_version_ntv = NiryoTopicValue('/niryo_robot_hardware_interface/software_version',
                                                      SoftwareVersion,
                                                      queue_size=1)

        # - Action server
        # Robot action
        self.__robot_action_nac = NiryoActionClient('/niryo_robot_arm_commander/robot_action', RobotMoveAction,
                                                    RobotMoveGoal)
        self.__follow_joint_traj_nac = NiryoActionClient(
            rospy.get_param("/niryo_robot_arm_commander/joint_controller_name") + "/follow_joint_trajectory",
            FollowJointTrajectoryAction, FollowJointTrajectoryGoal)

        # Tool action
        self.__tool_action_nac = NiryoActionClient('/niryo_robot_tools_commander/action_server', ToolAction,
                                                   ToolGoal)

        # database
        from niryo_robot_database.api import DatabaseRosWrapper
        self.__database = DatabaseRosWrapper(self.__service_timeout)

        # system_api_client
        from niryo_robot_system_api_client.api import SystemApiClientRosWrapper
        self.__system_api_client = SystemApiClientRosWrapper(self.__service_timeout)

        if self.__hardware_version == 'ned2':
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

    def __del__(self):
        del self

    def __advertise_stop(self):
        if self.__hardware_version in ['ned', 'ned2']:
            self.__advertise_ros_wrapper_srv(self.__node_name, False)

    def __ping_ros_wrapper_callback(self):
        return CommandStatus.SUCCESS, self.__node_name

    @classmethod
    def wait_for_nodes_initialization(cls, simulation_mode=False):
        params_checked = [
            '/niryo_robot_poses_handlers/initialized',
            '/niryo_robot_arm_commander/initialized',
        ]
        while not all([rospy.has_param(param) for param in params_checked]):
            rospy.sleep(0.1)
        if simulation_mode:
            rospy.sleep(1)  # Waiting to be sure Gazebo is open

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

    # -- Service & Action executors
    def __call_service(self, service_name, service_msg_type, *args):
        """
        Wait for the service called service_name
        Then call the service with args

        :param service_name:
        :param service_msg_type:
        :param args: Tuple of arguments
        :raises NiryoRosWrapperException: Timeout during waiting of services
        :return: Response
        """
        # Connect to service
        try:
            rospy.wait_for_service(service_name, self.__service_timeout)
        except rospy.ROSException as e:
            raise NiryoRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise NiryoRosWrapperException(e)

    # --- Functions interface
    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise NiryoRosWrapperException("Error Code : {}\nMessage : {}".format(result.status, result.message))

    @staticmethod
    def return_success(message=""):
        return CommandStatus.SUCCESS, message

    # -- Main Purpose
    def request_new_calibration(self):
        """
        Call service to set the request calibration value. If failed, raise NiryoRosWrapperException.

        :return: status, message
        :rtype: (int, str)
        """
        try:
            return self.__call_service('/niryo_robot/joints_interface/request_new_calibration', Trigger)
        except rospy.ROSException as e:
            raise NiryoRosWrapperException(str(e))

    def calibrate_auto(self):
        """
        Call service to calibrate motors then wait for its end. If failed, raise NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        return self.__calibrate(calib_type_int=1)

    def calibrate_manual(self):
        """
        Call service to calibrate motors then wait for its end. If failed, raise NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        return self.__calibrate(calib_type_int=2)

    def __calibrate(self, calib_type_int):
        """
        Call service to calibrate motors then wait for its end. If failed, raise NiryoRosWrapperException

        :param calib_type_int: 1 for auto-calibration & 2 for manual calibration
        :return: status, message
        :rtype: (int, str)
        """
        hw_status = self.__hw_status_ntv.wait_for_message()
        if not hw_status.calibration_needed:
            return self.return_success("Calibration not needed")

        result = self.__call_service('/niryo_robot/joints_interface/calibrate_motors',
                                     SetInt, calib_type_int)
        self.__check_result_status(result)
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
        Use /niryo_robot/learning_mode/state topic subscriber to get learning mode status

        :return: ``True`` if activate else ``False``
        :rtype: bool
        """
        return self.__learning_mode_on_ntv.value.data

    def set_learning_mode(self, set_bool):
        """
        Call service to set_learning_mode according to set_bool. If failed, raise NiryoRosWrapperException

        :param set_bool: ``True`` to activate, ``False`` to deactivate
        :type set_bool: bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot/learning_mode/activate',
                                     SetBool, set_bool)
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    def get_max_velocity_scaling_factor(self):
        """
        Get the max velocity scaling factor
        :return: max velocity scaling factor
        :rtype: float
        """
        return self.__max_velocity_scaling_factor_ntv.value

    def set_arm_max_velocity(self, percentage):
        """
        Set relative max velocity (in %)

        :param percentage: Percentage of max velocity
        :type percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_arm_commander/set_max_velocity_scaling_factor',
                                     SetInt, percentage)
        return self.__classic_return_w_check(result)

    def set_arm_max_acceleration(self, percentage):
        """
        Set relative max acceleration (in %)

        :param percentage: Percentage of max acceleration
        :type percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_arm_commander/set_acceleration_factor', SetFloat, percentage / 100.)
        return self.__classic_return_w_check(result)

    # - Useful functions
    @staticmethod
    def wait(time_sleep):
        rospy.sleep(time_sleep)

    # -- Move

    # - Pose

    def get_joints(self):
        """
        Use /joint_states topic to get joints status

        :return: list of joints value
        :rtype: list[float]
        """
        return list(self.__joints_ntv.value.position[:6])

    def get_joint_names(self):
        """
        Use /joint_states topic to get the name of the joints

        :return: list of the name of the joints
        :rtype: list[string]
        """
        return list(self.__joints_ntv.value.name[:6])

    def get_pose(self):
        """
        Use /niryo_robot/robot_state topic to get pose status

        :return: RobotState object (position.x/y/z && rpy.roll/pitch/yaw && orientation.x/y/z/w)
        :rtype: RobotState
        """
        return self.__pose_ntv.value

    def get_pose_as_list(self):
        """
        Use /niryo_robot/robot_state topic to get pose status

        :return: list corresponding to [x, y, z, roll, pitch, yaw]
        :rtype: list[float]
        """
        p = self.get_pose()
        return [p.position.x, p.position.y, p.position.z, p.rpy.roll, p.rpy.pitch, p.rpy.yaw]

    def move_joints(self, j1, j2, j3, j4, j5, j6):
        """
        Execute Move joints action

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
        :return: status, message
        :rtype: (int, str)
        """
        cmd = ArmMoveCommand(cmd_type=ArmMoveCommand.JOINTS, joints=[j1, j2, j3, j4, j5, j6])
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)

    def move_to_sleep_pose(self):
        """
        Move to Sleep pose which allows the user to activate the learning mode without the risk
        of the robot hitting something because of gravity

        :return: status, message
        :rtype: (int, str)
        """
        return self.move_joints(0.0, 0.50, -1.25, 0.0, 0.0, 0.0)

    def move_pose(self, x, y, z, roll, pitch, yaw):
        """
        Move robot end effector pose to a (x, y, z, roll, pitch, yaw) pose.

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

        return self.__move_pose_with_cmd(ArmMoveCommand.POSE, x, y, z, roll, pitch, yaw)

    def move_circle(self, x, y, z):
        return self.__move_pose_with_cmd(ArmMoveCommand.DRAW_CIRCLE, x, y, z, 0, 0, 0)

    def move_pose_saved(self, pose_name):
        """
        Move robot end effector pose to a pose saved

        :param pose_name:
        :type pose_name: str
        :return: status, message
        :rtype: (int, str)
        """
        x, y, z, roll, pitch, yaw = self.get_pose_saved(pose_name)
        return self.__move_pose_with_cmd(ArmMoveCommand.POSE, x, y, z, roll, pitch, yaw)

    def __move_pose_with_cmd(self, cmd_type, *pose):
        """
        Execute Move pose action

        :param cmd_type: Command Type
        :type cmd_type: ArmMoveCommand -> POSE, LINEAR_POSE
        :param pose: tuple corresponding to x, y, z, roll, pitch, yaw
        :return: status, message
        :rtype: (int, str)
        """
        x, y, z, roll, pitch, yaw = pose
        cmd = ArmMoveCommand(cmd_type=cmd_type, position=Point(x, y, z), rpy=RPY(roll=roll, pitch=pitch, yaw=yaw))
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)

    def shift_pose(self, axis, value):
        """
        Execute Shift pose action

        :param axis: Value of RobotAxis enum corresponding to where the shift happens
        :type axis: ShiftPose
        :param value: shift value
        :type value: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.msg import ShiftPose
        cmd = ArmMoveCommand(cmd_type=ArmMoveCommand.SHIFT_POSE, shift=ShiftPose(axis_number=axis, value=value))
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)

    def shift_linear_pose(self, axis, value):
        """
        Execute Shift pose action with a linear trajectory

        :param axis: Value of RobotAxis enum corresponding to where the shift happens
        :type axis: ShiftPose
        :param value: shift value
        :type value: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.msg import ShiftPose
        cmd = ArmMoveCommand(cmd_type=ArmMoveCommand.SHIFT_LINEAR_POSE, shift=ShiftPose(axis_number=axis, value=value))
        goal = RobotMoveGoal(cmd=cmd)
        return self.__robot_action_nac.execute(goal)

    def move_linear_pose(self, x, y, z, roll, pitch, yaw):
        """
        Move robot end effector pose to a (x, y, z, roll, pitch, yaw) pose, with a linear trajectory

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
        return self.__move_pose_with_cmd(ArmMoveCommand.LINEAR_POSE, x, y, z, roll, pitch, yaw)

    def move_spiral(self, radius=0.2, angle_step=5, nb_steps=72, plan=1):
        """
        Call robot action service to draw a spiral trajectory

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

        msg_dict = {result.SUCCESSFUL: "Successful",
                    result.INVALID_GOAL: "Invalid goal",
                    result.INVALID_JOINTS: "Invalid joints",
                    result.OLD_HEADER_TIMESTAMP: "Old header timestamp",
                    result.PATH_TOLERANCE_VIOLATED: "Path tolerance violated",
                    result.GOAL_TOLERANCE_VIOLATED: "Goal tolerance violated"}

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
        Stop the robot movement

        :return: list of joints value
        :rtype: list[float]
        """
        result = self.__call_service('/niryo_robot_commander/stop_command', Trigger)
        return self.__classic_return_w_check(result)

    def set_jog_use_state(self, state):
        """
        Turn jog controller On or Off

        :param state: ``True`` to turn on, else ``False``
        :type state: bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot/jog_interface/enable', SetBool, state)
        return self.__classic_return_w_check(result)

    def jog_joints_shift(self, shift_values):
        """
        Make a Jog on joints position

        :param shift_values: list corresponding to the shift to be applied to each joint
        :type shift_values: list[float]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import JogShift, JogShiftRequest
        return self.__jog_shift(JogShiftRequest.JOINTS_SHIFT, shift_values)

    def jog_pose_shift(self, shift_values):
        """
        Make a Jog on end-effector position

        :param shift_values: list corresponding to the shift to be applied to the position
        :type shift_values: list[float]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_arm_commander.srv import JogShift, JogShiftRequest
        return self.__jog_shift(JogShiftRequest.POSE_SHIFT, shift_values)

    def __jog_shift(self, cmd, shift_values):
        from niryo_robot_arm_commander.srv import JogShift, JogShiftRequest
        result = self.__call_service('/niryo_robot/jog_interface/jog_shift_commander', JogShift, cmd, shift_values)

        return self.__classic_return_w_check(result)

    def forward_kinematics(self, j1, j2, j3, j4, j5, j6):
        """
        Compute forward kinematics

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
        :return: list corresponding to [x, y, z, roll, pitch, yaw]
        :rtype: list[float]
        """
        from niryo_robot_arm_commander.srv import GetFK
        joints = [j1, j2, j3, j4, j5, j6]
        result = self.__call_service('/niryo_robot/kinematics/forward', GetFK, joints)
        return self.robot_state_msg_to_list(result.pose)

    def inverse_kinematics(self, x, y, z, roll, pitch, yaw):
        """
        Compute inverse kinematics

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
        :return: list of joints value
        :rtype: list[float]
        """
        from niryo_robot_arm_commander.srv import GetIK
        state = RobotState(position=Point(x, y, z), rpy=RPY(roll, pitch, yaw))
        result = self.__call_service('/niryo_robot/kinematics/inverse', GetIK, state)
        if not result.success:
            raise NiryoRosWrapperException("Failed to perform invert kinematic")

        return result.joints

    # - Saved Pose

    def get_pose_saved(self, pose_name):
        """
        Get saved pose from robot intern storage
        Will raise error if position does not exist

        :param pose_name: Pose Name
        :type pose_name: str
        :return: x, y, z, roll, pitch, yaw
        :rtype: tuple[float]
        """
        from niryo_robot_poses_handlers.srv import GetPose

        result = self.__call_service('/niryo_robot_poses_handlers/get_pose', GetPose, pose_name)
        self.__check_result_status(result)
        pose = result.pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        roll, pitch, yaw = pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw
        return x, y, z, roll, pitch, yaw

    def save_pose(self, name, x, y, z, roll, pitch, yaw):
        """
        Save pose in robot's memory

        :param name:
        :type name: str
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
        from niryo_robot_poses_handlers.srv import ManagePose, ManagePoseRequest

        req = ManagePoseRequest()
        req.cmd = ManagePoseRequest.SAVE
        req.pose.name = name
        req.pose.position = Point(x, y, z)
        req.pose.rpy = RPY(roll, pitch, yaw)

        result = self.__call_service('/niryo_robot_poses_handlers/manage_pose',
                                     ManagePose, req)
        return self.__classic_return_w_check(result)

    def delete_pose(self, name):
        """
        Send delete command to the pose manager service

        :param name:
        :type name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManagePose, ManagePoseRequest

        req = ManagePoseRequest()
        req.cmd = ManagePoseRequest.DELETE
        req.pose.name = name
        result = self.__call_service('/niryo_robot_poses_handlers/manage_pose',
                                     ManagePose, req)
        return self.__classic_return_w_check(result)

    def get_saved_pose_list(self, with_desc=False):
        """
        Ask the pose manager service which positions are available

        :param with_desc: If it returns the poses descriptions
        :type with_desc: bool
        :return: list of positions name
        :rtype: list[str]
        """
        result = self.__call_service('/niryo_robot_poses_handlers/get_pose_list',
                                     GetNameDescriptionList)
        if with_desc:
            return result.name_list, result.description_list

        return result.name_list

    # - Pick/Place

    def pick_from_pose(self, x, y, z, roll, pitch, yaw):
        """
        Execute a picking from a position. If an error happens during the movement, error will be raised.
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
        :return: status, message
        :rtype: (int, str)
       """
        self.release_with_tool()

        self.move_pose(x, y, z + 0.05, roll, pitch, yaw)
        self.move_pose(x, y, z, roll, pitch, yaw)

        self.grasp_with_tool()

        return self.move_pose(x, y, z + 0.05, roll, pitch, yaw)

    def place_from_pose(self, x, y, z, roll, pitch, yaw):
        """
        Execute a placing from a position. If an error happens during the movement, error will be raised.
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
        :return: status, message
        :rtype: (int, str)
        """
        self.move_pose(x, y, z + 0.05, roll, pitch, yaw)
        self.move_pose(x, y, z, roll, pitch, yaw)

        self.release_with_tool()

        return self.move_pose(x, y, z + 0.05, roll, pitch, yaw)

    def pick_and_place(self, pick_pose, place_pose, dist_smoothing=0.0):
        """
        Execute a pick and place. If an error happens during the movement, error will be raised.
        -> Args param is for development purposes

        :param pick_pose:
        :type pick_pose: list[float]
        :param place_pose:
        :type place_pose: list[float]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float
        :return: status, message
        :rtype: (int, str)
        """
        self.release_with_tool()

        pick_pose_high = self.copy_position_with_offsets(pick_pose, z_offset=0.05)
        place_pose_high = self.copy_position_with_offsets(place_pose, z_offset=0.05)

        self.execute_trajectory_from_poses([pick_pose_high, pick_pose], dist_smoothing)
        self.grasp_with_tool()

        self.execute_trajectory_from_poses([pick_pose_high, place_pose_high, place_pose], dist_smoothing)

        self.release_with_tool()

        return self.move_pose(*place_pose_high)

    # - Trajectories

    def get_trajectory_saved(self, trajectory_name):
        """
        Get saved trajectory from robot intern storage
        Will raise error if position does not exist

        :param trajectory_name:
        :type trajectory_name: str
        :raises NiryoRosWrapperException: If trajectory file doesn't exist
        :return: list of [x, y, z, qx, qy, qz, qw]
        :rtype: list[list[float]]
        """
        from niryo_robot_poses_handlers.srv import GetTrajectory

        result = self.__call_service('/niryo_robot_poses_handlers/get_trajectory',
                                     GetTrajectory, trajectory_name)
        self.__check_result_status(result)
        list_poses = result.list_poses
        list_poses_raw = []
        for p in list_poses:
            pose_raw = [p.position.x, p.position.y, p.position.z,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
            list_poses_raw.append(pose_raw)
        return list_poses_raw

    def execute_trajectory_saved(self, trajectory_name):
        """
        Execute trajectory saved in Robot internal storage

        :param trajectory_name:
        :type trajectory_name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import GetTrajectory

        result = self.__call_service('/niryo_robot_poses_handlers/get_trajectory',
                                     GetTrajectory, trajectory_name)
        list_poses = result.list_poses
        return self.__execute_trajectory_from_formatted_poses(list_poses)

    def execute_trajectory_from_poses(self, list_poses_raw, dist_smoothing=0.0):
        """
        Execute trajectory from a list of pose

        :param list_poses_raw: list of [x, y, z, qx, qy, qz, qw] or list of [x, y, z, roll, pitch, yaw]
        :type list_poses_raw: list[list[float]]
        :param dist_smoothing: Distance from waypoints before smoothing trajectory
        :type dist_smoothing: float
        :return: status, message
        :rtype: (int, str)
        """

        if len(list_poses_raw) < 2:
            return "Give me at least 2 points"
        list_poses = self.__list_pose_raw_to_list_poses(list_poses_raw)
        return self.__execute_trajectory_from_formatted_poses(list_poses, dist_smoothing)

    def compute_trajectory_from_poses(self, list_poses_raw, dist_smoothing=0.0):
        from niryo_robot_arm_commander.srv import ComputeTrajectory

        if len(list_poses_raw) < 2:
            return "Give me at least 2 points"
        list_poses = self.__list_pose_raw_to_list_poses(list_poses_raw)

        result = self.__call_service("/niryo_robot_arm_commander/compute_waypointed_trajectory",
                                     ComputeTrajectory, list_poses, dist_smoothing)
        self.__classic_return_w_check(result)
        return result.trajectory

    def __list_pose_raw_to_list_poses(self, list_poses_raw):
        from niryo_robot_poses_handlers.transform_functions import quaternion_from_euler

        list_poses = []
        for pose in list_poses_raw:
            point = Point(*pose[:3])
            angle = pose[3:]
            if len(angle) == 3:
                quaternion = quaternion_from_euler(*angle)
            else:
                quaternion = angle
            orientation = Quaternion(*quaternion)
            list_poses.append(Pose(point, orientation))
        return list_poses

    def execute_trajectory_from_poses_and_joints(self, list_pose_joints, list_type=None, dist_smoothing=0.0):
        """
        Execute trajectory from list of poses and joints

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

        list_pose_waypoints = self.__list_pose_joints_to_list_poses(list_pose_joints, list_type)
        return self.execute_trajectory_from_poses(list_pose_waypoints, dist_smoothing)

    def compute_trajectory_from_poses_and_joints(self, list_pose_joints, list_type=None, dist_smoothing=0.0):
        list_pose_waypoints = self.__list_pose_joints_to_list_poses(list_pose_joints, list_type)
        return self.compute_trajectory_from_poses(list_pose_waypoints, dist_smoothing)

    def __list_pose_joints_to_list_poses(self, list_pose_joints, list_type=None):
        if list_type is None:
            list_type = ['pose']
        list_pose_waypoints = []

        if len(list_type) == 1:  # only one type of object
            if list_type[0] == "pose":  # every elem in list is a pose
                list_pose_waypoints = list_pose_joints
            elif list_type[0] == "joint":  # every elem in list is a joint
                list_pose_waypoints = [self.forward_kinematics(*joint) for joint in list_pose_joints]

            else:
                raise NiryoRosWrapperException(
                    "Execute trajectory from poses and joints - Wrong list_type argument : got " +
                    list_type[0] +
                    ", expected 'pose' or 'joint'")

        elif len(list_type) == len(list_pose_joints):
            # convert every joints to poses
            for target, type_ in zip(list_pose_joints, list_type):
                if type_ == 'joint':
                    pose_from_joint = self.forward_kinematics(*target)
                    list_pose_waypoints.append(pose_from_joint)
                elif type_ == 'pose':
                    list_pose_waypoints.append(target)
                else:
                    raise NiryoRosWrapperException(
                        'Execute trajectory from poses and joints - Wrong list_type argument at index ' + str(i) +
                        ' got ' + type_ + ", expected 'pose' or 'joint'")

        else:
            raise NiryoRosWrapperException(
                'Execute trajectory from poses and joints - List of waypoints (size ' + str(len(list_pose_joints)) +
                ') and list of type (size ' + str(len(list_type)) + ') must be the same size.')

        return list_pose_waypoints

    def save_trajectory(self, trajectory_name, list_poses_raw):
        """
        Save trajectory object and send it to the trajectory manager service

        :param trajectory_name: name which will have the trajectory
        :type trajectory_name: str
        :param list_poses_raw: list of [x, y, z, qx, qy, qz, qw] or list of [x, y, z, roll, pitch, yaw]
        :type list_poses_raw: list[list[float]]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageTrajectory, ManageTrajectoryRequest
        from niryo_robot_poses_handlers.transform_functions import quaternion_from_euler

        req = ManageTrajectoryRequest()
        req.cmd = ManageTrajectoryRequest.SAVE
        req.name = trajectory_name
        list_poses = []
        for pose in list_poses_raw:
            angle = pose[3:]
            if len(angle) == 3:
                quaternion = quaternion_from_euler(*angle)
            else:
                quaternion = angle
            orientation = Quaternion(*quaternion)
            list_poses.append(Pose(Point(*pose[:3]), orientation))
        req.poses = list_poses

        result = self.__call_service('/niryo_robot_poses_handlers/manage_trajectory',
                                     ManageTrajectory, req)
        return self.__classic_return_w_check(result)

    def __execute_trajectory_from_formatted_poses(self, list_poses, dist_smoothing=0.0):

        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.EXECUTE_TRAJ
        goal.cmd.list_poses = list_poses
        goal.cmd.dist_smoothing = dist_smoothing
        return self.__robot_action_nac.execute(goal)

    def execute_moveit_robot_trajectory(self, moveit_robot_trajectory):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.EXECUTE_FULL_TRAJ
        goal.cmd.trajectory = moveit_robot_trajectory
        return self.__robot_action_nac.execute(goal)

    def delete_trajectory(self, trajectory_name):
        """
        Send delete command to the trajectory manager service

        :param trajectory_name: name
        :type trajectory_name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageTrajectory, ManageTrajectoryRequest
        req = ManageTrajectoryRequest()
        req.cmd = ManageTrajectoryRequest.DELETE
        req.name = trajectory_name
        result = self.__call_service('/niryo_robot_poses_handlers/manage_trajectory',
                                     ManageTrajectory, req)
        return self.__classic_return_w_check(result)

    def get_saved_trajectory_list(self):
        """
        Ask the pose trajectory service which trajectories are available

        :return: list of trajectory name
        :rtype: list[str]
        """
        result = self.__call_service('/niryo_robot_poses_handlers/get_trajectory_list',
                                     GetNameDescriptionList)
        return result.name_list

    # - Useful Pose functions

    @staticmethod
    def copy_position_with_offsets(copied_pose, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        """
        Copy a position and add offset to some coordinates
        """
        new_pose = [v for v in copied_pose]  # Copying all values
        new_pose[0] += x_offset  # adjust x
        new_pose[1] += y_offset  # adjust y
        new_pose[2] += z_offset  # adjust z
        return new_pose

    @staticmethod
    def list_to_robot_state_msg(x, y, z, roll, pitch, yaw):
        """
        Translate (x, y, z, roll, pitch, yaw) to a RobotState Object
        """
        r = RobotState()
        r.position.x = x
        r.position.y = y
        r.position.z = z
        r.rpy.roll = roll
        r.rpy.pitch = pitch
        r.rpy.yaw = yaw
        return r

    @staticmethod
    def robot_state_msg_to_list(robot_state):
        """
        Translate a RobotState Object to (x, y, z, roll, pitch, yaw)
        """
        return (robot_state.position.x, robot_state.position.y, robot_state.position.z,
                robot_state.rpy.roll, robot_state.rpy.pitch, robot_state.rpy.yaw)

    # -- Tools

    def get_current_tool_id(self):
        """
        Use /niryo_robot_tools_commander/current_id  topic to get current tool id

        :return: Tool Id
        :rtype: ToolID
        """
        return self.__current_tool_id_ntv.value.data

    def update_tool(self):
        """
        Call service niryo_robot_tools_commander/update_tool to update tool

        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_tools_commander/update_tool', Trigger)
        return self.__classic_return_w_check(result)

    def grasp_with_tool(self, pin_id=""):
        """
        Grasp with the tool linked to tool_id.
        This action correspond to
        - Close gripper for Grippers
        - Pull Air for Vacuum pump
        - Activate for Electromagnet

        :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
        :type pin_id: PinID
        :return: status, message
        :rtype: (int, str)
        """
        tool_id = self.get_current_tool_id()

        if tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4):
            return self.close_gripper()
        elif tool_id == ToolID.VACUUM_PUMP_1:
            return self.pull_air_vacuum_pump()
        elif tool_id == ToolID.ELECTROMAGNET_1:
            return self.activate_electromagnet(pin_id)

    def release_with_tool(self, pin_id=""):
        """
        Release with the tool associated to tool_id.
        This action correspond to
        - Open gripper for Grippers
        - Push Air for Vacuum pump
        - Deactivate for Electromagnet

        :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
        :type pin_id: PinID
        :return: status, message
        :rtype: (int, str)
        """
        tool_id = self.get_current_tool_id()
        print(tool_id)

        if tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4):
            return self.open_gripper()
        elif tool_id == ToolID.VACUUM_PUMP_1:
            return self.push_air_vacuum_pump()
        elif tool_id == ToolID.ELECTROMAGNET_1:
            return self.deactivate_electromagnet(pin_id)

    # - Gripper
    def open_gripper(self, speed=500, max_torque_percentage=100, hold_torque_percentage=20):
        """
        Open gripper with a speed 'speed'

        :param speed: Default -> 500
        :type speed: int
        :param max_torque_percentage: Default -> 100
        :type max_torque_percentage: int
        :param hold_torque_percentage: Default -> 20
        :type hold_torque_percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_gripper(ToolCommand.OPEN_GRIPPER, speed, max_torque_percentage, hold_torque_percentage)

    def close_gripper(self, speed=500, max_torque_percentage=100, hold_torque_percentage=50):
        """
        Close gripper with a speed 'speed'

        :param speed: Default -> 500
        :type speed: int
        :param max_torque_percentage: Default -> 100
        :type max_torque_percentage: int
        :param hold_torque_percentage: Default -> 20
        :type hold_torque_percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_gripper(ToolCommand.CLOSE_GRIPPER, speed, max_torque_percentage, hold_torque_percentage)

    def __deal_with_gripper(self, command_int, speed=500, max_torque_percentage=100, hold_torque_percentage=100):
        goal = ToolGoal()
        goal.cmd.tool_id = self.get_current_tool_id()
        goal.cmd.cmd_type = command_int
        goal.cmd.max_torque_percentage = max_torque_percentage
        goal.cmd.hold_torque_percentage = hold_torque_percentage
        if command_int == ToolCommand.OPEN_GRIPPER:
            goal.cmd.speed = speed
        else:
            goal.cmd.speed = speed
        return self.__tool_action_nac.execute(goal)

    # - Vacuum
    def pull_air_vacuum_pump(self):
        """
        Pull air

        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_vacuum_pump(ToolCommand.PULL_AIR_VACUUM_PUMP)

    def push_air_vacuum_pump(self):
        """
        Pull air

        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_vacuum_pump(ToolCommand.PUSH_AIR_VACUUM_PUMP)

    def __deal_with_vacuum_pump(self, command_int):
        goal = ToolGoal()
        goal.cmd.tool_id = ToolID.VACUUM_PUMP_1
        goal.cmd.cmd_type = command_int

        return self.__tool_action_nac.execute(goal.goal)

    # - Electromagnet
    def setup_electromagnet(self, pin_id):
        """
        Setup electromagnet on pin

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_tools_commander/equip_electromagnet',
                                     SetInt, ToolID.ELECTROMAGNET_1)

        if result.status != CommandStatus.SUCCESS:
            return result.status, result.message

        return self.__deal_with_electromagnet(pin_id, ToolCommand.SETUP_DIGITAL_IO)

    def activate_electromagnet(self, pin_id):
        """
        Activate electromagnet associated to electromagnet_id on pin_id

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_electromagnet(pin_id, ToolCommand.ACTIVATE_DIGITAL_IO)

    def deactivate_electromagnet(self, pin_id):
        """
        Deactivate electromagnet associated to electromagnet_id on pin_id

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_electromagnet(pin_id, ToolCommand.DEACTIVATE_DIGITAL_IO)

    def __deal_with_electromagnet(self, pin_id, command_int):
        goal = ToolGoal()
        goal.cmd.tool_id = ToolID.ELECTROMAGNET_1
        goal.cmd.cmd_type = command_int
        goal.cmd.gpio = pin_id
        return self.__tool_action_nac.execute(goal)

    # - TCP
    def enable_tcp(self, enable=True):
        """
        Enables or disables the TCP function (Tool Center Point).
        If activation is requested, the last recorded TCP value will be applied.
        The default value depends on the gripper equipped.
        If deactivation is requested, the TCP will be coincident with the tool_link.

        :param enable: True to enable, False otherwise.
        :type enable: Bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_tools_commander/enable_tcp', SetBool, enable)
        return self.__classic_return_w_check(result)

    def set_tcp(self, x, y, z, roll, pitch, yaw):
        """
        Activates the TCP function (Tool Center Point)
        and defines the transformation between the tool_link frame and the TCP frame.

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
        from niryo_robot_tools_commander.srv import SetTCP, SetTCPRequest
        req = SetTCPRequest(position=Point(x, y, z), rpy=RPY(roll, pitch, yaw))
        result = self.__call_service('/niryo_robot_tools_commander/set_tcp', SetTCP, req)
        return self.__classic_return_w_check(result)

    def reset_tcp(self):
        """
        Reset the TCP (Tool Center Point) transformation.
        The TCP will be reset according to the tool equipped.

        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_tools_commander/reset_tcp', Trigger)
        return self.__classic_return_w_check(result)

    def tool_reboot(self):
        """
        Reboot the motor of the tool equipped. Useful when an Overload error occurs. (cf HardwareStatus)

        :return: success, message
        :rtype: (bool, str)
        """
        from std_srvs.srv import Trigger as StdTrigger
        result = self.__call_service('/niryo_robot/tools/reboot', StdTrigger)

        return result.success, result.message

    # - Hardware

    def get_simulation_mode(self):
        """
        The simulation mode
        """
        return self.__simulation_mode

    def set_pin_mode(self, pin_id, pin_mode):
        """
        Set pin number pin_id to mode pin_mode

        :param pin_id:
        :type pin_id: PinID
        :param pin_mode:
        :type pin_mode: PinMode
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_rpi.srv import SetIOMode
        result = self.__call_service('/niryo_robot_rpi/set_digital_io_mode', SetIOMode, pin_id, pin_mode)
        return self.__classic_return_w_check(result)

    def digital_write(self, pin_id, digital_state):
        """
        Set pin_id state to pin_state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :param digital_state:
        :type digital_state: Union[ PinState, bool]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_rpi.srv import SetDigitalIO
        result = self.__call_service('/niryo_robot_rpi/set_digital_io', SetDigitalIO, pin_id, digital_state)
        return self.__classic_return_w_check(result)

    def analog_write(self, pin_id, analog_state):
        """
        Set pin_id state to pin_state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :param analog_state:
        :type analog_state: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_rpi.srv import SetAnalogIO
        result = self.__call_service('/niryo_robot_rpi/set_analog_io', SetAnalogIO, pin_id, analog_state)
        return self.__classic_return_w_check(result)

    def digital_read(self, pin_id):
        """
        Read pin number pin_id and return its state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :return: state
        :rtype: PinState
        """
        from niryo_robot_rpi.srv import GetDigitalIO
        result = self.__call_service('/niryo_robot_rpi/get_digital_io', GetDigitalIO, pin_id)
        self.__check_result_status(result)
        return PinState.LOW if result.value == 0 else PinState.HIGH

    def analog_read(self, pin_id):
        """
        Read pin number pin_id and return its state

        :param pin_id: The name of the pin
        :type pin_id: Union[ PinID, str]
        :return: state
        :rtype: PinState
        """
        from niryo_robot_rpi.srv import GetAnalogIO
        result = self.__call_service('/niryo_robot_rpi/get_analog_io', GetAnalogIO, pin_id)
        self.__check_result_status(result)
        return result.value

    def get_digital_io_state(self):
        """
        Get Digital IO state : Names, modes, states

        :return: Infos contains in a IOsState object (see niryo_robot_msgs)
        :rtype: IOsState
        """
        return self.__digital_io_state_ntv.value

    def get_analog_io_state(self):
        return self.__analog_io_state_ntv.value

    def get_hardware_version(self):
        """
        Get the robot hardware version
        """
        return self.__hardware_version

    def get_hardware_status(self):
        """
        Get hardware status : Temperature, Hardware version, motors names & types ...

        :return: Infos contains in a HardwareStatus object (see niryo_robot_msgs)
        :rtype: HardwareStatus
        """
        return self.__hw_status_ntv.value

    def get_robot_status(self):
        msg = rospy.wait_for_message('/niryo_robot_status/robot_status', RobotStatus, 2)
        return msg

    def get_axis_limits(self):
        """
        Return the joints and positions min and max values

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
        Reboot the robots motors

        :raises NiryoRosWrapperException:
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_hardware_interface/reboot_motors', Trigger)
        return self.__classic_return_w_check(result)

    # - Conveyor

    def set_conveyor(self):
        """
        Scan for conveyor on can bus. If conveyor detected, return the conveyor ID

        :raises NiryoRosWrapperException:
        :return: ID
        :rtype: ConveyorID
        """
        from conveyor_interface.srv import SetConveyor, SetConveyorRequest
        req = SetConveyorRequest(cmd=SetConveyorRequest.ADD)
        result = self.__call_service('/niryo_robot/conveyor/ping_and_set_conveyor', SetConveyor, req)

        # If no new conveyor is detected, it should not crash
        if result.status in [CommandStatus.NO_CONVEYOR_LEFT, CommandStatus.NO_CONVEYOR_FOUND]:
            rospy.logwarn_throttle(1, 'ROS Wrapper - No new conveyor found')
        else:
            self.__check_result_status(result)
        return result.id

    def unset_conveyor(self, conveyor_id):
        """
        Remove specific conveyor.

        :param conveyor_id: Basically, ConveyorID.ONE or ConveyorID.TWO
        :type conveyor_id: ConveyorID
        :raises NiryoRosWrapperException:
        :return: status, message
        :rtype: (int, str)
        """
        from conveyor_interface.srv import SetConveyor, SetConveyorRequest
        req = SetConveyorRequest(cmd=SetConveyorRequest.REMOVE, id=conveyor_id)
        result = self.__call_service('/niryo_robot/conveyor/ping_and_set_conveyor', SetConveyor, req)
        return self.__classic_return_w_check(result)

    def control_conveyor(self, conveyor_id, bool_control_on, speed, direction):
        """
        Control conveyor associated to conveyor_id.
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
        from conveyor_interface.srv import ControlConveyor
        result = self.__call_service('/niryo_robot/conveyor/control_conveyor',
                                     ControlConveyor, conveyor_id, bool_control_on, speed, direction)
        return self.__classic_return_w_check(result)

    def get_conveyors_feedback(self):
        """
        Give conveyors feedback

        :return: List[ID, connection_state, running, speed, direction]
        :rtype: List(int, bool, bool, int, int)
        """
        fb = self.__conveyors_feedback_ntv.value
        return fb.conveyors

    # - Vision

    def get_compressed_image(self, with_seq=False):
        """
        Get last stream image in a compressed format

        :return: string containing a JPEG compressed image
        :rtype: str
        """
        compressed_img = self.__compressed_image_message_ntv.value

        if with_seq:
            return compressed_img.data, compressed_img.header.seq

        return compressed_img.data

    def set_brightness(self, brightness_factor):
        """
        Modify image brightness

        :param brightness_factor: How much to adjust the brightness. 0.5 will
            give a darkened image, 1 will give the original image while
            2 will enhance the brightness by a factor of 2.
        :type brightness_factor: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_vision.srv import SetImageParameter
        result = self.__call_service('/niryo_robot_vision/set_brightness', SetImageParameter, brightness_factor)
        return self.__classic_return_w_check(result)

    def set_contrast(self, contrast_factor):
        """
        Modify image contrast

        :param contrast_factor: While a factor of 1 gives original image.
            Making the factor towards 0 makes the image greyer, while factor>1 increases the contrast of the image.
        :type contrast_factor: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_vision.srv import SetImageParameter
        result = self.__call_service('/niryo_robot_vision/set_contrast', SetImageParameter, contrast_factor)
        return self.__classic_return_w_check(result)

    def set_saturation(self, saturation_factor):
        """
        Modify image saturation

        :param saturation_factor: How much to adjust the saturation. 0 will
            give a black and white image, 1 will give the original image while
            2 will enhance the saturation by a factor of 2.
        :type saturation_factor: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_vision.srv import SetImageParameter
        result = self.__call_service('/niryo_robot_vision/set_saturation', SetImageParameter, saturation_factor)
        return self.__classic_return_w_check(result)

    @staticmethod
    def get_image_parameters():
        """
        Get last stream image parameters: Brightness factor, Contrast factor, Saturation factor.

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
            img_param_msg = rospy.wait_for_message('/niryo_robot_vision/video_stream_parameters', ImageParameters,
                                                   timeout=5)
        except rospy.ROSException:
            raise NiryoRosWrapperException(
                "Could not get image parameters on the {} topic".format('/niryo_robot_vision/video_stream_parameters'))

        return img_param_msg.brightness_factor, img_param_msg.contrast_factor, img_param_msg.saturation_factor

    def get_target_pose_from_rel(self, workspace_name, height_offset, x_rel, y_rel, yaw_rel):
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
        :return: target_pose
        :rtype: RobotState
        """
        from niryo_robot_poses_handlers.srv import GetTargetPose, GetTargetPoseRequest

        result = self.__call_service('/niryo_robot_poses_handlers/get_target_pose', GetTargetPose,
                                     workspace_name, height_offset, x_rel, y_rel, yaw_rel)
        self.__check_result_status(result)
        return result.target_pose

    def get_target_pose_from_cam(self, workspace_name, height_offset, shape, color):
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
        :return: object_found, object_pose, object_shape, object_color
        :rtype: (bool, RobotState, str, str)
        """
        object_found, rel_pose, obj_shape, obj_color = self.detect_object(workspace_name, shape, color)
        if not object_found:
            return False, None, "", ""
        obj_pose = self.get_target_pose_from_rel(workspace_name, height_offset, rel_pose.x, rel_pose.y, rel_pose.yaw)
        return True, obj_pose, obj_shape, obj_color

    def vision_pick_w_obs_joints(self, workspace_name, height_offset, shape, color, observation_joints):
        """
        Move Joints to observation_joints, then execute a vision pick
        """
        self.move_joints(*observation_joints)
        return self.vision_pick(workspace_name, height_offset, shape, color)

    def vision_pick_w_obs_pose(self, workspace_name, height_offset, shape, color, observation_pose_list):
        """
        Move Pose to observation_pose, then execute a vision pick
        """
        self.move_pose(*observation_pose_list)
        return self.vision_pick(workspace_name, height_offset, shape, color)

    def vision_pick(self, workspace_name, height_offset, shape, color):
        """
        Picks the specified object from the workspace. This function has multiple phases:
        1. detect object using the camera
        2. prepare the current tool for picking
        3. approach the object
        4. move down to the correct picking pose
        5. actuate the current tool
        6. lift the object

        :param workspace_name: name of the workspace
        :type workspace_name: str
        :param height_offset: offset between the workspace and the target height
        :type height_offset: float
        :param shape: shape of the target
        :type shape: ObjectShape
        :param color: color of the target
        :type color: ObjectColor
        :return: object_found, object_shape, object_color
        :rtype: (bool, ObjectShape, ObjectColor)
        """
        object_found, rel_pose, obj_shape, obj_color = self.detect_object(workspace_name, shape, color)
        if not object_found:
            return False, "", ""

        pick_pose = self.get_target_pose_from_rel(
            workspace_name, height_offset, rel_pose.x, rel_pose.y, rel_pose.yaw)
        approach_pose = self.get_target_pose_from_rel(
            workspace_name, height_offset + 0.05, rel_pose.x, rel_pose.y, rel_pose.yaw)

        self.release_with_tool()

        self.move_pose(*self.robot_state_msg_to_list(approach_pose))
        self.move_pose(*self.robot_state_msg_to_list(pick_pose))

        self.grasp_with_tool()

        self.move_pose(*self.robot_state_msg_to_list(approach_pose))
        return True, obj_shape, obj_color

    def move_to_object(self, workspace, height_offset, shape, color):
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
        :return: object_found, object_shape, object_color
        :rtype: (bool, ObjectShape, ObjectColor)
        """
        obj_found, obj_pose, obj_shape, obj_color = self.get_target_pose_from_cam(
            workspace, height_offset, shape, color)
        if not obj_found:
            return False, "", ""
        self.move_pose(*self.robot_state_msg_to_list(obj_pose))
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
        :rtype: (bool, RobotState, str, str)
        """
        from niryo_robot_vision.srv import ObjDetection

        ratio = self.get_workspace_ratio(workspace_name)
        response = self.__call_service("/niryo_robot_vision/obj_detection_rel", ObjDetection,
                                       shape, color, ratio, False)
        if response.status == CommandStatus.SUCCESS:
            return True, response.obj_pose, response.obj_type, response.obj_color
        elif response.status == CommandStatus.MARKERS_NOT_FOUND:
            rospy.logwarn_throttle(1, 'ROS Wrapper - Markers Not Found')
        elif response.status == CommandStatus.VIDEO_STREAM_NOT_RUNNING:
            rospy.logwarn_throttle(1, 'Video Stream not running')
        return False, None, "", ""

    def get_camera_intrinsics(self):
        """
        Get calibration object: camera intrinsics, distortions coefficients

        :return: raw camera intrinsics, distortions coefficients
        :rtype: (list, list)
        """

        camera_intrinsics = self.__camera_intrinsics_message_ntv.value
        return camera_intrinsics.K, camera_intrinsics.D

    def get_debug_markers(self):
        from niryo_robot_vision.srv import DebugMarkers, DebugMarkersRequest
        result = self.__call_service('/niryo_robot_vision/debug_markers', DebugMarkers, DebugMarkersRequest())
        return result

    def get_debug_colors(self, color):
        from niryo_robot_vision.srv import DebugColorDetection, DebugColorDetectionRequest
        req = DebugColorDetectionRequest(color=color)
        result = self.__call_service('/niryo_robot_vision/debug_colors', DebugColorDetection, req)
        return result

    # - Workspace
    def save_workspace_from_poses(self, name, list_poses_raw):
        """
        Save workspace by giving the poses of the robot to point its 4 corners
        with the calibration Tip. Corners should be in the good order

        :param name: workspace name, max 30 char.
        :type name: str
        :param list_poses_raw: list of 4 corners pose
        :type list_poses_raw: list[list]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageWorkspace, ManageWorkspaceRequest

        list_poses = [self.list_to_robot_state_msg(*pose) for pose in list_poses_raw]
        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.SAVE
        if len(name) > 30:
            rospy.logwarn('ROS Wrapper - Workspace name is too long, using : %s instead', name[:30])
        req.workspace.name = name[:30]
        req.workspace.poses = list_poses
        result = self.__call_service('/niryo_robot_poses_handlers/manage_workspace',
                                     ManageWorkspace, req)
        return self.__classic_return_w_check(result)

    def save_workspace_from_points(self, name, list_points_raw):
        """
        Save workspace by giving the poses of its 4 corners in the good order

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
        result = self.__call_service('/niryo_robot_poses_handlers/manage_workspace',
                                     ManageWorkspace, req)
        return self.__classic_return_w_check(result)

    def delete_workspace(self, name):
        """
        Call workspace manager to delete a certain workspace

        :param name: workspace name
        :type name: str
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_poses_handlers.srv import ManageWorkspace, ManageWorkspaceRequest

        req = ManageWorkspaceRequest()
        req.cmd = ManageWorkspaceRequest.DELETE
        req.workspace.name = name
        result = self.__call_service('/niryo_robot_poses_handlers/manage_workspace',
                                     ManageWorkspace, req)
        return self.__classic_return_w_check(result)

    def get_workspace_poses(self, name):
        """
        Get the 4 workspace poses of the workspace called 'name'

        :param name: workspace name
        :type name: str
        :return: List of the 4 workspace poses
        :rtype: list[list]
        """
        from niryo_robot_poses_handlers.srv import GetWorkspaceRobotPoses

        result = self.__call_service('/niryo_robot_poses_handlers/get_workspace_poses', GetWorkspaceRobotPoses, name)
        self.__check_result_status(result)

        poses = result.poses
        list_p_raw = []
        for p in poses:
            pose = [p.position.x, p.position.y, p.position.z, p.rpy.roll, p.rpy.pitch, p.rpy.yaw]
            list_p_raw.append(pose)
        return list_p_raw

    def get_workspace_ratio(self, name):
        """
        Give the length over width ratio of a certain workspace

        :param name: workspace name
        :type name: str
        :return: ratio
        :rtype: float
        """
        from niryo_robot_poses_handlers.srv import GetWorkspaceRatio

        result = self.__call_service('/niryo_robot_poses_handlers/get_workspace_ratio',
                                     GetWorkspaceRatio, name)
        self.__check_result_status(result)
        return result.ratio

    def get_workspace_list(self, with_desc=False):
        """
        Ask the workspace manager service names of the available workspace

        :return: list of workspaces name
        :rtype: list[str]
        """
        result = self.__call_service('/niryo_robot_poses_handlers/get_workspace_list',
                                     GetNameDescriptionList)
        if with_desc:
            return result.name_list, result.description_list
        return result.name_list

    # - Software

    def get_software_version(self):
        """
        Get the robot software version

        :return: rpi_image_version, ros_niryo_robot_version, motor_names, stepper_firmware_versions
        :rtype: (str, str, list[str], list[str])
        """
        return self.__software_version_ntv.value

    @property
    def system_api_client(self):
        return self.__system_api_client

    # - Ned

    @property
    def database(self):
        return self.__database

    # - Ned 2

    @property
    def led_ring(self):
        return self.__led_ring

    @property
    def sound(self):
        return self.__sound

    @property
    def custom_button(self):
        return self.__custom_button
