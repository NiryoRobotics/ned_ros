#!/usr/bin/env python
"""
The Jog Controller allows to easily command the robot manually. It means that you can use
mouse input, Leap Motion, Wii Controller or even Image processing to calculate determine
where the robot should go
This controller can be used :
- directly by using ROS Topic | ROS Low Level (see the file client_jog_interface_mouse.py which shows how to use
a mouse to control the robot)
- with Python ROS Wrapper | "Medium" Level
- through the TCP server | High Level
"""

import rospy
import math
from threading import Lock
from niryo_robot_arm_commander.command_enums import ArmCommanderException

# Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import RobotState as RobotStateMoveIt
from moveit_msgs.msg import Constraints

from niryo_robot_msgs.msg import HardwareStatus
from niryo_robot_msgs.msg import RobotState
from niryo_robot_msgs.msg import CommandJog
from niryo_robot_msgs.msg import RPY

# Services
from niryo_robot_arm_commander.srv import JogShift, JogShiftRequest
from niryo_robot_msgs.srv import SetBool
from niryo_robot_msgs.srv import GetBool
from moveit_msgs.srv import GetStateValidity


class JogController:
    def __init__(self, arm, kinematics_handler, parameters_validator):

        # - Values Init
        self._shift_mode = None
        self.__collision_detected = False
        self._last_target_values = [0.0 for _ in range(6)]
        self._last_shift_values_cmd = None
        self._target_lock = Lock()
        self._target_values = None
        self.__joints_name = rospy.get_param("~joint_names")
        self._current_jogged_joints = []
        self.__error_tolerance_joint = rospy.get_param("~error_tolerance_joint")
        self.__time_without_jog_limit = rospy.get_param(
            "~time_without_jog_TCP_limit")  # jog disabled after one second for the jogTCP Niryo Studio

        # - Move It Commander / Get Arm MoveGroupCommander
        self.__arm = arm

        # Validation
        self.__parameters_validator = parameters_validator
        jog_limits = rospy.get_param("~jog_limits")
        self.__pose_translation_max = jog_limits["translation"]
        self.__pose_rotation_max = jog_limits["rotation"]
        self.__joints_rotation_max = jog_limits["joints"]

        # - Kinematics
        self.__kinematics_handler = kinematics_handler
        self._new_robot_state = RobotState()
        self._last_robot_state_published = RobotState()

        # - Publisher which publishes if JogController is enabled
        self._enabled = False
        self.__jog_errors_cpt = 0

        self._jog_enabled_publisher = rospy.Publisher('/niryo_robot/jog_interface/is_enabled',
                                                      Bool, queue_size=3)
        self._jog_errors_publisher = rospy.Publisher('/niryo_robot/jog_interface/errors',
                                                     String, queue_size=3)

        rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~jog_enable_publish_rate")),
                    self._publish_jog_enabled)

        self._check_disable_jog_timer = None
        self._last_command_timer = rospy.get_time()
        self._publish_jog_enabled()

        # - Direct publisher to joint controller
        self._joint_trajectory_publisher = rospy.Publisher(
            rospy.get_param("~joint_controller_name") + '/command', JointTrajectory,
            queue_size=3)

        # Publishing rate
        self._timer_rate = rospy.get_param("~jog_timer_rate_sec")
        self._publisher_joint_trajectory_timer = None

        # - Subscribers
        # - Joint controller state, used to check collisions
        rospy.Subscriber(rospy.get_param("~joint_controller_name") + '/state', JointTrajectoryControllerState,
                         self.__callback_joint_controller_state)

        self._joint_states = None
        rospy.Subscriber('/joint_states', JointState,
                         self.__callback_joint_states)

        self._robot_state = None
        rospy.Subscriber('/niryo_robot/robot_state', RobotState,
                         self.__callback_sub_robot_state)

        self.__learning_mode_on = None
        rospy.Subscriber('/niryo_robot/learning_mode/state', Bool,
                         self.__callback_sub_learning_mode)

        self.__hardware_status = None
        rospy.Subscriber('/niryo_robot_hardware_interface/hardware_status', HardwareStatus,
                         self.__callback_hardware_status)

        # topic used to jog pose via Niryo Studio, to avoid calling the service
        self._jog_command_ik = None
        rospy.Subscriber('/niryo_robot_arm_commander/send_jog_command_ik', CommandJog,
                         self.__callback_send_jog_command_ik, queue_size=20)

        # topic used to jog joints via Niryo Studio, to avoid calling the service
        rospy.Subscriber('/niryo_robot_arm_commander/send_jog_joints_command', CommandJog,
                         self.__callback_send_jog_joints_command, queue_size=10)

        # - Service
        # Service to enable Jog Controller
        rospy.Service('/niryo_robot/jog_interface/enable', SetBool, self.__callback_enable_jog)

        rospy.Service('/niryo_robot/jog_interface/jog_shift_commander', JogShift,
                      self.__callback_jog_commander)

        # Check joint validity service (used for self collisions checking)
        self.check_state_validity = rospy.ServiceProxy('check_state_validity', GetStateValidity)

    # - Callbacks

    def __callback_sub_robot_state(self, robot_state):
        self._robot_state = robot_state

    def __callback_joint_states(self, joint_states_msg):
        self._joint_states = joint_states_msg.position[:6]

    def __callback_sub_learning_mode(self, learning_mode):
        self.__learning_mode_on = learning_mode.data

    def __callback_hardware_status(self, msg):
        self.__hardware_status = msg

    def __callback_enable_jog(self, req):
        if req.value:
            return self.enable()
        else:
            return self.disable()

    def __callback_send_jog_command_ik(self, msg):
        # check if the learning mode is false and the robot is calibrated
        self.__check_before_use_jog()

        shift_mode = msg.cmd
        if shift_mode != self._shift_mode:
            self._reset_last_pub()
            self._shift_mode = shift_mode

        shift_command = list(msg.shift_values)
        shift_command[:3] = [min([v, math.copysign(self.__pose_translation_max, v)], key=abs) for v in
                             shift_command[:3]]
        shift_command[3:] = [min([v, math.copysign(self.__pose_rotation_max, v)], key=abs) for v in
                             shift_command[3:]]

        if self._last_shift_values_cmd != msg.shift_values:
            self._last_shift_values_cmd = msg.shift_values
            self.__jog_errors_cpt = 0

        try:
            success, potential_target_values = self._get_new_joints_w_ik(shift_command)
        except ArmCommanderException as e:
            return self.__publish_jog_error(e.status, "Error while validating pose : {}".format(e.message))
        if not success:
            return self.__publish_jog_error(CommandStatus.NO_PLAN_AVAILABLE,
                                            "Unable to find an invert kinematics for the target position")

        success, message = self.__validate_ik_joints(potential_target_values)
        return self.__publish_jog_error(CommandStatus.SUCCESS if success else CommandStatus.JOG_CONTROLLER_FAILURE,
                                        message)

    def __publish_jog_error(self, status, message):
        if status < CommandStatus.SUCCESS:
            self.__jog_errors_cpt += 1
            if self.__jog_errors_cpt == 3:
                self._jog_errors_publisher.publish(message)
        else:
            self.__jog_errors_cpt = 0

    def __callback_send_jog_joints_command(self, msg):
        # check if the learning mode is false and the robot is calibrated
        self.__check_before_use_jog()

        shift_mode = msg.cmd
        if shift_mode != self._shift_mode:
            self._reset_last_pub()
            self._shift_mode = shift_mode

        if self._last_shift_values_cmd != msg.shift_values:
            self._last_shift_values_cmd = msg.shift_values
            self.__jog_errors_cpt = 0

        shift_command = list(msg.shift_values)
        if shift_mode == JogShiftRequest.JOINTS_SHIFT:
            # Accumulate multiple commands if they come faster than the publish rate
            shift_command = [min([v, math.copysign(self.__joints_rotation_max, v)], key=abs) for v in shift_command]
            target_values = [actual + shift for actual, shift in
                             zip(self._last_target_values, shift_command)]

            # get index of the jogged joints in a list
            joints_to_jog = [i for i, value in enumerate(shift_command) if value != 0]

            target_values = self.__limit_params_joints(target_values)

            try:
                self.__validate_params_joints(target_values)  # validate joints according to joints limits
            except ArmCommanderException as e:
                message = "Jog Controller - Error while validating joint : {}".format(e.message)
                rospy.logwarn_throttle(1, message)
                return self.__publish_jog_error(CommandStatus.JOG_CONTROLLER_FAILURE, message)

            # validate target joints validity, based on collisions checking
            try:
                valid, link_colliding1, link_colliding2 = self.__check_joint_validity_moveit(target_values)
                if not valid:
                    return self.__publish_jog_error(
                        CommandStatus.JOG_CONTROLLER_FAILURE,
                        "Joints target unreachable because of collision between {} and {}".format(
                            link_colliding1, link_colliding2))
            except rospy.ServiceException as e:
                message = "Jog Controller - Error while validating joint : {}".format(e.message)
                rospy.logwarn_throttle(1, message)
                return self.__publish_jog_error(CommandStatus.JOG_CONTROLLER_FAILURE, message)

            if self.__collision_detected:
                return self.__publish_jog_error(CommandStatus.JOG_CONTROLLER_FAILURE, "Collision detected")

            self.set_target_values(target_values)
            self._current_jogged_joints = joints_to_jog
            self._new_robot_state = RobotState()
            return self.__publish_jog_error(CommandStatus.SUCCESS, "Success")

    def __callback_joint_controller_state(self, msg):
        # check if collision when jogging joints
        current_joints_error = msg.error.positions

        # this error tolerance is lower than the one in arm_commander bc the jog is much slower
        for error, tolerance in zip(current_joints_error, self.__error_tolerance_joint):
            if abs(error) > tolerance and self._enabled and self._shift_mode == JogShiftRequest.JOINTS_SHIFT:
                self.__collision_detected = True
                self.__set_learning_mode(True)
                abort_str = "Command has been aborted due to a collision or " \
                            "a motor not able to follow the given trajectory"
                rospy.logwarn(abort_str)
                self.disable()
                rospy.sleep(1)  # sleep so if the arrow in NS is still pressed, the jog wont re-start directly.
                return

        self.__collision_detected = False

    def __callback_jog_commander(self, msg):
        # check if the learning mode is false and the robot is calibrated
        self.__check_before_use_jog()

        shift_mode = msg.cmd
        if shift_mode != self._shift_mode:
            self._reset_last_pub()
            self._shift_mode = shift_mode
            rospy.logwarn("new")

        shift_command = list(msg.shift_values)
        if shift_mode == JogShiftRequest.POSE_SHIFT:
            shift_command[:3] = [min([v, math.copysign(self.__pose_translation_max, v)], key=abs) for v in
                                 shift_command[:3]]
            shift_command[3:] = [min([v, math.copysign(self.__pose_rotation_max, v)], key=abs) for v in
                                 shift_command[3:]]
            try:
                success, potential_target_values = self._get_new_joints_w_ik(shift_command)
            except ArmCommanderException as e:
                return e.status, "Error while validating pose : {}".format(e.message)
            if not success:
                return CommandStatus.NO_PLAN_AVAILABLE, "Unable to find on invert kinematics for the target position"
            else:
                self.set_target_values(potential_target_values)
        else:
            # Accumulate multiple commands if they come faster than the publish rate
            shift_command = [min([v, math.copysign(self.__joints_rotation_max, v)], key=abs) for v in shift_command]
            target_values = [actual + shift for actual, shift in
                             zip(self._last_target_values, shift_command)]

            target_values = self.__limit_params_joints(target_values)

            joints_to_jog = [i for i, value in enumerate(shift_command) if value != 0]

            try:
                self.__validate_params_joints(target_values)  # validate joints according to joints limits
            except ArmCommanderException as e:
                return e.status, "Error while validating joints : {}".format(e.message)

            # validate target joints validity, based on collisions checking
            try:
                valid, link_colliding1, link_colliding2 = self.__check_joint_validity_moveit(target_values)
                if not valid:
                    if link_colliding1 is not None and link_colliding2 is not None:
                        return CommandStatus.JOG_CONTROLLER_FAILURE, \
                               "Joints target unreachable because of collision between {} and {}".format(
                                   link_colliding1, link_colliding2)
                    else:
                        return CommandStatus.JOG_CONTROLLER_FAILURE, \
                               "Joints target unreachable because of collision between two parts of Ned"
            except rospy.ServiceException as e:
                return e, "Error while validating joint : {}".format(e.message)

            self._current_jogged_joints = joints_to_jog
            self.set_target_values(target_values)
            self._new_robot_state = RobotState()
        return CommandStatus.SUCCESS, "Command send"

    # - Publishers

    def _publish_jog_enabled(self, *_):
        msg = Bool()
        msg.data = self._enabled
        try:
            self._jog_enabled_publisher.publish(msg)
        except rospy.ROSException:
            return

    def _publish_joint_trajectory(self, *_):
        if not self._enabled:
            return

        target_values = self.get_target_values()
        if not target_values:
            return
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()

        point = JointTrajectoryPoint()

        if self._shift_mode == JogShiftRequest.JOINTS_SHIFT:
            # if jogging joint, we send a command with only one joint,
            # thanks to the allow_partial_joints_goal parameter,
            # so the other joints are not affected by the jog.
            joint_names = []
            positions = []

            for elem in self._current_jogged_joints:
                joint_names.append('joint_{}'.format(elem + 1))
                positions.append(target_values[elem])

            msg.joint_names = joint_names
            point.positions = positions

        else:
            msg.joint_names = self.__joints_name
            point.positions = target_values
        point.time_from_start = rospy.Duration(self._timer_rate)
        msg.points = [point]

        try:
            self._joint_trajectory_publisher.publish(msg)
        except rospy.ROSSerializationException:
            rospy.logerr(target_values)
            rospy.logerr(msg)

        # Reset Target
        self.set_target_values(None)
        self._current_jogged_joints = None

        # Save state for next time
        self._last_target_values = target_values
        self._last_robot_state_published = self._new_robot_state

    # - Setters & Getters
    def set_target_values(self, target_values):
        with self._target_lock:
            self._target_values = target_values

    def get_target_values(self):
        with self._target_lock:
            target_values = self._target_values[:] if self._target_values else None
        return target_values

    @staticmethod
    def can_be_enable():
        """
        Check if Jog Controller can be enabled
        Basically, it checks if commander is running

        :return: Bool indicating if Controller can be enabled
        :rtype: bool
        """
        try:
            rospy.wait_for_service('/niryo_robot_arm_commander/is_active')
            is_active_service = rospy.ServiceProxy('/niryo_robot_arm_commander/is_active', GetBool)
            response = is_active_service()
            return not response.value
        except rospy.ServiceException:
            return False

    def enable(self):
        """
        Enable jog controller if possible

        :return: status, message
        :rtype: (GoalStatus, str)
        """
        if not self.can_be_enable():
            msg_str = "Jog Controller - Wait for the end of command to enable Jog Controller"
            rospy.logwarn(msg_str)
            return CommandStatus.ABORTED, msg_str

        rospy.wait_for_message('/niryo_robot/robot_state', RobotState, timeout=2)
        rospy.wait_for_message('/joint_states', JointState, timeout=2)
        self._enabled = True
        self._reset_last_pub()
        self._last_command_timer = rospy.get_time()
        self._publisher_joint_trajectory_timer = rospy.Timer(rospy.Duration(self._timer_rate),
                                                             self._publish_joint_trajectory)
        self._check_disable_jog_timer = rospy.Timer(rospy.Duration(1.0 / rospy.get_param("~jog_enable_publish_rate")),
                                                    self._check_for_disable)
        msg_str = "Jog Controller - Enabled"
        rospy.loginfo(msg_str)
        return CommandStatus.SUCCESS, msg_str

    def disable(self):
        """
        Disable jog controller

        :return: status, message
        :rtype: (GoalStatus, str)
        """
        if not self._enabled:
            msg_str = "Jog Controller - Already Disabled"
            rospy.loginfo(msg_str)
            return CommandStatus.SUCCESS, msg_str

        self._enabled = False
        self._shift_mode = None
        self.__jog_errors_cpt = 0
        self.set_target_values(None)
        self._current_jogged_joints = None
        # Shutdown timer (Only launched when jog enabled)
        self._publisher_joint_trajectory_timer.shutdown()
        self._publisher_joint_trajectory_timer = None
        self._check_disable_jog_timer.shutdown()
        self._check_disable_jog_timer = None

        msg_str = "Jog Controller - Disabled"
        rospy.loginfo(msg_str)
        return CommandStatus.SUCCESS, msg_str

    def is_enabled(self):
        """
        Return the if Jog Controller is enable

        :return: True if enable else False
        :rtype: bool
        """
        return self._enabled

    # - Useful Functions
    def _check_for_disable(self, *_):
        if not self.is_enabled():
            return
        delta_time = rospy.get_time() - self._last_command_timer
        if delta_time > self.__time_without_jog_limit:
            self.disable()

    def _reset_last_pub(self):
        self._last_target_values = self._joint_states
        self._last_robot_state_published = self._robot_state
        self.__jog_errors_cpt = 0

    def _get_new_joints_w_ik(self, shift_command):
        quat_jog = quaternion_from_euler(shift_command[3], shift_command[4], shift_command[5])
        quat_target = quaternion_multiply(quat_jog, [self._last_robot_state_published.orientation.x,
                                                     self._last_robot_state_published.orientation.y,
                                                     self._last_robot_state_published.orientation.z,
                                                     self._last_robot_state_published.orientation.w])
        rpy_target = RPY(*euler_from_quaternion(quat_target))

        self._new_robot_state = RobotState()
        self._new_robot_state.position.x = self._last_robot_state_published.position.x + shift_command[0]
        self._new_robot_state.position.y = self._last_robot_state_published.position.y + shift_command[1]
        self._new_robot_state.position.z = self._last_robot_state_published.position.z + shift_command[2]
        self._new_robot_state.orientation = Quaternion(*quat_target)
        self._new_robot_state.rpy = rpy_target

        self.__validate_params_pose(self._new_robot_state)

        success, joints = self.__kinematics_handler.get_inverse_kinematics(
            Pose(self._new_robot_state.position, self._new_robot_state.orientation))
        return success, joints

    def __validate_params_pose(self, new_robot_state):
        self.__parameters_validator.validate_position(new_robot_state.position)
        self.__parameters_validator.validate_orientation(new_robot_state.rpy)

    def __validate_params_joints(self, joints):
        self.__parameters_validator.validate_joints(joints)

    def __check_joint_validity_moveit(self, joints):
        """
        Check target joint validity (no collision) with MoveIt. 
        :return: The target joint validity in a boolean, and the two links colliding if available
        """
        robot_state_target = RobotStateMoveIt()
        robot_state_target.joint_state.header.frame_id = "world"
        robot_state_target.joint_state.position = joints
        robot_state_target.joint_state.name = self.__joints_name
        group_name = self.__arm.get_name()
        null_constraints = Constraints()
        try:
            response = self.check_state_validity(robot_state_target, group_name, null_constraints)
            if not response.valid:
                if len(response.contacts) > 0:
                    rospy.logwarn('Jog Controller - Joints target unreachable because of collision between %s and %s',
                                  response.contacts[0].contact_body_1, response.contacts[0].contact_body_2)
                    return False, response.contacts[0].contact_body_1, response.contacts[0].contact_body_2
                else:  # didn't succeed to get the contacts on the real robot
                    rospy.logwarn_throttle(1, 'Jog Controller - Joints target unreachable because of '
                                              'collision between two parts of Ned')
                    return False, None, None
            else:
                return True, None, None

        except AttributeError as _e:  # maybe delete later, useful for the test on real robot when using the service
            return True, None, None

    def __limit_params_joints(self, joints):
        joints_limits = self.__parameters_validator.get_joints_limits()
        for j, joint_value in enumerate(joints):
            joints[j] = max(joints_limits[j].lower, min(joints_limits[j].upper, joint_value))
        return joints

    def __validate_ik_joints(self, joints):
        """
        Check if the list of joints values received from the inverse kinematic is usable.
        If a joint is bigger than 0.5 radian the list becomes None and a warning appears

        :return: List with the values from the inverse kinematic or None if the movement is too complicated
        :rtype: list
        """

        actual_joints = list(self._joint_states)
        for i in range(len(joints)):
            if abs(actual_joints[i] - joints[i]) > 0.5:
                error_str = "Jog Controller can't execute this command: {}".format(
                    [round(joint, 3) for joint in joints])
                rospy.logwarn_throttle(0.5, error_str)
                self.set_target_values(None)
                self._current_jogged_joints = None
                return False, "Unable to find an invert kinematics for the target position " \
                              "close enough to the current position"
        else:
            self.set_target_values(joints)
        return True, "Success"

    def __check_before_use_jog(self):
        """
        Check if the calibration has already been done on the robot and if the learning mode is off.

        :return: None
        :rtype: None
        """
        if self.__hardware_status.calibration_needed or self.__hardware_status.calibration_in_progress:
            return CommandStatus.ABORTED, "Cannot send command cause Jog because calibration is not done"
        if self.__learning_mode_on:
            try:
                self.__set_learning_mode(False)
                rospy.sleep(0.1)
            except (rospy.ROSException, rospy.ServiceException):
                return CommandStatus.ABORTED, "Error while trying to turn Off learning mode"

        if not self._enabled:
            ret, str_msg = self.enable()
            if ret == CommandStatus.ABORTED:
                return CommandStatus.ABORTED, "Cannot send command cause Jog is not activated and cannot be"
        self._last_command_timer = rospy.get_time()

    @staticmethod
    def __set_learning_mode(set_bool):
        """
        Activate or deactivate the learning mode using the ros service /niryo_robot/learning_mode/activate

        :param set_bool:
        :type set_bool: bool

        :return: Success if the learning mode was properly activate or deactivate, False if not
        :rtype: bool
        """
        try:
            rospy.wait_for_service('/niryo_robot/learning_mode/activate', timeout=1)
            srv = rospy.ServiceProxy('/niryo_robot/learning_mode/activate', SetBool)
            resp = srv(set_bool)
            return resp.status == CommandStatus.SUCCESS
        except (rospy.ServiceException, rospy.ROSException):
            return False


if __name__ == '__main__':
    pass
