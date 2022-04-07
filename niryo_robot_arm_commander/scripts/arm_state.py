#!/usr/bin/env python

import rospy
import moveit_commander

# - Messages
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import JointState
from niryo_robot_msgs.msg import RobotState, HardwareStatus, CommandStatus

# - Services
from moveit_msgs.srv import GetStateValidity
from niryo_robot_msgs.srv import SetBool, SetInt, SetFloat

# - Classes
from transform_handler import ArmTCPTransformHandler
from ArmParametersValidator import ArmParametersValidator
from kinematics_handler import KinematicsHandler
from robot_state_publisher import StatePublisher

# - Enums
from niryo_robot_arm_commander.command_enums import ArmCommanderException


class ArmState(object):

    def __init__(self):
        self.__hardware_version = rospy.get_param('~hardware_version')

        self.__joints_name = rospy.get_param("~joint_names")
        rospy.logdebug("ArmCommander.init - joint_controller_name: %s", self.__joints_name)

        # Getting reference frame from robot_commander.launch
        self.__reference_frame = rospy.get_param("~reference_frame")
        rospy.logdebug("ArmCommander.init - reference_frame: %s", self.__reference_frame)

        self.__arm = None
        self.__acceleration_scaling_factor = 1.0
        self.__velocity_scaling_factor = 1.0
        self.__velocity_percentage_scaling_factor = 100.0

        # specific values for ned2
        if self.__hardware_version == 'ned2':
            self.__velocity_scaling_factor = 0.5
            self.__velocity_percentage_scaling_factor = 200.0

        # Arm velocity
        self.__max_velocity_scaling_percentage = (self.__velocity_scaling_factor *
                                                  self.__velocity_percentage_scaling_factor)
        self.__max_velocity_scaling_factor_pub = rospy.Publisher(
            '/niryo_robot/max_velocity_scaling_factor', Int32, queue_size=10, latch=True)
        self.__publish_velocity_scaling_percentage()
        rospy.Service('/niryo_robot_arm_commander/set_max_velocity_scaling_factor', SetInt,
                      self.__callback_set_max_velocity_scaling_factor)

        rospy.Service('/niryo_robot_arm_commander/set_acceleration_factor', SetFloat,
                      self.__callback_set_acceleration_factor)

        # Joint State
        self.__joint_states = None
        rospy.Subscriber('/joint_states', JointState, self.__callback_joint_states)

        # Check joint validity service (used for self collisions checking)
        self.check_state_validity = rospy.ServiceProxy('check_state_validity', GetStateValidity)

        self.__learning_mode_on = None
        rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.__callback_sub_learning_mode)
        self.__learning_mode_service = rospy.ServiceProxy("/niryo_robot/learning_mode/activate", SetBool)

        self.__hardware_status = rospy.wait_for_message('/niryo_robot_hardware_interface/hardware_status',
                                                        HardwareStatus, timeout=20)
        rospy.Subscriber('/niryo_robot_hardware_interface/hardware_status', HardwareStatus,
                         self.__callback_hardware_status)

        self.__robot_state = None

        # Init move group
        self.__init_move_group_commander()

        # First, get Parameters Validator for Arm
        self.__arm_param_validator = ArmParametersValidator(rospy.get_param("/niryo_robot/robot_command_validation"))

        # Transform handler
        self.__transform_handler = ArmTCPTransformHandler()

        # Publish robot state (position, orientation, tool)
        self.__state_publisher = StatePublisher(self)

        # Kinematics
        self.__kinematics = KinematicsHandler(self)

    @property
    def hardware_version(self):
        return self.__hardware_version

    @property
    def joints_name(self):
        return self.__joints_name

    @property
    def joint_states(self):
        return self.__joint_states

    @property
    def robot_state(self):
        return self.__robot_state

    @robot_state.setter
    def robot_state(self, robot_state):
        self.__robot_state = robot_state

    @property
    def learning_mode_on(self):
        return self.__learning_mode_on

    @property
    def hardware_status(self):
        return self.__hardware_status

    @property
    def transform_handler(self):
        return self.__transform_handler

    @property
    def parameters_validator(self):
        return self.__arm_param_validator

    @property
    def arm(self):
        return self.__arm

    @property
    def kinematics(self):
        return self.__kinematics

    @property
    def velocity_scaling_factor(self):
        return self.__velocity_scaling_factor

    @property
    def acceleration_scaling_factor(self):
        return self.__acceleration_scaling_factor

    # -- Callbacks
    def __callback_sub_robot_state(self, robot_state):
        self.__robot_state = robot_state

    def __callback_joint_states(self, joint_states_msg):
        self.__joint_states = list(joint_states_msg.position[:6])

    def __callback_sub_learning_mode(self, learning_mode):
        self.__learning_mode_on = learning_mode.data

    def __callback_hardware_status(self, msg):
        self.__hardware_status = msg

    def __callback_set_max_velocity_scaling_factor(self, req):
        rospy.logdebug("ArmCommander - __callback_set_max_velocity_scaling_factor: %d", req.value)

        if not 0 < req.value <= 200:
            return {'status': CommandStatus.INVALID_PARAMETERS, 'message': 'Value must be between 1 and 200'}

        if 100 < req.value <= 200:
            rospy.logwarn("ArmCommander - __callback_set_max_velocity_scaling_factor %d is above 100%%."
                          "You are now in the experimental mode of the robot", req.value)

        try:
            self.__set_max_velocity_scaling_percentage(req.value)
        except ArmCommanderException as e:
            return {'status': CommandStatus.ARM_COMMANDER_FAILURE, 'message': e.message}

        return {'status': CommandStatus.SUCCESS, 'message': 'Success'}

    def __callback_set_acceleration_factor(self, req):
        if not 0.1 <= req.value <= 1:
            return {'status': CommandStatus.INVALID_PARAMETERS, 'message': 'Value must be between 0.1 and 1'}

        self.__acceleration_scaling_factor = req.value
        self.__arm.set_max_acceleration_scaling_factor(self.__acceleration_scaling_factor)

        return {'status': CommandStatus.SUCCESS, 'message': 'Success'}

    # -- Publishers call
    def __publish_velocity_scaling_percentage(self):
        """
        Publish Integer between 1 and 100 which correspond to the function name :)
        :param _: TimeEvent object which is not used
        :return: None
        """
        rospy.logdebug("ArmCommander.init - Velocity_scaling_percentage: %d", self.__max_velocity_scaling_percentage)
        msg = Int32(data=self.__max_velocity_scaling_percentage)
        self.__max_velocity_scaling_factor_pub.publish(msg)

    # -- Core
    def __set_max_velocity_scaling_percentage(self, percentage):
        """
        Ask MoveIt to set the relative speed to (percentage)%
        :param percentage:
        :return: None
        """
        self.__max_velocity_scaling_percentage = percentage
        self.__velocity_scaling_factor = percentage / self.__velocity_percentage_scaling_factor
        self.__arm.set_max_velocity_scaling_factor(self.__velocity_scaling_factor)
        self.__publish_velocity_scaling_percentage()

    def set_learning_mode(self, set_bool):
        """
        Activate or deactivate the learning mode using the ros service /niryo_robot/learning_mode/activate

        :param set_bool:
        :type set_bool: bool

        :return: Success if the learning mode was properly activate or deactivate, False if not
        :rtype: bool
        """
        if set_bool and self.__hardware_version == 'ned2':
            return True
        try:
            return self.__learning_mode_service(set_bool).status == CommandStatus.SUCCESS
        except (rospy.ServiceException, rospy.ROSException):
            return False

    def force_learning_mode(self, set_bool):
        """
        Activate or deactivate the learning mode using the ros service /niryo_robot/learning_mode/activate

        :param set_bool:
        :type set_bool: bool

        :return: Success if the learning mode was properly activate or deactivate, False if not
        :rtype: bool
        """
        try:
            return self.__learning_mode_service(set_bool).status == CommandStatus.SUCCESS
        except (rospy.ServiceException, rospy.ROSException):
            return False

    def __init_move_group_commander(self):
        # Get Arm MoveGroupCommander
        move_group_commander_name = rospy.get_param("~move_group_commander_name")
        rospy.logdebug("ArmCommander.init - move_group_commander_name: %s", move_group_commander_name)

        self.__arm = moveit_commander.MoveGroupCommander(move_group_commander_name)

        # Set pose reference frame
        self.__arm.set_pose_reference_frame(self.__reference_frame)
        self.__arm.set_max_velocity_scaling_factor(self.__velocity_scaling_factor)
        self.__arm.set_max_acceleration_scaling_factor(self.__acceleration_scaling_factor)

        # Set planning parameters
        self.__arm.allow_replanning(rospy.get_param("~allow_replanning"))
        self.__arm.set_goal_joint_tolerance(rospy.get_param("~goal_joint_tolerance"))
        self.__arm.set_goal_position_tolerance(rospy.get_param("~goal_position_tolerance"))
        self.__arm.set_goal_orientation_tolerance(rospy.get_param("~goal_orientation_tolerance"))

        rospy.loginfo("Arm commander - MoveIt! successfully connected to move_group '{}'".format(self.__arm.get_name()))
        rospy.logdebug("Arm commander - MoveIt! will move '{}' in the planning_frame '{}'".format(
            self.__arm.get_end_effector_link(), self.__arm.get_planning_frame()))
