#!/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy
import actionlib
import math

# Services
from roscpp.srv import SetLoggerLevel, SetLoggerLevelRequest, GetLoggers
from niryo_robot_msgs.srv import Trigger, SetBool, SetInt
from std_srvs.srv import Empty, EmptyResponse
from niryo_robot_rpi.srv import LedBlinker, LedBlinkerRequest

# Messages
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint
from niryo_robot_msgs.msg import CommandStatus

CHECK_EQUAT_STATES_SENSIBILITY = 0.2


class MotordebugException(Exception):
    pass


class MotorDebug:
    def __init__(self):
        # - Action serv
        self.__action_server_name = rospy.get_param("~joint_controller_name") + "/follow_joint_trajectory"

        self.__traj_client = actionlib.SimpleActionClient(self.__action_server_name, FollowJointTrajectoryAction)
        self.__traj_client.wait_for_server()

        # - Log services
        self.__hardware_log_level_service_name = "/niryo_robot_hardware_interface/set_logger_level"
        self.__hardware_log_level_service = rospy.ServiceProxy(self.__hardware_log_level_service_name, SetLoggerLevel)
        self.__get_hardware_loggers_service_name = "/niryo_robot_hardware_interface/get_loggers"
        self.__get_hardware_loggers_service = rospy.ServiceProxy(self.__get_hardware_loggers_service_name, GetLoggers)

        self.__commander_log_level_service_name = "/niryo_robot_arm_commander/set_logger_level"
        self.__commander_log_level_service = rospy.ServiceProxy(self.__commander_log_level_service_name, SetLoggerLevel)

        # - Services
        self.__motor_debug_service_start = rospy.Service('~motor_debug_start', SetInt,
                                                         self.__callback_motor_debug_start)
        self.__motor_debug_service_stop = rospy.Service('~motor_debug_stop', Empty, self.__callback_motor_debug_stop)

        self.__motor_report_service_name = "/niryo_robot_hardware_interface/launch_motors_report"
        self.__motor_report_service = rospy.ServiceProxy(self.__motor_report_service_name, Trigger)
        self.__stop_motor_report_service_name = "/niryo_robot_hardware_interface/stop_motors_report"
        self.__stop_motor_report_service = rospy.ServiceProxy(self.__stop_motor_report_service_name, Trigger)

        self.__set_led_state_service = rospy.ServiceProxy('/niryo_robot_rpi/set_led_custom_blinker',
                                                          LedBlinker)

        self.__reset_controller_service = rospy.ServiceProxy('/niryo_robot/joints_interface/steppers_reset_controller',
                                                             Trigger)

        # - Subscribers
        self.__is_active_publisher = rospy.Subscriber('~is_active', Bool, self.__callback_commander_is_active)

        # - Params
        self._joint_limits = rospy.get_param("/niryo_robot/robot_command_validation/joint_limits")
        self._joint_names = sorted(self._joint_limits.keys())

        self.__is_running = False
        self.__robot = NiryoRosWrapper()

    def __callback_commander_is_active(self, msg):
        # if commander is active, stop motor debug
        if msg.data:
            self.__is_running = False

    def __callback_motor_debug_stop(self, _):
        self.__is_running = False
        self.__stop_motor_report_service()
        self.__cancel()
        # self._set_log_debug(False)
        self.__robot.set_learning_mode(True)
        self.__set_led_state_service(False, 0, 0, 0)
        return EmptyResponse()

    def __callback_motor_debug_start(self, req):
        if not self.__is_running:
            self.__run(req.value)
            return CommandStatus.SUCCESS, "Success"
        return CommandStatus.ABORTED, "Debug motor already started or commander is running"

    def __check_state(self):
        if not self.__is_running:
            rospy.logwarn("Motor Debug - Cancelled")
            self.__cancel()
            raise MotordebugException("Cancelled")

    def __cancel(self):
        self.__reset_controller_service()
        # very important delay to avoid unexpected issues from ros_control
        rospy.sleep(0.05)

    def __run(self, nb_loops):
        try:
            error_counter = 0
            self.__is_running = True
            self.__set_led_state_service(False, 0, 0, 0)
            if nb_loops < 1:
                # self._set_log_debug(True)
                self._display_hardware_params()
                status, message = self._scan_with_motor_report()
                self.__check_state()
                rospy.loginfo("Motor Debug - {}".format(message))

                if status != CommandStatus.SUCCESS:
                    error_counter += 1
            else:
                default_joint_pose = [0.0] * len(self._joint_names)
                self.__robot.calibrate_auto()
                self.__robot.set_learning_mode(False)
                rospy.sleep(1)

                rospy.loginfo("Motor Debug - Go to default pose")
                error_counter += 0 if self._play_trajectory(self._create_goal(default_joint_pose)) else 1
                for _ in range(nb_loops):
                    for joint_index, joint_name in enumerate(self._joint_names):
                        rospy.loginfo("Motor Debug - Test {} limits".format(joint_name))
                        for limit_type in ["min", "max"]:
                            if joint_index == 1 and limit_type == 'min':
                                continue
                            rospy.loginfo("Motor Debug - Go to {} limit of {}".format(limit_type, joint_name))
                            test_limit_joints = default_joint_pose[:]
                            test_limit_joints[joint_index] = self._joint_limits[joint_name][limit_type]
                            test_limit_joints[joint_index] -= math.copysign(0.1, test_limit_joints[joint_index])
                            error_counter += 0 if self._play_trajectory(self._create_goal(test_limit_joints),
                                                                        wait=2 if joint_index == 5 else 1) else 1

                            rospy.loginfo("Motor Debug - Return to default pose")
                            error_counter += 0 if self._play_trajectory(self._create_goal(default_joint_pose),
                                                                        wait=2 if joint_index == 5 else 1) else 1

                            self.__set_led_state_service(error_counter > 0, 5, LedBlinkerRequest.LED_WHITE, 60)
                    if error_counter > 0:
                        break
                else:
                    for _ in range(nb_loops):
                        self.__play_fun_poses()

                self._play_trajectory(self._create_goal([0.0, 0.50, -1.25, 0.0, 0.0, 0.0]))

            rospy.sleep(1)
            self.__robot.set_learning_mode(True)
            # self._set_log_debug(False)
            self.__is_running = False

            self.__set_led_state_service(error_counter > 0, 5, LedBlinkerRequest.LED_WHITE, 360)

        except:
            self.__is_running = False
            self.__robot.set_learning_mode(True)

    def __play_fun_poses(self):
        error_cpt = 0
        waypoints = [[0.16,  0.00, -0.75, -0.56,  0.60, -2.26], 
                     [2.25, -0.25, -0.90,  1.54, -1.70,  1.70], 
                     [1.40,  0.35, -0.34, -1.24, -1.23, -0.10],
                     [0.00,  0.60,  0.46, -1.55, -0.15,  2.50],
                     [-1.0,  0.00, -1.00, -1.70, -1.35, -0.14]]
        for wayoint in waypoints:
            error_cpt += self._play_trajectory(self._create_goal(wayoint))

        return error_cpt

    def _scan_with_motor_report(self):
        try:
            rospy.wait_for_service(self.__motor_report_service_name, 2)
        except rospy.ROSException:
            rospy.logwarn("Motor Debug - Report motor service is not connected")
            return CommandStatus.FAIL, "Report motor service is not connected"

        response = self.__motor_report_service()
        return response.status, response.message

    def _play_trajectory(self, goal, wait=1):
        self.__check_state()
        rospy.loginfo("Motor Debug - Waiting for the joint_trajectory_action server")
        self.__traj_client.wait_for_server()
        rospy.loginfo("joint_trajectory_action server is ready")

        # When to start the trajectory: 0.1s from now
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        rospy.loginfo("Send Goad")
        self.__traj_client.send_goal(goal)

        rospy.loginfo("Goal sent: is waiting for result")
        self.__traj_client.wait_for_result(timeout=rospy.Duration(6))

        result = self.__traj_client.get_result()
        if not result:
            rospy.loginfo("Trajectory command has reached timeout limit")
            return False

        rospy.loginfo("Trajectory finished")
        rospy.sleep(wait)
        self.__check_state()
        return self._check_if_goal_reached(goal)

    def _check_if_goal_reached(self, goal):
        current_joints = self.__robot.get_joints()
        target_joints = goal.trajectory.points[-1].positions

        rospy.loginfo("Target joints: {} \nCurrent joints: {}".format(target_joints, current_joints))
        for joint_index in range(len(current_joints)):
            if not (target_joints[joint_index] - CHECK_EQUAT_STATES_SENSIBILITY <= current_joints[joint_index] <=
                    target_joints[
                        joint_index] + CHECK_EQUAT_STATES_SENSIBILITY):
                rospy.logerr("Motor Debug - The joint targets were not properly reached")
                return False

        return True

    def _create_goal(self, joints_position, duration=2):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [JointTrajectoryPoint()]
        goal.trajectory.points[0].positions = joints_position
        goal.trajectory.points[0].velocities = [0.0] * len(self._joint_names)
        goal.trajectory.points[0].time_from_start = rospy.Duration(duration)
        return goal

    def _set_log_debug(self, debug_mode):
        # loggers_list = self.__get_hardware_loggers_service()
        for logger_name in ["ros.niryo_robot_hardware_interface", "ros.conveyor_interface", "ros.dynamixel_driver",
                            "ros.stepper_driver", "ros.tools_interface", "rosout"]:
            log_state = SetLoggerLevelRequest(logger=logger_name, level='DEBUG' if debug_mode else 'INFO')
            self.__hardware_log_level_service(log_state)

        for logger_name in ["rospy.rosout", "rosout"]:
            log_state = SetLoggerLevelRequest(logger=logger_name, level='DEBUG' if debug_mode else 'INFO')
            self.__commander_log_level_service(log_state)

    def _display_hardware_params(self):
        rospy.loginfo("Motor Debug - Hardware params" + '\n' + self._display_dict(
            rospy.get_param("/niryo_robot_hardware_interface")))

    def _display_dict(self, dict_to_display, prefix='  '):
        string_to_display = ""
        for key in sorted(dict_to_display.keys()):
            if isinstance(dict_to_display[key], dict):
                values_to_string = self._display_dict(dict_to_display[key], prefix=prefix + "  ")
                string_to_display += prefix + key + ":\n" + values_to_string
            else:
                string_to_display += "{}{}: {}\n".format(prefix, key, dict_to_display[key])

        return string_to_display
