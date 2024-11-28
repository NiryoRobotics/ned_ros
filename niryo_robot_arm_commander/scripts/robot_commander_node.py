#!/usr/bin/env python3

# Lib
import rospy
import logging
import actionlib
import threading

import sys
from niryo_robot_utils import sentry_init

# Commanders
from niryo_robot_arm_commander.arm_commander import ArmCommander
from niryo_robot_arm_commander.arm_state import ArmState
import moveit_commander

from niryo_robot_arm_commander.command_enums import RobotCommanderException, ArmCommanderException
from niryo_robot_poses_handlers.transform_functions import convert_legacy_rpy_to_dh_convention

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from niryo_robot_arm_commander.msg import ArmMoveCommand
from moveit_msgs.msg import RobotTrajectory

# Services
from niryo_robot_msgs.srv import Trigger, GetBool
from niryo_robot_arm_commander.srv import ComputeTrajectory

# Action msgs
from niryo_robot_arm_commander.msg import PausePlanExecution, RobotMoveAction, RobotMoveResult


class RobotCommanderNode:
    """
    This class is in charge of the Arm Commander Node
    It contains:
    - The State Publisher
    - Arm & Tools Commanders

    Its main goal is to interpret command, and launch execution according to this command
    """

    def __init__(self):
        rospy.logdebug("Arm Commander - Entering in Init")
        # Initialize MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        # - Load all the sub-commanders

        # Initialize Arm
        self.__arm_state = ArmState()
        self.__arm_commander = ArmCommander(self.__arm_state)

        rospy.logdebug("Arm Commander - Sub Commanders are loaded")

        # Dict which link MoveCommand to arm functions
        self.dict_interpreter_move_cmd = {
            # Move to One Pose
            ArmMoveCommand.JOINTS: self.__arm_commander.set_joint_target,
            ArmMoveCommand.POSE: self.__arm_commander.set_pose_target_from_cmd,
            ArmMoveCommand.POSITION: self.__arm_commander.set_position_target,
            ArmMoveCommand.RPY: self.__arm_commander.set_rpy_target,
            ArmMoveCommand.POSE_QUAT: self.__arm_commander.set_pose_quat_target,
            ArmMoveCommand.LINEAR_POSE: self.__arm_commander.set_linear_trajectory,
            ArmMoveCommand.SHIFT_POSE: self.__arm_commander.set_shift_pose_target,
            ArmMoveCommand.SHIFT_LINEAR_POSE: self.__arm_commander.set_shift_linear_pose_target,

            # Trajectory
            ArmMoveCommand.EXECUTE_TRAJ: self.__arm_commander.compute_and_execute_waypointed_trajectory,
            ArmMoveCommand.EXECUTE_RAW_TRAJ: self.__arm_commander.execute_raw_waypointed_trajectory,
            ArmMoveCommand.EXECUTE_FULL_TRAJ: self.__arm_commander.execute_waypointed_trajectory,

            # Add-Ons
            ArmMoveCommand.DRAW_SPIRAL: self.__arm_commander.draw_spiral_trajectory,
            ArmMoveCommand.DRAW_CIRCLE: self.__arm_commander.draw_circle_trajectory,
        }

        # - Subscribers
        self.__learning_mode_on = True
        rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.__callback_learning_mode)

        self.__pause_state = PausePlanExecution.STANDBY
        rospy.Subscriber('/niryo_robot_rpi/pause_state', PausePlanExecution, self.__callback_pause_movement)

        # Event which allows to timeout if pause take too long
        self.__pause_finished_event = threading.Event()
        self.__pause_finished_event.set()
        self.__pause_timeout = PausePlanExecution.PAUSE_TIMEOUT
        self.__command_still_active_max_tries = rospy.get_param("~command_still_active_max_tries")
        active_publish_rate_sec = rospy.get_param("~active_publish_rate_sec")

        rospy.logdebug("RobotCommanderNode.init - pause_timeout: %s", self.__pause_timeout)
        rospy.logdebug("RobotCommanderNode.init - command_still_active_max_tries: %s",
                       self.__command_still_active_max_tries)
        rospy.logdebug("RobotCommanderNode.init - active_publish_rate_sec: %s", active_publish_rate_sec)

        # - Services
        rospy.Service('~stop_command', Trigger, self.__callback_stop_command)
        rospy.Service('~is_active', GetBool, self.__callback_is_active)
        rospy.Service('~compute_waypointed_trajectory', ComputeTrajectory, self.__callback_compute_trajectory)

        # Robot Action Server
        self.__current_goal_handle = actionlib.ServerGoalHandle()
        self.__action_server = actionlib.ActionServer('~robot_action',
                                                      RobotMoveAction,
                                                      goal_cb=self.__callback_goal,
                                                      cancel_cb=self.__callback_cancel,
                                                      auto_start=False)
        self.__action_server_thread = threading.Thread()
        self.__action_server_lock = threading.Lock()
        # Starting Action server
        self.__start_action_server()

        rospy.logdebug("Arm Commander - Services & Actions server are created")

        # - Publisher
        self.__is_active_publisher = rospy.Publisher('~is_active', Bool, queue_size=5)
        rospy.Timer(rospy.Duration(active_publish_rate_sec), self.__publish_is_active)

        # Set a bool to mention this node is initialized
        rospy.set_param('~initialized', True)
        rospy.loginfo("Arm Commander - Started")

    def __start_action_server(self):
        self.__action_server.start()
        rospy.logdebug("Arm Commander - Action Server started")

    # -- CALLBACKS
    # - Subscribers
    def __publish_is_active(self, *_):
        msg = Bool()
        msg.data = self.__current_goal_is_active()
        try:
            self.__is_active_publisher.publish(msg)
        except rospy.ROSException:
            return

    # - Subscribers
    def __callback_learning_mode(self, msg):
        activate = msg.data
        if not self.__learning_mode_on and activate:
            self.__arm_commander.trajectories_executor.stop_arm()
        self.__learning_mode_on = activate

    # - Services
    def __callback_stop_command(self, _):
        self.__cancel_command()
        return CommandStatus.SUCCESS, "Command stopped"

    def __callback_is_active(self, _):
        return self.__current_goal_is_active()

    def __callback_pause_movement(self, msg):
        self.__pause_state = msg.state
        if msg.state == PausePlanExecution.PAUSE:
            rospy.loginfo("Arm Commander - Receive Set Pause Mode from button")
            self.__pause_finished_event.clear()
            self.__cancel_command()
        elif msg.state == PausePlanExecution.CANCEL:
            rospy.loginfo("Arm Commander - Receive Cancel Command from button")
            self.__pause_finished_event.set()
            self.__cancel_command()
        elif msg.state == PausePlanExecution.RESUME:
            self.__pause_finished_event.set()

    def __callback_compute_trajectory(self, req):
        try:
            status, message, plan = self.__arm_commander.compute_waypointed_trajectory(req)
        except ArmCommanderException as e:
            return e.status, e.message, None

        return status, message, RobotTrajectory() if plan is None else plan.joint_trajectory

    # - Action Server
    def __callback_goal(self, goal_handle):
        """
        This function verifies that all conditions are met in order to execute the request
        :param goal_handle: object use to communicate with the action server
        :return: None
        """
        rospy.loginfo("Commander Action Serv - Received goal. Check if can be executed")

        # Check if motor connection problem
        if not self.__arm_state.hardware_status.connection_up:
            result = self.create_result(CommandStatus.HARDWARE_NOT_OK,
                                        "Motor connection problem, you can't send a command now")
            goal_handle.set_rejected(result)
            return

        # Check if calibration is needed
        if self.__arm_state.hardware_status.calibration_needed:
            result = self.create_result(CommandStatus.CALIBRATION_NOT_DONE,
                                        "You need to calibrate the robot before sending a command")
            goal_handle.set_rejected(result)
            return

        # Check if calibration is in progress
        if self.__arm_state.hardware_status.calibration_in_progress:
            result = self.create_result(CommandStatus.CALIBRATION_NOT_DONE,
                                        "Calibration in progress, wait until it ends to send a command")
            goal_handle.set_rejected(result)
            return

        # Check if jog controller enabled
        if self.__arm_commander.jog_controller.is_enabled():
            result = self.create_result(CommandStatus.JOG_CONTROLLER_ENABLED,
                                        "You need to deactivate jog controller to execute a new command")
            goal_handle.set_rejected(result)
            return

        # check if still have a goal
        if self.__current_goal_is_active():
            # If still have a goal, wait a bit to be sure it's not goal is still active
            # due to concurrency issue
            for i in range(self.__command_still_active_max_tries):
                rospy.logwarn("Commander Action Serv - Current goal seems to be still active, "
                              "will retry {} time(s)".format(self.__command_still_active_max_tries - i))
                rospy.sleep(0.2)
                if not self.__current_goal_is_active():
                    break
            else:
                result = self.create_result(CommandStatus.GOAL_STILL_ACTIVE, "Current command is still active")
                goal_handle.set_rejected(result)
                return

        # Check if learning mode ON
        if self.__learning_mode_on:
            if not self.__arm_state.set_learning_mode(False):
                result = self.create_result(CommandStatus.LEARNING_MODE_ON, "Learning mode could not be deactivated")
                goal_handle.set_rejected(result)
                return

        # set accepted
        self.__current_goal_handle = goal_handle
        self.__current_goal_handle.set_accepted()
        rospy.loginfo("Commander Action Serv - Goal has been accepted")

        # Launch compute + execution in a new thread
        self.__action_server_thread = threading.Thread(target=self.__execute_goal_action,
                                                       name="worker_execute_goal_action")
        self.__action_server_thread.start()
        rospy.logdebug("Commander Action Serv - Executing command in a new thread")

    def __callback_cancel(self, goal_handle):
        rospy.loginfo("Commander Action Serv - Received cancel command")

        if goal_handle == self.__current_goal_handle:
            self.__cancel_current_command()
        else:
            rospy.logdebug("Commander Action Serv - No current goal, nothing to do")

    # -- EXECUTORS
    def __reset_pause_play_state(self):
        self.__pause_finished_event.set()
        self.__pause_state = PausePlanExecution.STANDBY

    def __current_goal_is_active(self):
        if not self.__current_goal_handle.goal:
            return False
        return self.__current_goal_handle.get_goal_status().status in [GoalStatus.ACTIVE, GoalStatus.PENDING]

    def __cancel_due_to_pause(self):
        # Check if plan is paused
        if not self.__pause_finished_event.wait(timeout=self.__pause_timeout):
            result = self.create_result(CommandStatus.PAUSE_TIMEOUT,
                                        "Goal has been paused since too long, cancelling it")
            self.__current_goal_handle.set_canceled(result=result)

            rospy.logwarn("Commander Action Serv - {}".format(result.message))
            return True

        if self.__pause_state == PausePlanExecution.CANCEL:
            result = self.create_result(CommandStatus.CANCEL_PAUSE, "Paused as been canceled")
            self.__current_goal_handle.set_canceled(result)
            rospy.loginfo("Commander Action Serv - Goal has been successfully canceled")
            return True

        return False

    def __execute_goal_action(self):
        """
        Threaded function which interpret and execute the command
        It waits until the action finished and set goal_handle according to the result
        :return: None
        """
        if self.__pause_state == PausePlanExecution.PAUSE:
            while not self.__pause_finished_event.wait(1):
                pass
            self.__pause_finished_event.clear()
            if self.__pause_state == PausePlanExecution.RESUME:
                rospy.loginfo("Commander Action Serv - Resuming goal action")
                return self.__execute_goal_action()

        try:
            cmd = self.__current_goal_handle.goal.goal.cmd

            (status, message) = self.__interpret_and_execute_command(cmd)
            response = self.create_result(status, message)
            result = response
        except (RobotCommanderException, ArmCommanderException) as e:
            result = self.create_result(e.status, e.message)
            response = None
            rospy.loginfo("Commander Action Serv - An exception was "
                          "thrown during command execution : {}".format(e.message))

        # Check response
        if not response:
            self.__current_goal_handle.set_aborted(result)
            rospy.logwarn("Commander Action Serv - Execution has been aborted")
        elif response.status == CommandStatus.SUCCESS:
            self.__current_goal_handle.set_succeeded(result)
            rospy.loginfo("Commander Action Serv - Goal has been set as succeeded")
        elif response.status == CommandStatus.STOPPED:
            self.__current_goal_handle.set_canceled(result)
            rospy.loginfo("Commander Action Serv - Goal has been successfully canceled")
        elif response.status == CommandStatus.CONTROLLER_PROBLEMS:
            self.__cancel_command()
            self.__current_goal_handle.set_aborted(result)
            rospy.logwarn("Commander Action Serv - Controller failed during execution : " + "Goal has been aborted.\n" +
                          "This is due to either a collision, or a motor unable to follow a given command" +
                          " (overload, extreme positions, ...)")
        else:
            self.__current_goal_handle.set_aborted(result)
            rospy.logwarn("Commander Action Serv - Unknown result, goal has been set as aborted")

    def __interpret_and_execute_command(self, cmd):
        """
        Take a Robot command, give it to Arm or Tools commander, and return the result
        :param cmd: ArmMoveCommand (see niryo_robot_msgs)
        :return: status, message
        """
        cmd_type = cmd.cmd_type
        if cmd.tcp_version != cmd.DH_CONVENTION:
            roll, pitch, yaw = convert_legacy_rpy_to_dh_convention(cmd.rpy.roll, cmd.rpy.pitch, cmd.rpy.yaw)
            cmd.rpy.roll = roll
            cmd.rpy.pitch = pitch
            cmd.rpy.yaw = yaw
        return self.dict_interpreter_move_cmd[cmd_type](cmd)

    def __cancel_command(self):
        self.__arm_commander.trajectories_executor.cancel_goal()  # Send a cancel signal to Moveit interface

    @staticmethod
    def create_result(status, message):
        """
        Create a RobotMoveResult object which can be send to the Action Server
        :param status:
        :param message:
        :return: RobotMoveResult object
        """
        result = RobotMoveResult()
        result.status = status
        result.message = message
        return result

    def __cancel_current_command(self):
        try:
            self.__cancel_command()
        except RobotCommanderException:
            rospy.logwarn("Commander Action Serv - Could not cancel current command ")

    @staticmethod
    def goal_to_cmd_type(goal_handle):
        return goal_handle.goal.goal.cmd.cmd_type


if __name__ == '__main__':
    sentry_init()

    rospy.init_node('niryo_robot_arm_commander', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = RobotCommanderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
