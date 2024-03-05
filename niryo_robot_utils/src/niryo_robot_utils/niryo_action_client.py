#!/usr/bin/env python
# Lib
import rospy
import actionlib

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Actions
from actionlib_msgs.msg import GoalStatus

# Enums
from niryo_robot_utils.niryo_ros_wrapper_exceptions import NiryoRosWrapperException


class NiryoActionClient(object):

    def __init__(self, action_name, action_type, action_goal_type):

        self.__action_name = action_name
        self.__action_type = action_type
        self.__action_goal_type = action_goal_type

        self.__action_connection_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/action_connection_timeout")
        self.__action_execute_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/action_execute_timeout")
        self.__action_preempt_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/action_preempt_timeout")

        self.__action_server = None

    @property
    def name(self):
        return self.__action_name

    @property
    def empty_goal(self):
        return self.__action_goal_type()

    @property
    def get_empty_goal(self):
        return self.empty_goal

    @property
    def action_server(self):
        if self.__action_server is None:
            self.__action_server = actionlib.SimpleActionClient(self.__action_name, self.__action_type)
        return self.__action_server

    def execute(self, goal):
        if not isinstance(goal, self.__action_goal_type):
            raise NiryoRosWrapperException('Wrong goal type: expected {} but got {}'.format(
                type(goal), type(self.__action_goal_type)))

        if self.__action_server is None:
            self.__action_server = actionlib.SimpleActionClient(self.__action_name, self.__action_type)

        # Connect to server
        if not self.__action_server.wait_for_server(rospy.Duration(self.__action_connection_timeout)):
            rospy.logwarn("ROS Wrapper - Failed to connect to {} action server".format(self.__action_name))
            return CommandStatus.SUCCESS, ""

        # Send goal and check response
        goal_state, response = self.__send_goal_and_wait_for_completed(goal)

        if goal_state == CommandStatus.GOAL_TIMEOUT:
            return goal_state, response

        if response.status == CommandStatus.GOAL_STILL_ACTIVE:
            rospy.loginfo("ROS Wrapper - Command still active: try to stop it")
            self.__action_server.cancel_goal()
            self.__action_server.stop_tracking_goal()
            rospy.sleep(0.2)
            rospy.loginfo("ROS Wrapper - Trying to resend command ...")
            goal_state, response = self.__send_goal_and_wait_for_completed(goal)

        if goal_state != GoalStatus.SUCCEEDED:
            self.__action_server.stop_tracking_goal()

        if goal_state == GoalStatus.REJECTED:
            return CommandStatus.REJECTED, "Goal has been rejected : {}".format(response.message)
        elif goal_state == GoalStatus.ABORTED:
            return CommandStatus.ABORTED, "Goal has been aborted : {}".format(response.message)
        elif goal_state == GoalStatus.PREEMPTED:
            return CommandStatus.PREEMPTED, "Goal has been preempted: {}".format(response.message)
        elif goal_state != GoalStatus.SUCCEEDED:
            return CommandStatus.FAILURE, "Error when processing goal : {}".format(response.message)

        return response.status, response.message

    def __send_goal_and_wait_for_completed(self, goal):
        self.__action_server.send_goal(goal)
        if not self.__action_server.wait_for_result(timeout=rospy.Duration(self.__action_execute_timeout)):
            self.__action_server.cancel_goal()
            self.__action_server.stop_tracking_goal()
            return CommandStatus.GOAL_TIMEOUT, "Action Server timeout : {}".format(self.__action_name)

        goal_state = self.__action_server.get_state()
        response = self.__action_server.get_result()
        return goal_state, response
