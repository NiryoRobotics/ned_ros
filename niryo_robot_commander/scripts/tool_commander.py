#!/usr/bin/env python
# Lib
import rospy
import actionlib

# Messages
from niryo_robot_tools.msg import ToolAction
from niryo_robot_tools.msg import ToolGoal

# Action msgs
from actionlib_msgs.msg import GoalStatus

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Exception
from niryo_robot_commander.command_enums import ToolCommanderException


class ToolCommander:
    """
    Class which allows interaction with Tool action server
    """

    def __init__(self):
        self.__tool_timeout = rospy.get_param("~simu_gripper")

        self.__action_client = actionlib.SimpleActionClient('/niryo_robot_tools/action_server', ToolAction)
        rospy.logdebug("Tool Commander - Waiting for action server : niryo_robot_tools/action_server...")
        self.__action_client.wait_for_server()
        rospy.logdebug("Tool Commander - Found action server : niryo_robot_tools/action_server")
        rospy.loginfo("Tool Commander - Started")

    def send_tool_command(self, cmd):
        """
        Send tool command and wait for goal transition to Done
        Finally, check if the command finished well
        :param cmd:
        :type cmd: ToolCommand
        :return: status, message
        """
        goal = self.create_goal(cmd)
        self.__action_client.send_goal(goal)
        rospy.loginfo("Tool Commander - Command sent")

        # Wait for goal transition to DONE
        bool_done = self.__action_client.wait_for_result(timeout=rospy.Duration(self.__tool_timeout))

        result = self.__action_client.get_result()
        if not result:
            rospy.loginfo("Tool Commander - Action Server's result is None")
            return CommandStatus.TOOL_FAILURE, "Tool command has reached timeout limit"

        message = "Tool Commander - " + result.message
        # If goal has been rejected/aborted, stop tracking it and return error
        if self.has_problem():
            self.__action_client.stop_tracking_goal()
            raise ToolCommanderException(CommandStatus.TOOL_FAILURE, message)

        if bool_done:
            rospy.loginfo(message)
            return CommandStatus.SUCCESS, "Tool command has been successfully processed"
        else:
            rospy.logwarn(message)
            return CommandStatus.TOOL_FAILURE, "Tool command has reached timeout limit"

    def stop_tool_command(self):
        self.__action_client.stop_tracking_goal()

    def get_command_status(self):
        """
        Returns LOST if this SimpleActionClient isn't tracking a goal. See documentation :
        http://docs.ros.org/melodic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html
        """
        return self.__action_client.get_state()

    def has_problem(self):
        return self.get_command_status() != GoalStatus.SUCCEEDED

    @staticmethod
    def create_goal(cmd):
        """
        Create ToolGoal
        :param cmd: ToolCommand
        :type cmd: ToolCommand
        :return: ToolGoal
        """
        goal = ToolGoal()
        goal.cmd = cmd
        return goal
