#!/usr/bin/env python

# Lib
import rospy
import moveit_commander

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from actionlib_msgs.msg import GoalStatus

# Services
from niryo_robot_msgs.srv import Trigger


class ToolCommanderSimu:
    """
    Object which command the Tool via MoveIt
    """

    def __init__(self):
        self.__reference_frame = rospy.get_param("~reference_frame")

        # Get Tool MoveGroupCommander
        self.__tool = moveit_commander.MoveGroupCommander("tool")

        # Set pose reference frame
        self.__tool.set_pose_reference_frame(self.__reference_frame)

        self.__set_gripper()

        list_commands_all_tools = rospy.get_param("/niryo_robot_tools/command_list")

        self.__open_gripper_cmd_id = list_commands_all_tools["open_gripper"]
        self.__close_gripper_cmd_id = list_commands_all_tools["close_gripper"]

    @staticmethod
    def __set_gripper():
        rospy.wait_for_service('/niryo_robot_tools/update_tool', 3)
        service = rospy.ServiceProxy('/niryo_robot_tools/update_tool', Trigger)
        result = service()
        if result.status != CommandStatus.SUCCESS:
            raise Exception(result.message)

    def send_tool_command(self, req):
        """
        Check niryo_robot_tools/config/end_effectors.yaml for cmd_signification
        """
        cmd = req.cmd_type
        if cmd == self.__open_gripper_cmd_id:
            rospy.loginfo("Tool Commander - Setting tool move group to open position")

            self.__tool.set_named_target("open")
            self.__tool.go()
        elif cmd == self.__close_gripper_cmd_id:
            rospy.loginfo("Tool Commander - Setting tool move group to close position")

            self.__tool.set_named_target("close")
            self.__tool.go()
        return CommandStatus.SUCCESS, "Tool Commander - Command has been successfully processed"

    def stop_tool_command(self):
        pass

    def get_command_status(self):
        pass

    def has_problem(self):
        return self.get_command_status() in [GoalStatus.ABORTED, GoalStatus.REJECTED]
