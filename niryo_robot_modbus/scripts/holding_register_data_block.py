#!/usr/bin/env python

import rospy
import actionlib
import threading

from data_block import DataBlock

# Messages
from actionlib_msgs.msg import GoalStatus
from niryo_robot_arm_commander.msg import RobotMoveAction
from niryo_robot_arm_commander.msg import RobotMoveGoal
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_tools_commander.msg import ToolCommand, ToolAction
from niryo_robot_arm_commander.msg import ArmMoveCommand

# Services
from conveyor_interface.srv import ControlConveyor, SetConveyor
from niryo_robot_msgs.srv import SetInt, SetBool
from niryo_robot_msgs.srv import Trigger

"""
 - Each address contains a 16 bits value
 - READ/WRITE registers

 --> Used to give commands to the robot 
 ( ! the stored values correspond to the last given command,
 not the current robot state !)
"""

HR_JOINTS = 0
HR_POSITION_X = 10
HR_POSITION_Y = 11
HR_POSITION_Z = 12
HR_ORIENTATION_X = 13
HR_ORIENTATION_Y = 14
HR_ORIENTATION_Z = 15

HR_MOVE_JOINTS_COMMAND = 100
HR_MOVE_POSE_COMMAND = 101
HR_MOVE_LINEAR_POSE_COMMAND = 102
HR_STOP_COMMAND = 110

# You should not write any value on those 3 addresses
# Only read to get info about command execution
HR_IS_EXECUTING_CMD = 150  # Is a command currently being executed (only for move/tool)
HR_LAST_ROBOT_CMD_RESULT = 151  # Get the status of the last command
HR_LAST_ROBOT_CMD_DATA_RESULT = 152  # Contains data retrieved from last cmd (depends of the cmd)

HR_LEARNING_MODE = 300

HR_NEW_CALIBRATION_REQUEST = 310
HR_START_AUTO_CALIBRATION = 311
HR_START_MANUAL_CALIBRATION = 312

HR_GRIPPER_OPEN_SPEED = 401
HR_GRIPPER_CLOSE_SPEED = 402

HR_SELECT_TOOL_FROM_ID = 500

HR_OPEN_GRIPPER = 510
HR_CLOSE_GRIPPER = 511
HR_PULL_AIR_VACUUM_PUMP = 512
HR_PUSH_AIR_VACUUM_PUMP = 513

# conveyor commands 
HR_PING_AND_SET_CONVEYOR = 520
HR_REMOVE_CONVEYOR_WITH_ID = 521
HR_CONTROL_CONVEYOR = 522
HR_CONTROL_CONVEYOR_DIRECTION = 523
HR_CONTROL_CONVEYOR_SPEED = 524
HR_CONTROL_CONVEYOR_ID = 525
HR_STOP_CONVEYOR_WITH_ID = 526


# Positive number : 0 - 32767
# Negative number : 32768 - 65535
def handle_negative_hr(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val


class HoldingRegisterDataBlock(DataBlock):

    def __init__(self):
        
        super(HoldingRegisterDataBlock, self).__init__()
        self.execution_thread = threading.Thread()
        self.is_action_client_running = False
        self.cmd_action_client = None

    def __set_command_in_progress(self):
        """
        Set a command as in progress and refresh the last result
        :return:
        :rtype:
        """
        self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [0])
        self.setValuesOffset(HR_IS_EXECUTING_CMD, [1])

    def __set_command_done(self, status_result):
        """
        Set a command as completed and store the result
        :param status_result: Status stored to HR_LAST_ROBOT_CMD_RESULT
        :type status_result:
        :return:
        :rtype:
        """
        self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [status_result])
        self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])

        # Override

    def setValues(self, address, values):
        self.process_command(address, values)
        super(HoldingRegisterDataBlock, self).setValues(address, values)

    def process_command(self, address, values):
        address -= 1
        if len(values) == 0:
            return

        if address == HR_LEARNING_MODE:
            self.activate_learning_mode(values[0])
        elif address == HR_MOVE_JOINTS_COMMAND:
            self.move_joints_command()
        elif address == HR_MOVE_POSE_COMMAND:
            self.move_pose_command()
        elif address == HR_MOVE_LINEAR_POSE_COMMAND:
            self.move_linear_pose_command()
        elif address == HR_STOP_COMMAND:
            self.stop_current_command()
        elif address == HR_NEW_CALIBRATION_REQUEST:
            self.request_new_calibration()
        elif address == HR_START_AUTO_CALIBRATION:
            self.start_auto_calibration()
        elif address == HR_START_MANUAL_CALIBRATION:
            self.start_manual_calibration()
        elif address == HR_SELECT_TOOL_FROM_ID:
            self.select_tool(values[0])
        elif address == HR_OPEN_GRIPPER:
            self.open_gripper_command(values[0])
        elif address == HR_CLOSE_GRIPPER:
            self.close_gripper_command(values[0])
        elif address == HR_PULL_AIR_VACUUM_PUMP:
            self.pull_air_vacuum_pump_command(values[0])
        elif address == HR_PUSH_AIR_VACUUM_PUMP:
            self.push_air_vacuum_pump_command(values[0])
        elif address == HR_PING_AND_SET_CONVEYOR:
            self.ping_and_set_conveyor()
        elif address == HR_REMOVE_CONVEYOR_WITH_ID:
            self.remove_conveyor_with_id()
        elif address == HR_CONTROL_CONVEYOR:
            self.control_conveyor()
        elif address == HR_STOP_CONVEYOR_WITH_ID:
            self.stop_conveyor()

    def request_new_calibration(self):
        self.call_ros_service('/niryo_robot/joints_interface/request_new_calibration', Trigger)

    def start_auto_calibration(self):
        self.__set_command_in_progress()
        response = self.call_ros_service('/niryo_robot/joints_interface/calibrate_motors', SetInt, 1)
        self.__set_command_done(response.status)

    def start_manual_calibration(self):
        self.call_ros_service('/niryo_robot/joints_interface/calibrate_motors', SetInt, 2)

    def activate_learning_mode(self, activate):
        activate = int(activate >= 1)
        self.call_ros_service('/niryo_robot/learning_mode/activate', SetBool, activate)

    def stop_current_command(self):
        if self.is_action_client_running:
            self.cmd_action_client.cancel_goal()

    def select_tool(self, tool_id):
        self.call_ros_service('/niryo_robot/update_tool', SetInt, int(tool_id))

    def open_gripper_command(self, tool_id):
        speed = self.getValuesOffset(HR_GRIPPER_OPEN_SPEED, 1)[0]
        if speed < 100:
            speed = 100
        elif speed > 1000:
            speed = 1000
        self.open_gripper(tool_id, speed)

    def close_gripper_command(self, tool_id):
        speed = self.getValuesOffset(HR_GRIPPER_CLOSE_SPEED, 1)[0]
        if speed < 100:
            speed = 100
        elif speed > 1000:
            speed = 1000
        self.close_gripper(tool_id, speed)

    def pull_air_vacuum_pump_command(self, tool_id):
        self.pull_air_vacuum_pump(tool_id)

    def push_air_vacuum_pump_command(self, tool_id):
        self.push_air_vacuum_pump(tool_id)

    def move_joints_command(self):
        joints_raw_values = self.getValuesOffset(HR_JOINTS, 6)
        joints = []
        for j in joints_raw_values:
            joints.append(handle_negative_hr(j) / 1000.0)
        self.move_joints(joints)

    def move_pose_command(self):
        pose_raw_values = self.getValuesOffset(HR_POSITION_X, 6)
        pose = []
        for p in pose_raw_values:
            pose.append(handle_negative_hr(p) / 1000.0)
        self.move_pose(pose)

    def move_linear_pose_command(self):
        pose_raw_values = self.getValuesOffset(HR_POSITION_X, 6)
        pose = []
        for p in pose_raw_values:
            pose.append(handle_negative_hr(p) / 1000.0)
        self.move_linear_pose(pose)

    def open_gripper(self, gripper_id, speed):
        goal = ToolActionGoal()
        goal.cmd.cmd_type = ToolCommand.OPEN_GRIPPER
        goal.cmd.tool_id = int(gripper_id)
        goal.cmd.gripper_open_speed = speed
        self.start_execution_thread_tool(goal)

    def close_gripper(self, gripper_id, speed):
        goal = ToolActionGoal()
        goal.cmd.cmd_type = ToolCommand.CLOSE_GRIPPER
        goal.cmd.tool_id = int(gripper_id)
        goal.cmd.gripper_close_speed = speed
        self.start_execution_thread_tool(goal)

    def pull_air_vacuum_pump(self, vacuum_pump_id):
        goal = ToolActionGoal()
        goal.cmd.cmd_type = ToolCommand.PULL_AIR_VACUUM_PUMP
        goal.cmd.tool_id = int(vacuum_pump_id)
        self.start_execution_thread_tool(goal)

    def push_air_vacuum_pump(self, vacuum_pump_id):
        goal = ToolActionGoal()
        goal.cmd.cmd_type = ToolCommand.PUSH_AIR_VACUUM_PUMP
        goal.cmd.tool_id = int(vacuum_pump_id)
        self.start_execution_thread_tool(goal)

    def move_pose(self, pose):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.POSE
        goal.cmd.position.x = pose[0]
        goal.cmd.position.y = pose[1]
        goal.cmd.position.z = pose[2]
        goal.cmd.rpy.roll = pose[3]
        goal.cmd.rpy.pitch = pose[4]
        goal.cmd.rpy.yaw = pose[5]
        self.start_execution_thread_arm(goal)

    def move_linear_pose(self, pose):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.LINEAR_POSE
        goal.cmd.position.x = pose[0]
        goal.cmd.position.y = pose[1]
        goal.cmd.position.z = pose[2]
        goal.cmd.rpy.roll = pose[3]
        goal.cmd.rpy.pitch = pose[4]
        goal.cmd.rpy.yaw = pose[5]
        self.start_execution_thread_arm(goal)

    def move_joints(self, joints):
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = ArmMoveCommand.JOINTS
        goal.cmd.joints = joints

        self.start_execution_thread_arm(goal)

    def start_execution_thread_arm(self, goal):
        if not self.execution_thread.is_alive():
            self.execution_thread = threading.Thread(target=self.execute_action,
                                                     args=['niryo_robot_arm_commander/robot_action', RobotMoveAction, goal]) 
            self.execution_thread.start()

    def start_execution_thread_tool(self, goal):
        if not self.execution_thread.is_alive():
            self.execution_thread = threading.Thread(target=self.execute_action,
                                                     args=['niryo_robot_tools_commander/action_server', ToolAction, goal]) 
            self.execution_thread.start()


    def execute_action(self, action_name, action_msg_type, goal):
        self.setValuesOffset(HR_IS_EXECUTING_CMD, [1])
        self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [0])
        self.cmd_action_client = actionlib.SimpleActionClient(action_name, action_msg_type)

        # Connect to server
        if not self.cmd_action_client.wait_for_server(rospy.Duration(0.5)):
            self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [7])
            return

        # Send goal and check response
        self.cmd_action_client.send_goal(goal)

        self.is_action_client_running = True
        if not self.cmd_action_client.wait_for_result(timeout=rospy.Duration(15.0)):
            self.cmd_action_client.cancel_goal()
            self.cmd_action_client.stop_tracking_goal()
            self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [6])
            self.is_action_client_running = False
            return

        self.is_action_client_running = False
        goal_state = self.cmd_action_client.get_state()

        if goal_state != GoalStatus.SUCCEEDED:
            self.cmd_action_client.stop_tracking_goal()

        self.setValuesOffset(HR_IS_EXECUTING_CMD, [0])

        if goal_state == GoalStatus.REJECTED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [2])
        elif goal_state == GoalStatus.ABORTED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [3])
        elif goal_state == GoalStatus.PREEMPTED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [4])
        elif goal_state != GoalStatus.SUCCEEDED:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [5])
        else:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_RESULT, [1])

    def ping_and_set_conveyor(self):
        """
        Scan for a conveyor connected and return it's ID (attribute a new ID internally)
        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum && HR_LAST_ROBOT_CMD_DATA_RESULT
        to the conveyor_id on result == CommandStatus.SUCCESS
        :rtype:
        """
        self.__set_command_in_progress()
        response = self.call_ros_service('/niryo_robot/conveyor/ping_and_set_conveyor',
                                         SetConveyor, 1, 0)
        self.__set_command_done(response.status)
        if response == CommandStatus.SUCCESS:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_DATA_RESULT, [response.id])

    def remove_conveyor_with_id(self):
        """
        Remove (unset) conveyor with ID given at HR_CONTROL_CONVEYOR_ID
        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum
        :rtype:
        """
        self.__set_command_in_progress()
        conveyor_id = self.getValuesOffset(HR_CONTROL_CONVEYOR_ID, 1)[0]
        response = self.call_ros_service('/niryo_robot/conveyor/ping_and_set_conveyor',
                                         SetConveyor, 2, conveyor_id)
        self.__set_command_done(response.status)

    def control_conveyor(self):
        """
        Control the conveyor with ID given at HR_CONTROL_CONVEYOR_ID with speed HR_CONTROL_CONVEYOR_SPEED
         and direction HR_CONTROL_CONVEYOR_DIRECTION
        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum
        :rtype:
        """
        self.__set_command_in_progress()
        conveyor_speed = self.getValuesOffset(HR_CONTROL_CONVEYOR_SPEED, 1)[0]
        if conveyor_speed > 100:
            conveyor_speed = 100
        elif conveyor_speed < 0:
            conveyor_speed = 0

        conveyor_direction = self.getValuesOffset(HR_CONTROL_CONVEYOR_DIRECTION, 1)[0]
        conveyor_direction = handle_negative_hr(conveyor_direction)

        conveyor_id = self.getValuesOffset(HR_CONTROL_CONVEYOR_ID, 1)[0]
        response = self.call_ros_service('/niryo_robot/conveyor/control_conveyor',
                                         ControlConveyor, conveyor_id, True, int(conveyor_speed), conveyor_direction)
        self.__set_command_done(response.status)

    def stop_conveyor(self):
        """
        Stop conveyor with ID given at HR_CONTROL_CONVEYOR_ID
        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum
        :rtype:
        """
        self.__set_command_in_progress()
        conveyor_id = self.getValuesOffset(HR_CONTROL_CONVEYOR_ID, 1)[0]
        response = self.call_ros_service('/niryo_robot/conveyor/control_conveyor',
                                         ControlConveyor, conveyor_id, False, 0, 1)
        self.__set_command_done(response.status)
