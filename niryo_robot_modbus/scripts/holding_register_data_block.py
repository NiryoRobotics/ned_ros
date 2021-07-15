#!/usr/bin/env python

import rospy
import actionlib
import threading

from data_block import DataBlock

import pymodbus
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian

# Enums
from niryo_robot_vision.enums import ObjectType, ColorHSV  # Type and color used for services request
from niryo_robot_modbus.enums import ColorRequest, ShapeRequest  # Shape and color used from modbus client

# Messages
from actionlib_msgs.msg import GoalStatus
from niryo_robot_arm_commander.msg import RobotMoveAction
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_msgs.msg import RobotState, ObjectPose
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from niryo_robot_tools_commander.msg import ToolCommand, ToolAction
from niryo_robot_arm_commander.msg import ArmMoveCommand


# Services
from conveyor_interface.srv import ControlConveyor, SetConveyor, SetConveyorRequest
from niryo_robot_msgs.srv import SetInt, SetBool
from niryo_robot_msgs.srv import Trigger
from niryo_robot_poses_handlers.srv import GetTargetPose, GetTargetPoseRequest
from niryo_robot_vision.srv import ObjDetection
from niryo_robot_poses_handlers.srv import GetWorkspaceRatio, ManageWorkspace, ManageWorkspaceRequest

# Actions
from niryo_robot_tools_commander.msg import ToolActionGoal
from niryo_robot_arm_commander.msg import RobotMoveActionGoal

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

# ---
# Only read to get info about command execution
HR_IS_EXECUTING_CMD = 150  # Is a command currently being executed (only for move/tool)
HR_LAST_ROBOT_CMD_RESULT = 151  # Get the status of the last command

# Contains data retrieved from last cmd (depends of the cmd)
HR_LAST_ROBOT_CMD_DATA_RESULT = 152  # store 1 number results.
HR_VISION_TARGET_POSE = 153  # stores the target pose result

HR_VISION_SHAPE_FOUND = 159  # stores the number (enum) of the shape found in the workspace
HR_VISION_COLOR_FOUND = 160  # stores the number (enum) of the color found in the workspace

# ---

HR_LEARNING_MODE = 300

HR_NEW_CALIBRATION_REQUEST = 310
HR_START_AUTO_CALIBRATION = 311
HR_START_MANUAL_CALIBRATION = 312

HR_GRIPPER_OPEN_SPEED = 401
HR_GRIPPER_CLOSE_SPEED = 402

HR_UPDATE_TOOL_ID = 500
HR_TOOL_ID = 501

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

# TCP
HR_ENABLE_TCP = 600
HR_SET_TCP = 601

# Vision commands
HR_GET_TARGET_POSE_FROM_REL = 610
HR_GET_TARGET_POSE_FROM_CAM = 611
HR_VISION_PICK = 612
HR_MOVE_TO_OBJECT = 613
HR_DETECT_OBJECT = 614

# Args used for vision function
HR_HEIGHT_OFFSET = 620
HR_X_REL = 621
HR_Y_REL = 622
HR_YAW_REL = 623

HR_SHAPE_REQUEST = 624
HR_COLOR_REQUEST = 625

 # Workspaces' names are limited to 30 characters, so the number of register to store it is 15 (2 char = 1 register)
HR_WORKSPACE_NAME = 626

# Use next registers at address 641.



# Positive number : 0 - 32767
# Negative number : 32768 - 65535
def handle_negative_hr(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val


def handle_negative_to_write_in_register(val):
    """
    Positive number : 0 - 32767
    Negative number : 32768 - 65535
    """
    if val < 0:
        val = (1 << 15) - val
    return val


class HoldingRegisterDataBlock(DataBlock):

    def __init__(self):

        super(HoldingRegisterDataBlock, self).__init__()
        self.execution_thread = threading.Thread()
        self.is_action_client_running = False
        self.cmd_action_client = None

        # Tool
        self.current_tool_id = None
        self.list_id_grippers = [11, 12, 13, 14] 
        rospy.Subscriber('/niryo_robot_hardware/tools/current_id', Int32,
                                                     self.sub_selected_tool_id)

        # Variables
        self.workspace_name = None
        self.vision_color = None
        self.vision_shape = None
        self.height_offset = None
        self.x_rel = None
        self.y_rel = None
        self.yaw_rel = None

        # List of enums names for color and type that are available in vision package
        self.list_enum_color = [color_obj.name for color_obj in ColorHSV]
        self.list_enum_type = [type_obj.name for type_obj in ObjectType]

        # List of enums values for color and shape that are available for modbus client
        self.list_enum_color_nb = [color_obj.value for color_obj in ColorRequest]
        self.list_enum_shape_nb = [shape_obj.value for shape_obj in ShapeRequest]

        self.null_robot_state = RobotState()


    def setValuesOffset(self, address, values):
        # if values is not a list, transform it to a list or it won't work
        if type(values) != list:
            values = [values] 
        # set values in register
        super(HoldingRegisterDataBlock, self).setValuesOffset(address,
                                                              values) 

    # - Callbacks

    def sub_selected_tool_id(self, msg):
        value = int(msg.data)
        self.current_tool_id = value

    # - Manages commands

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
        elif address == HR_UPDATE_TOOL_ID:
            self.update_tool()
        elif address == HR_OPEN_GRIPPER:
            self.open_gripper_command()
        elif address == HR_CLOSE_GRIPPER:
            self.close_gripper_command()
        elif address == HR_PULL_AIR_VACUUM_PUMP:
            self.pull_air_vacuum_pump()
        elif address == HR_PUSH_AIR_VACUUM_PUMP:
            self.push_air_vacuum_pump()
        elif address == HR_PING_AND_SET_CONVEYOR:
            self.ping_and_set_conveyor()
        elif address == HR_REMOVE_CONVEYOR_WITH_ID:
            self.remove_conveyor_with_id()
        elif address == HR_CONTROL_CONVEYOR:
            self.control_conveyor()
        elif address == HR_STOP_CONVEYOR_WITH_ID:
            self.stop_conveyor()
        elif address == HR_ENABLE_TCP:
            self.enable_tcp(values[0])
        elif address == HR_SET_TCP:
            self.set_tcp(values)

        # Vision related methods
        elif address == HR_GET_TARGET_POSE_FROM_REL:
            self.get_target_pose_from_rel()
        elif address == HR_GET_TARGET_POSE_FROM_CAM:
            self.get_target_pose_from_cam()
        elif address == HR_VISION_PICK:
            self.vision_pick()
        elif address == HR_MOVE_TO_OBJECT:
            self.move_to_object()
        elif address == HR_DETECT_OBJECT:
            self.detect_object()

        # information on color, shape, workspace name,.. used as argument for vision functions
        elif address == HR_WORKSPACE_NAME:
            self.update_workspace_name(values)
        elif address == HR_SHAPE_REQUEST:
            self.update_vision_shape(values[0])
        elif address == HR_COLOR_REQUEST:
            self.update_vision_color(values[0])
        elif address == HR_HEIGHT_OFFSET:
            self.update_height_offset(values[0])
        elif address == HR_X_REL:
            self.update_x_rel(values[0])
        elif address == HR_Y_REL:
            self.update_y_rel(values[0])
        elif address == HR_YAW_REL:
            self.update_yaw_rel(values[0])

    # - Methods definition

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

    def update_tool(self):
        response = self.call_ros_service('/niryo_robot_tools_commander/update_tool', Trigger)
        response_list = response.message.split(" : ")
        tool_id = int(response_list[1])
        self.setValuesOffset(HR_TOOL_ID, [tool_id])

    def open_gripper_command(self):
        speed = self.getValuesOffset(HR_GRIPPER_OPEN_SPEED, 1)[0]
        tool_id = self.getValuesOffset(HR_TOOL_ID, 1)[0]
        self.open_gripper(tool_id, speed)

    def close_gripper_command(self):
        speed = self.getValuesOffset(HR_GRIPPER_CLOSE_SPEED, 1)[0]
        tool_id = self.getValuesOffset(HR_TOOL_ID, 1)[0]
        self.close_gripper(tool_id, speed)

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
        if speed < 100:
            speed = 100
        elif speed > 1000:
            speed = 1000
        goal = ToolActionGoal()
        goal.goal.cmd.cmd_type = ToolCommand.OPEN_GRIPPER
        goal.goal.cmd.tool_id = int(gripper_id)
        goal.goal.cmd.gripper_open_speed = speed
        self.start_execution_thread_tool(goal.goal)

    def close_gripper(self, gripper_id, speed):
        if speed < 100:
            speed = 100
        elif speed > 1000:
            speed = 1000
        goal = ToolActionGoal()
        goal.goal.cmd.cmd_type = ToolCommand.CLOSE_GRIPPER
        goal.goal.cmd.tool_id = int(gripper_id)
        goal.goal.cmd.gripper_close_speed = speed
        self.start_execution_thread_tool(goal.goal)

    def pull_air_vacuum_pump(self):
        goal = ToolActionGoal()
        goal.goal.cmd.cmd_type = ToolCommand.PULL_AIR_VACUUM_PUMP
        goal.goal.cmd.tool_id = 31
        self.start_execution_thread_tool(goal.goal)

    def push_air_vacuum_pump(self):
        goal = ToolActionGoal()
        goal.goal.cmd.cmd_type = ToolCommand.PUSH_AIR_VACUUM_PUMP
        goal.goal.cmd.tool_id = 31
        self.start_execution_thread_tool(goal.goal)

    def move_pose(self, pose):
        goal = RobotMoveActionGoal() 
        goal.goal.cmd.cmd_type = ArmMoveCommand.POSE
        goal.goal.cmd.position.x = pose[0]
        goal.goal.cmd.position.y = pose[1]
        goal.goal.cmd.position.z = pose[2]
        goal.goal.cmd.rpy.roll = pose[3]
        goal.goal.cmd.rpy.pitch = pose[4]
        goal.goal.cmd.rpy.yaw = pose[5]
        self.start_execution_thread_arm(goal.goal)

    def move_linear_pose(self, pose):
        goal = RobotMoveActionGoal()
        goal.goal.cmd.cmd_type = ArmMoveCommand.LINEAR_POSE
        goal.goal.cmd.position.x = pose[0]
        goal.goal.cmd.position.y = pose[1]
        goal.goal.cmd.position.z = pose[2]
        goal.goal.cmd.rpy.roll = pose[3]
        goal.goal.cmd.rpy.pitch = pose[4]
        goal.goal.cmd.rpy.yaw = pose[5]
        self.start_execution_thread_arm(goal.goal)

    def move_joints(self, joints):
        goal = RobotMoveActionGoal()
        goal.goal.cmd.cmd_type = ArmMoveCommand.JOINTS
        goal.goal.cmd.joints = joints
        self.start_execution_thread_arm(goal.goal)

    def start_execution_thread_arm(self, goal):
        if not self.execution_thread.is_alive():
            self.execution_thread = threading.Thread(target=self.execute_action,
                                                     args=['niryo_robot_arm_commander/robot_action', RobotMoveAction,
                                                           goal])
            self.execution_thread.start()
            self.execution_thread.join()

    def start_execution_thread_tool(self, goal):
        if not self.execution_thread.is_alive():
            self.execution_thread = threading.Thread(target=self.execute_action,
                                                     args=['niryo_robot_tools_commander/action_server', ToolAction,
                                                           goal])
            self.execution_thread.start()
            self.execution_thread.join() # added thread.join to avoid errors when thread is long (caused issue in vision pick)

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
        req = SetConveyorRequest()
        req.cmd = SetConveyorRequest.ADD
        self.__set_command_in_progress()
        response = self.call_ros_service('/niryo_robot/conveyor/ping_and_set_conveyor',
                                         SetConveyor, req)
        self.__set_command_done(response.status)
        if response.status == CommandStatus.SUCCESS:
            self.setValuesOffset(HR_LAST_ROBOT_CMD_DATA_RESULT, [int(response.id)])
            self.setValuesOffset(HR_CONTROL_CONVEYOR_ID, [int(response.id)])

    def remove_conveyor_with_id(self):
        """
        Remove (unset) conveyor with ID given at HR_CONTROL_CONVEYOR_ID
        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum
        :rtype:
        """
        req = SetConveyorRequest()
        req.cmd = SetConveyorRequest.REMOVE
        self.__set_command_in_progress()
        req.id = self.getValuesOffset(HR_CONTROL_CONVEYOR_ID, 1)[0]
        response = self.call_ros_service('/niryo_robot/conveyor/ping_and_set_conveyor',
                                         SetConveyor, req)
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

    def enable_tcp(self, enable):
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
        response = self.call_ros_service('/niryo_robot_tools_commander/enable_tcp', SetBool, bool(enable))
        return self.__set_command_done(response.status)

    def set_tcp(self, tcp_rpy_raw_values):
        """
        Activates the TCP function (Tool Center Point)
        and defines the transformation between the tool_link frame and the TCP frame.

        :param tcp_rpy_raw_values: [x, y, z, roll, pitch, yaw] as raw values
        :type tcp_rpy_raw_values: list[float]
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_tools_commander.srv import SetTCP, SetTCPRequest
        from geometry_msgs.msg import Point

        tcp = [handle_negative_hr(axis) / 1000.0 for axis in tcp_rpy_raw_values]

        req = SetTCPRequest()
        req.position = Point(*tcp[:3])
        req.rpy.roll = tcp[3]
        req.rpy.pitch = tcp[4]
        req.rpy.yaw = tcp[5]

        response = self.call_ros_service('/niryo_robot_tools_commander/set_tcp', SetTCP, req)
        return self.__set_command_done(response.status)


    # - Vision related methods 

    def get_target_pose_from_rel(self, height_offset=None, x_rel=None, y_rel=None, yaw_rel=None, sub_called=False):
        """
        Given a pose (x_rel, y_rel, yaw_rel) relative to a workspace, this function
        returns the robot pose in which the current tool will be able to pick an object at this pose.
        The height_offset argument (in m) defines how high the tool will hover over the workspace. If height_offset = 0,
        the tool will nearly touch the workspace.

        :param height_offset: height offset to workspace
        :type height_offset: int
        :param x_rel: x component of the relative pose
        :type x_rel: float
        :param y_rel: y component of the relative pose
        :type y_rel: float
        :param yaw_rel: yaw component of the relative pose
        :type yaw_rel: float
        :param sub_called: True if the function is called by another method, False otherwise
        :type sub_called: bool
        :return: If sub_called is False :
                set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum and set HR_VISION_TARGET_POSE to the target pose
                Else :
                a CommandStatus and a robot pose
        :rtype: int, RobotState
        """

        # manage arguments, so we can pass another value in arg than class variable (usefull for vision_pick for instance)
        if height_offset is None:
            height_offset = self.height_offset
        if x_rel is None:
            x_rel = self.x_rel
        if y_rel is None:
            y_rel = self.y_rel
        if yaw_rel is None:
            yaw_rel = self.yaw_rel


        if any(elem is None for elem in [x_rel, y_rel, yaw_rel]):
            if not sub_called:
                self.setValuesOffset(HR_VISION_TARGET_POSE, [0] * 6)
                rospy.logwarn(
                    'Modbus Server - Impossible to get target pose : x_rel, y_rel or yaw_rel requested is None')
                return
            else:
                return CommandStatus.INVALID_PARAMETERS, self.null_robot_state

        if not sub_called:
            self.__set_command_in_progress()

        if x_rel == 0 and y_rel == 0 and yaw_rel == 0:
            # if the relative position is zero, it means it was previously set to zero because 
            # no object was detected in the workspace. The relative position corresponding to those 3 zero values 
            # is located on the first landmark of the workspace. If an object is placed there,
            # it won't be detected anyway, so we set the position result registers to 0.
            if not sub_called:
                self.setValuesOffset(HR_VISION_TARGET_POSE, [0] * 6)
                self.__set_command_done(CommandStatus.OBJECT_NOT_FOUND)
                return
            else:
                return CommandStatus.OBJECT_NOT_FOUND, self.null_robot_state

        # - Get target pose from relative pose of object
        response = self.call_ros_service('/niryo_robot_poses_handlers/get_target_pose', GetTargetPose,
                                         self.workspace_name,height_offset, x_rel, y_rel, yaw_rel)

        if response.status == CommandStatus.SUCCESS:
            if not sub_called:
                # store pose in result register and update higher_address_result 
                obj_pose = self.get_pose_for_registers_from_target_pose(response.target_pose)
                self.setValuesOffset(HR_VISION_TARGET_POSE, obj_pose)
                self.__set_command_done(response.status)
            else:
                return response.status, response.target_pose

        else:
            if not sub_called:
                self.setValuesOffset(HR_VISION_TARGET_POSE, [0] * 6)
                self.__set_command_done(response.status)
            else:
                return response.status, self.null_robot_state

    def get_target_pose_from_cam(self, sub_called=False):
        """
        First detects the specified object using the camera and then returns the robot pose in which the object can
        be picked with the current tool

        :param sub_called: True if the function is called by another method, False otherwise
        :type sub_called: bool
        :return: If sub_called is False :
                set HR_VISION_TARGET_POSE to target pose, set HR_VISION_SHAPE_FOUND to object's type, set
                HR_VISION_COLOR_FOUND to object's color and set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum
                Else :
                A CommandStatus, a boolean obj_found, a resulting robot pose, type, color of object

        :rtype: int, bool, RobotState, str, str
        """

        if not sub_called:
            self.__set_command_in_progress()

        # - Detect objects in workspace
        status_object_detection, obj_rel_pose, obj_type_str, obj_color_str = self.detect_object(sub_called=True)

        if status_object_detection != CommandStatus.SUCCESS:
            # no object detected
            if not sub_called:
                self.store_pose_shape_color_in_registers(obj_rel_pose, obj_type_str, obj_color_str)
                self.__set_command_done(status_object_detection)
                return
            else:
                return status_object_detection, False, self.null_robot_state, obj_type_str, obj_color_str

        # - Get target pose from detected object's relative pose
        response_status, target_pose = self.get_target_pose_from_rel(x_rel=obj_rel_pose.x, y_rel=obj_rel_pose.y,
                                                                     yaw_rel=obj_rel_pose.yaw, sub_called=True)
        if not sub_called:
            self.store_pose_shape_color_in_registers(target_pose, obj_type_str, obj_color_str)  # set registers
            self.__set_command_done(response_status)
        else:
            if response_status == CommandStatus.SUCCESS:
                return response_status, True, target_pose, obj_type_str, obj_color_str
            else:
                return response_status, True, self.null_robot_state, obj_type_str, obj_color_str

    def vision_pick(self):
        """
        Picks the specified object from the workspace. This function has multiple phases:
        1. detect object using the camera
        2. prepare the current tool for picking
        3. approach the object
        4. move down to the correct picking pose
        5. actuate the current tool
        6. lift the object
        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum, set HR_VISION_SHAPE_FOUND to object's type, set
                HR_VISION_COLOR_FOUND to object's color
        :rtype:
        """

        self.__set_command_in_progress()

        # - Detect object in the workspace
        status_object_detection, obj_rel_pose, obj_type_str, obj_color_str = self.detect_object(sub_called=True)

        if status_object_detection != CommandStatus.SUCCESS:
            # if no object found
            self.__set_command_done(status_object_detection)
            return

        # - Get target pose from relative pose
        pick_pose_response_status, pick_pose_response = self.get_target_pose_from_rel(x_rel=obj_rel_pose.x,
                                                                                      y_rel=obj_rel_pose.y,
                                                                                      yaw_rel=obj_rel_pose.yaw,
                                                                                      sub_called=True)

        approach_pose_response_status, approach_pose_response = self.get_target_pose_from_rel(
            height_offset=self.height_offset + 0.05, x_rel=obj_rel_pose.x,
            y_rel=obj_rel_pose.y, yaw_rel=obj_rel_pose.yaw, sub_called=True)

        if pick_pose_response_status == CommandStatus.SUCCESS and approach_pose_response_status == CommandStatus.SUCCESS:
            # - First store shape and color of object
            self.store_pose_shape_color_in_registers(None, obj_type_str, obj_color_str)

            # - Then approach the current object and grab it
            pick_pose = self.robot_state_msg_to_list(pick_pose_response)
            approach_pose = self.robot_state_msg_to_list(approach_pose_response)
            speed_tool = self.getValuesOffset(HR_GRIPPER_OPEN_SPEED, 1)[0]

            # - Release tool
            if self.current_tool_id in self.list_id_grippers:
                self.open_gripper(self.current_tool_id, speed_tool)
            elif self.current_tool_id == 31:  # vacuum pump
                self.push_air_vacuum_pump(self.current_tool_id)

            # - Move pose
            self.move_pose(approach_pose)
            self.move_pose(pick_pose)

            # - Grasp with tool
            if self.current_tool_id in self.list_id_grippers:
                self.close_gripper(self.current_tool_id, speed_tool)
            elif self.current_tool_id == 31:  # vacuum pump
                self.pull_air_vacuum_pump(self.current_tool_id)

            # - Lift the object
            self.move_pose(approach_pose)

            self.__set_command_done(pick_pose_response_status)
            return

        else:  # if the target pose could not be found
            self.store_pose_shape_color_in_registers(None, ShapeRequest.NONE.name, ColorRequest.NONE.name)
            if pick_pose_response_status != CommandStatus.SUCCESS:
                self.__set_command_done(pick_pose_response_status)
            else:
                self.__set_command_done(approach_pose_response_status)

    def move_to_object(self):
        """
        Same as `get_target_pose_from_cam` but directly moves to this position

        :return: set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum, set HR_VISION_SHAPE_FOUND to object's type, set
                HR_VISION_COLOR_FOUND to object's color
        :rtype:
        """
        self.__set_command_in_progress()

         # - Get target pose of object from camera
        response_status, obj_found, obj_pose, obj_type_str, obj_color_str = self.get_target_pose_from_cam(
            sub_called=True)

        if not obj_found:
            self.store_pose_shape_color_in_registers(None, ShapeRequest.NONE.name, ColorRequest.NONE.name)
            self.__set_command_done(response_status)
            return

        # - Move to this object
        self.move_pose(self.robot_state_msg_to_list(obj_pose))

        # - store shape and color in registers
        self.store_pose_shape_color_in_registers(None, obj_type_str, obj_color_str)

        self.__set_command_done(response_status)

    def detect_object(self, sub_called=False):
        """
        Find the first detected object according to requested shape and color. 

        :param sub_called: True if the function is called by another method, False otherwise
        :type sub_called: bool
        :return: if sub_called is False :
                set HR_LAST_ROBOT_CMD_RESULT to CommandStatus Enum,
                set HR_VISION_TARGET_POSE to object's relative pose,
                set HR_VISION_SHAPE_FOUND to object's type,
                set HR_VISION_COLOR_FOUND to object's color
                Else if sub_called is True, return :
                CommandStatus, object's pose, type and color
        :rtype:
        """

        # Check that workspace_name, vision_shape and vision_color are not None. If they are, empty registers.
        if any(elem is None for elem in [self.workspace_name, self.vision_shape, self.vision_color]):
            if not sub_called:
                self.store_pose_shape_color_in_registers(ObjectPose(*([0] * 6)), ShapeRequest.NONE.name, ColorRequest.NONE.name)
                rospy.logwarn(
                    'Modbus Server - Impossible to detect object : Workspace name, Shape or Color requested is None')
                return
            else:
                return CommandStatus.INVALID_PARAMETERS, ObjectPose(*([0] * 6)), ShapeRequest.NONE.name, ColorRequest.NONE.name

        if not sub_called:
            self.__set_command_in_progress()

        # Get workspace's ratio
        ratio_status, ratio = self.get_ws_ratio(self.workspace_name)
        if ratio_status != CommandStatus.SUCCESS:
            if not sub_called:
                self.__set_command_done(ratio_status)
                return
            else:
                return ratio_status, ObjectPose(*([0] * 6)), ShapeRequest.NONE.name, ColorRequest.NONE.name

        # - Detect object in the workspace
        response = self.call_ros_service("/niryo_robot_vision/obj_detection_rel", ObjDetection,
                                         self.vision_shape, self.vision_color, ratio, False)

        if response.status == CommandStatus.SUCCESS:
            if not sub_called:
                self.store_pose_shape_color_in_registers(response.obj_pose, response.obj_type, response.obj_color)
                self.__set_command_done(response.status)
                return
            else:
                return response.status, response.obj_pose, response.obj_type, response.obj_color

        elif response.status == CommandStatus.OBJECT_NOT_FOUND:
            if not sub_called:
                self.store_pose_shape_color_in_registers(response.obj_pose, ShapeRequest.NONE.name, ColorRequest.NONE.name)
            else:
                return response.status, response.obj_pose, ShapeRequest.NONE.name, ColorRequest.NONE.name

        elif response.status == CommandStatus.MARKERS_NOT_FOUND:
            rospy.logwarn_throttle(1, 'ROS Wrapper - Markers Not Found')
        elif response.status == CommandStatus.VIDEO_STREAM_NOT_RUNNING:
            rospy.logwarn_throttle(1, 'Video Stream not running')

        if not sub_called:
            self.__set_command_done(response.status)
        else:
            return response.status, response.obj_pose, ShapeRequest.NONE.name, ColorRequest.NONE.name

    def get_ws_ratio(self, values):
        """
        Give the length over width ratio of a certain workspace

        :param values: workspace name
        :type values: list[int]
        :return: CommandStatus, ratio
        :rtype: int, float
        """
        if not isinstance(values, str):
            ws_name = self.convert_register_to_string(values)
        else:
            ws_name = values

        response = self.call_ros_service('/niryo_robot_poses_handlers/get_workspace_ratio',
                                         GetWorkspaceRatio, ws_name)

        if response.status == CommandStatus.SUCCESS: 
            return response.status, response.ratio
        else:
            rospy.logwarn('Modbus Server - Impossible to get workspace ratio : %s', response.message)
            return response.status, response.ratio


    # - Functions called when a client stored a value in a register and we need to update the class variable accordingly

    def update_workspace_name(self, values):
        self.workspace_name = self.convert_register_to_string(values)


    def update_vision_shape(self, value):
        """
        Update self.vision_color according to the requested type.
        :param value: register values for type
        :type value: int
        """
        shape_nb = handle_negative_hr(value)  # shape nb is a int, not a float

        # Check that shape exists in enum. If not, reset self.vision_shape
        if shape_nb not in self.list_enum_shape_nb:
            self.vision_shape = None
            rospy.logwarn('Modbus Server - Shape requested is not available for Vision methods')
            return

        self.vision_shape = ShapeRequest(shape_nb).name

    def update_vision_color(self, value):
        """
        Update self.vision_color according to the requested color.
        :param value: register value for color
        :type value: int
        """
        color_nb = handle_negative_hr(value)  # shape nb is a int, not a float

        # Check that color exists in enum
        if color_nb not in self.list_enum_color_nb:
            self.vision_color = None
            rospy.logwarn('Modbus Server - Color requested is not available for Vision methods')
            return

        self.vision_color = ColorRequest(color_nb).name

    def update_height_offset(self, value):
        self.height_offset = handle_negative_hr(value) / 1000.0

    def update_x_rel(self, value):
        self.x_rel = handle_negative_hr(value) / 1000.0

    def update_y_rel(self, value):
        self.y_rel = handle_negative_hr(value) / 1000.0

    def update_yaw_rel(self, value):
        self.yaw_rel = handle_negative_hr(value) / 1000.0

    @staticmethod
    def update_pose(pose_obj, pose_value):
        """
        Update a RobotState pose given registers values
        :param pose_obj: object pose to update
        :type pose_obj: RobotState
        :param pose_value: new pose value from 16 bits int registers
        :type pose_value: list[int]
        """
        if len(pose_value) == 6:
            pose_obj.position.x = handle_negative_hr(pose_value[0]) / 1000.0
            pose_obj.position.y = handle_negative_hr(pose_value[1]) / 1000.0
            pose_obj.position.z = handle_negative_hr(pose_value[2]) / 1000.0
            pose_obj.rpy.roll = handle_negative_hr(pose_value[3]) / 1000.0
            pose_obj.rpy.pitch = handle_negative_hr(pose_value[4]) / 1000.0
            pose_obj.rpy.yaw = handle_negative_hr(pose_value[5]) / 1000.0

    @staticmethod
    def update_point(point_obj, point_value):
        """
        Update a Point given registers values
        :param point_obj: object point to update
        :type point_obj: Point
        :param point_value: new point value from 16 bits int registers
        :type point_value: list[int]
        """
        if len(point_value) == 3:
            point_obj.x = handle_negative_hr(point_value[0]) / 1000.0
            point_obj.y = handle_negative_hr(point_value[1]) / 1000.0
            point_obj.z = handle_negative_hr(point_value[2]) / 1000.0

    # - usefull methods

    @staticmethod
    def robot_state_msg_to_list(robot_state):
        return [robot_state.position.x, robot_state.position.y, robot_state.position.z,
                robot_state.rpy.roll, robot_state.rpy.pitch, robot_state.rpy.yaw]

    @staticmethod
    def object_pose_to_list(object_pose):
        return [object_pose.x, object_pose.y, object_pose.z,
                object_pose.roll, object_pose.pitch, object_pose.yaw]

    @staticmethod
    def convert_register_to_string(values):
        """
        Convert a list of registers to a string value
        :param values: registers value
        :type values: list[int]
        :return: A string
        :rtype: str
        """
        string_value = ""
        for i in range(len(values)):
            decimal_elem = values[i]
            hexa_elem = hex(decimal_elem)
            string_elem = hexa_elem[2:].decode("hex")
            string_value += string_elem

        string_value = string_value.replace('\x00', '')
        return string_value

    @staticmethod
    def convert_string_to_register(value): # currently not used because no need to write string in register
        """
        Convert a string value to a list of registers
        :param value: A string
        :type value: str
        :return: registers value
        :rtype: list[int]
        """
        if isinstance(value, str):
            if pymodbus.__version__ == '1.3.2':
                # by default, on Version 1.3.2, endian is Endian.Little, 
                # so registers were written in the reverse order. 
                # For now, we use this version on the raspberry pi
                # and v 2.4.0 in simulation
                builder = BinaryPayloadBuilder(endian=Endian.Big)
            else:
                builder = BinaryPayloadBuilder()
            builder.add_string(value)
            payload = builder.to_registers()
            return payload
        else:
            return None

    @staticmethod
    def get_pose_for_registers_from_target_pose(target_pose):
        """
        Returns a list of 16bits int representing the target_pose of type RobotState

        :param target_pose: a pose
        :type target_pose: RobotState
        :return: pose values to store in register
        :rtype: list[int]
        """
        obj_pose_position = target_pose.position
        obj_pose_rpy = target_pose.rpy

        obj_pose = [0] * 6

        obj_pose[0] = handle_negative_to_write_in_register(int(obj_pose_position.x * 1000))
        obj_pose[1] = handle_negative_to_write_in_register(int(obj_pose_position.y * 1000))
        obj_pose[2] = handle_negative_to_write_in_register(int(obj_pose_position.z * 1000))
        obj_pose[3] = handle_negative_to_write_in_register(int(obj_pose_rpy.roll * 1000))
        obj_pose[4] = handle_negative_to_write_in_register(int(obj_pose_rpy.pitch * 1000))
        obj_pose[5] = handle_negative_to_write_in_register(int(obj_pose_rpy.yaw * 1000))

        return obj_pose

    @staticmethod
    def get_pose_for_registers_from_rel_pose(obj_pose_rel):
        """
        Returns a list of 16bits int representing the relative pose of type ObjectPose

        :param obj_pose_rel: a pose
        :type obj_pose_rel: ObjectPose
        :return: pose values to store in register
        :rtype: list[int]
        """
        obj_pose = [0] * 6

        obj_pose[0] = handle_negative_to_write_in_register(int(obj_pose_rel.x * 1000))
        obj_pose[1] = handle_negative_to_write_in_register(int(obj_pose_rel.y * 1000))
        obj_pose[2] = handle_negative_to_write_in_register(int(obj_pose_rel.z * 1000))
        obj_pose[3] = handle_negative_to_write_in_register(int(obj_pose_rel.roll * 1000))
        obj_pose[4] = handle_negative_to_write_in_register(int(obj_pose_rel.pitch * 1000))
        obj_pose[5] = handle_negative_to_write_in_register(int(obj_pose_rel.yaw * 1000))

        return obj_pose

    def store_pose_shape_color_in_registers(self, obj_pose, obj_type_str, obj_color_str):
        """
        Store pose, shape and color of object detected in appropriate registers. 
        If obj_pose is a null array/object, set [HR_VISION_TARGET_POSE: HR_VISION_TARGET_POSE+5] to 0. If 
        obj_pose is None, those registers are not updated.
        If obj_type_str (or obj_color_str) is None, HR_VISION_SHAPE_FOUND (or HR_VISION_COLOR_FOUND) is not updated.
        If obj_type_str (or obj_color_str) is ShapeRequest.NONE.name (or ColorRequest.NONE.name), set the register to 0.

        Registers are set to 0 if no object was detected. They are not updated in case the function
        used doesn't return any information on the variable.

        obj_type_str and obj_color_str should be string, like 'ANY' or 'BLUE'. 
        :param obj_pose: a pose
        :type obj_pose: either a RobotState, an ObjectPose or an array of 6x0. If None, the register is not updated.
        :param obj_type_str: name of the object type, corresponding to a name in ShapeRequest
        :type obj_type_str: str or None
        :param obj_color_str: name of the object color, corresponding to a name in ColorRequest
        :type obj_color_str: str or None
        """
        # POSE
        if obj_pose is not None:
            if obj_pose._type == "niryo_robot_msgs/RobotState":
                obj_pose = self.get_pose_for_registers_from_target_pose(obj_pose)

            elif obj_pose._type == "niryo_robot_msgs/ObjectPose":
                obj_pose = self.get_pose_for_registers_from_rel_pose(obj_pose)

            self.setValuesOffset(HR_VISION_TARGET_POSE, obj_pose)

        # Type (Shape)
        if obj_type_str is not None: 
            obj_shape_register = handle_negative_to_write_in_register(ShapeRequest[obj_type_str].value)
            self.setValuesOffset(HR_VISION_SHAPE_FOUND, [obj_shape_register])

        # Color
        if obj_color_str is not None:
            obj_color_register = handle_negative_to_write_in_register(ColorRequest[obj_color_str].value)
            self.setValuesOffset(HR_VISION_COLOR_FOUND, [obj_color_register])


    def reset_result_registers(self,address_list): # currently not used 
        """
        Empty results register at the beginning of a vision method
        to delete old results
        """
        for address in address_list:
            self.setValuesOffset(address,[0])



