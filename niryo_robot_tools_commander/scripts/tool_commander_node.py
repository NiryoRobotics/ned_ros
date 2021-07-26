#!/usr/bin/env python

import rospy
import actionlib
import moveit_commander

from tools_classes import *
from tool_ros_command_interface import ToolRosCommandInterface

from transform_handler import ToolTransformHandler

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from std_msgs.msg import Int32

from niryo_robot_tools_commander.msg import ToolAction, ToolResult
from niryo_robot_tools_commander.msg import ToolCommand

# Services
from niryo_robot_msgs.srv import Trigger, SetInt


class ToolsState:
    def __init__(self, state_dict):
        self.PING_OK = state_dict["ping_ok"]
        self.PING_ERROR = state_dict["ping_error"]
        self.WRONG_ID = state_dict["wrong_id"]
        self.TIMEOUT = state_dict["timeout"]

        self.GRIPPER_OPEN = state_dict["gripper_open"]
        self.GRIPPER_CLOSE = state_dict["gripper_close"]

        self.VACUUM_PUMP_PULLED = state_dict["vacuum_pump_pulled"]
        self.VACUUM_PUMP_PUSHED = state_dict["vacuum_pump_pushed"]

        self.ROS_COMMUNICATION_PROBLEM = state_dict["ros_communication_problem"]


class ToolCommander:
    def __init__(self):
        self.__tools_state = ToolsState(rospy.get_param("~state_dict"))
        self.__ros_command_interface = ToolRosCommandInterface(self.__tools_state.ROS_COMMUNICATION_PROBLEM)

        # Transform Handlers
        self.__transform_handler = ToolTransformHandler()

        self.__current_tool = None

        self.__available_tools, self.__dict_commands_string_to_id = self.create_tools()

        self.__dict_id_commands_to_string = {string: id_ for id_, string
                                             in self.__dict_commands_string_to_id.iteritems()}

        self.__is_simulation = rospy.get_param("~simulation_mode")

        # if gripper simulated, setup variables to control it through moveit
        self.__is_gripper_simulated = rospy.get_param("~simu_gripper")

        if self.__is_gripper_simulated:
            # Get Tool MoveGroupCommander
            self.__tool_simu = moveit_commander.MoveGroupCommander(rospy.get_param("~move_group_tool_commander_name"))
            # Set pose reference frame
            self.__tool_simu.set_pose_reference_frame(rospy.get_param("~reference_frame"))
            

        # Subscriber
        rospy.Subscriber('/niryo_robot_hardware/tools/current_id', Int32,
                         self.__callback_current_tool_id)

        # Publisher
        self.__tool_id_publisher = rospy.Publisher('~current_id', Int32, queue_size=10, latch=True)

        # Services
        rospy.Service('~update_tool', Trigger,
                      self.__callback_update_tool)

        rospy.Service('~equip_electromagnet', SetInt,
                      self.__callback_equip_electromagnet)

        # Action Server
        self.__action_server = actionlib.SimpleActionServer('~action_server', ToolAction,
                                                            self.__callback_goal, auto_start=False)
        self.__end_init()

        rospy.logdebug("Poses Handlers - Transform Handler created")
        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Tools Commander - Started")

    def __end_init(self):
        self.__action_server.start()
        self.update_tool()

    # Subscriber

    def __callback_current_tool_id(self, msg):
        if self.__current_tool != "electromagnet":
            id_ = msg.data
            self.set_tool(self.__available_tools[id_])

        self.__tool_id_publisher.publish(self.__current_tool.get_id())

    # - Callbacks

    def __callback_goal(self, goal):
        cmd = goal.cmd
        rospy.logdebug("Tool Commander - received goal : " + str(goal))

        if self.__current_tool is None:
            rospy.logwarn("Tool Commander - No tool selected")
            self.__action_server.set_aborted(self.create_action_result(CommandStatus.TOOL_FAILURE, "No tool selected"))
            return

        if not self.__is_simulation:
            # 1. Check tool id
            if cmd.tool_id != self.__current_tool.get_id():
                msg = "Tools ID do not match -> Given : {} Expected : {}".format(cmd.tool_id,
                                                                                 self.__current_tool.get_id())
                rospy.logwarn("Tool Commander - {}".format(msg))
                self.__action_server.set_aborted(self.create_action_result(CommandStatus.TOOL_FAILURE, msg))
                return

            # 2. Check if current tool is active
            if self.__current_tool.is_active():
                self.__action_server.set_aborted(
                    self.create_action_result(CommandStatus.TOOL_FAILURE, "Tool still active, retry later"))
                return

        # 3. Check cmd_type (if exists, and is available for selected tool)
        # Skip in case of simulation
        if not self.__is_simulation and cmd.cmd_type not in self.__current_tool.get_available_commands():
            msg = "Command '{}' is not available for {}".format(self.__dict_id_commands_to_string[cmd.cmd_type],
                                                                self.__current_tool.name)
            rospy.logwarn("Tool Commander - {}".format(msg))
            self.__action_server.set_aborted(self.create_action_result(CommandStatus.TOOL_FAILURE, msg))
            return

        # 3.1 Validate params
        try:
            self.__current_tool.validate_command(cmd)
        except ToolValidationException as e:
            self.__action_server.set_aborted(self.create_action_result(CommandStatus.TOOL_FAILURE, str(e)))
            return

        if not self.__is_simulation:
            # 4. Execute cmd -> retrieve cmd name in command list and execute on current tool
            self.__current_tool.set_as_active()

            function_name = self.__dict_id_commands_to_string[cmd.cmd_type]
            success, message = self.__current_tool(function_name, cmd)  # Execute function from name
            
            self.__current_tool.set_as_non_active()

            # 5. Return success or error
            if success:
                self.__action_server.set_succeeded(
                    self.create_action_result(CommandStatus.SUCCESS, "Tool action successfully finished"))
            else:
                rospy.loginfo("Tool Commander - error : {}".format(message))
                self.__action_server.set_aborted(self.create_action_result(CommandStatus.TOOL_FAILURE, message))
        
        # Gripper simulated
        elif self.__is_gripper_simulated:
            if cmd.cmd_type == ToolCommand.OPEN_GRIPPER:
                self.__tool_simu.set_named_target("open")
                self.__tool_simu.go()
            elif cmd.cmd_type == ToolCommand.CLOSE_GRIPPER:
                self.__tool_simu.set_named_target("close")
                self.__tool_simu.go()     
            
            self.__action_server.set_succeeded(
                self.create_action_result(CommandStatus.SUCCESS, "Simulated tool action successfully finished"))      
            return

        #Simulation mode without gripper
        else: 
            self.__action_server.set_succeeded(
                self.create_action_result(CommandStatus.SUCCESS, "Tool action successfully finished"))

    def __callback_update_tool(self, _req):
        state, id_ = self.update_tool()
        if state != self.__tools_state.PING_OK:
            return self.create_response(CommandStatus.TOOL_FAILURE,
                                        "Impossible to set new tool")

        if id_ not in self.__available_tools:
            # no tool found in available tools
            return self.create_response(CommandStatus.TOOL_ID_INVALID, "This ID does not match any available tool ID")

        self.set_tool(self.__available_tools[id_])

        return self.create_response(CommandStatus.SUCCESS,
                                    "New tool has been set, id : {}".format(id_))

    def __callback_equip_electromagnet(self, req):
        id_ = req.value
        if id_ not in self.__available_tools:
            # no tool found in available tools
            return self.create_response(CommandStatus.TOOL_ID_INVALID, "This ID does not match any electromagnet ID")

        self.set_tool(self.__available_tools[id_])

        return CommandStatus.SUCCESS, "Electromagnet properly equipped"

    # Functions
    def create_tools(self):
        # Get params from rosparams
        tool_config_dict = rospy.get_param("~tool_list")
        list_commands_all_tools = rospy.get_param("~command_list")

        dict_tools = {}

        for tool in tool_config_dict:
            tool_type, tool_id, tool_name, tool_transformation, specs = (tool[key] for key in ['type', 'id', 'name', 'transformation', 'specs'])
            tool_transformation = self.__transform_handler.transform_from_dict(tool_transformation)
            if tool_type == Gripper.get_type():
                new_tool = Gripper(tool_id, tool_name, tool_transformation, self.__tools_state, self.__ros_command_interface, specs)
            elif tool_type == Electromagnet.get_type():
                new_tool = Electromagnet(tool_id, tool_name, tool_transformation, self.__tools_state, self.__ros_command_interface)
            elif tool_type == VacuumPump.get_type():
                new_tool = VacuumPump(tool_id, tool_name, tool_transformation, self.__tools_state, self.__ros_command_interface, specs)
            elif tool_type == NoTool.get_type():
                new_tool = NoTool(tool_id, tool_name, tool_transformation, self.__tools_state, self.__ros_command_interface)
            else:
                rospy.logwarn("Tool Commander - Type not recognized from tool config list : " + str(tool['type']))
                continue

            tool_command_list = []

            for cmd in tool['available_commands']:
                try:
                    tool_command_list.append(list_commands_all_tools[cmd])
                except KeyError:
                    rospy.logwarn("Tool Commander - Unknown command : {}".format(cmd))
                    continue
                
            new_tool.set_available_commands(tool_command_list)

            dict_tools[tool_id] = new_tool

        return dict_tools, list_commands_all_tools

    def update_tool(self):
        return self.__ros_command_interface.ping_dxl_tool()

    def set_tool(self, tool_name):
        self.__current_tool = tool_name
        self.__tool_id_publisher.publish(self.__current_tool.get_id())
        self.__transform_handler.set_tool(self.__current_tool.transformation)

    # - General Purposes
    @staticmethod
    def create_action_result(status, message):
        result = ToolResult()
        result.status = status
        result.message = message
        return result

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}


if __name__ == '__main__':
    rospy.init_node('niryo_robot_tools_commander', anonymous=False, log_level=rospy.INFO)
    try:
        node = ToolCommander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
