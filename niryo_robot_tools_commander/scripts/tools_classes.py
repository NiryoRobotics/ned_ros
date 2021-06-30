#!/usr/bin/env python

# Command Status
from niryo_robot_msgs.msg import CommandStatus

__all__ = [
    "Tool",
    "NoTool",
    "Gripper",
    "Electromagnet",
    "VacuumPump",

    "ToolValidationException",
]


class ToolValidationException(Exception):
    pass


#
# Base class for any tool
#
class Tool(object):
    def __init__(self, tool_id, tool_name, tool_transformation, tools_state, ros_command_interface):
        self._functions_dict = {}
        self._id = tool_id
        self._name = tool_name
        self._tools_state = tools_state
        self.ros_command_interface = ros_command_interface
        self._is_active = False
        self.available_commands = []
        self._tool_transformation = tool_transformation

    # To override
    @staticmethod
    def get_type():
        raise NotImplementedError

    # To override
    def validate_command(self, cmd):
        raise NotImplementedError

    def set_as_active(self):
        self._is_active = True

    def set_as_non_active(self):
        self._is_active = False

    def is_active(self):
        return self._is_active

    def get_id(self):
        return self._id

    @property
    def name(self):
        return self._name

    @property
    def transformation(self):
        return self._tool_transformation

    def set_available_commands(self, cmd_list):
        self.available_commands = cmd_list

    def get_available_commands(self):
        return self.available_commands

    def __call__(self, function_name, *args):
        return self._functions_dict[function_name](*args)

    def __str__(self):
        return "Tool {} of type {} with ID {}".format(self._name, self.get_type(), self._id)

    def __repr__(self):
        return self.__str__()


class NoTool(Tool):
    def __init__(self, tool_id, tool_name, tool_transformation, tools_state, ros_command_interface):
        super(NoTool, self).__init__(tool_id, tool_name, tool_transformation, tools_state, ros_command_interface)

    @staticmethod
    def get_type():
        return "no_tool"

    def validate_command(self, cmd):
        pass

    def is_connected(self):
        state, id_ = self.ros_command_interface.ping_dxl_tool()
        return state == self._tools_state.PING_OK and id_ == self._id


class Gripper(Tool):
    def __init__(self, tool_id, tool_name, tool_transformation, tools_state, ros_command_interface, specs):
        super(Gripper, self).__init__(tool_id, tool_name, tool_transformation, tools_state, ros_command_interface)

        self.open_position = specs["open_position"]
        self.open_hold_torque = specs["open_hold_torque"]
        self.close_position = specs["close_position"]
        self.close_hold_torque = specs["close_hold_torque"]
        self.close_max_torque = specs["close_max_torque"]

        self.open_speed_limits = specs["limit_open_speed"]
        self.close_speed_limits = specs["limit_close_speed"]

        self._functions_dict = {
            "open_gripper": self.open_gripper,
            "close_gripper": self.close_gripper,
        }

    @staticmethod
    def get_type():
        return "gripper"

    def validate_command(self, cmd):
        if not self.open_speed_limits["min"] <= cmd.gripper_open_speed < self.open_speed_limits["max"]:
            raise ToolValidationException(
                "Gripper open speed must be in ( {}, {})".format(self.open_speed_limits["min"],
                                                                 self.open_speed_limits["max"]))
        elif not self.close_speed_limits["min"] <= cmd.gripper_close_speed < self.close_speed_limits["max"]:
            raise ToolValidationException(
                "Gripper close speed must be in ( {}, {})".format(self.close_speed_limits["min"],
                                                                  self.close_speed_limits["max"]))

    def is_connected(self):
        state, id_ = self.ros_command_interface.ping_dxl_tool()
        return state == self._tools_state.PING_OK and id_ == self._id

    def return_gripper_status(self, state):
        if state == self._tools_state.GRIPPER_OPEN:
            return True, "Successfully opened gripper"
        if state == self._tools_state.GRIPPER_CLOSE:
            return True, "Successfully closed gripper"
        if state == self._tools_state.PING_OK:
            return True, "Gripper is connected"
        if state == self._tools_state.PING_ERROR:
            return False, "Gripper not detected"
        if state == self._tools_state.TIMEOUT:
            return False, "Gripper action - Timeout"
        if state == self._tools_state.WRONG_ID:
            return False, "This gripper is not the one attached"
        if state == self._tools_state.ROS_COMMUNICATION_PROBLEM:
            return False, "A communication problem occured, please retry"

    def open_gripper(self, cmd):
        state = self.ros_command_interface.open_gripper(
            self._id, self.open_position, cmd.gripper_open_speed, self.open_hold_torque)
        return self.return_gripper_status(state)

    def close_gripper(self, cmd):
        state = self.ros_command_interface.close_gripper(
            self._id, self.close_position, cmd.gripper_close_speed, self.close_hold_torque, self.close_max_torque)
        return self.return_gripper_status(state)

    def update_params(self, open_position, open_hold_torque, close_position,
                      close_hold_torque, close_max_torque):
        self.open_position = open_position
        self.open_hold_torque = open_hold_torque
        self.close_position = close_position
        self.close_hold_torque = close_hold_torque
        self.close_max_torque = close_max_torque


class Electromagnet(Tool):
    def __init__(self, tool_id, tool_name, tool_transformation, tools_state, ros_command_interface):
        super(Electromagnet, self).__init__(tool_id, tool_name, tool_transformation, tools_state, ros_command_interface)
        self._functions_dict = {
            "setup_digital_io": self.setup_digital_io,
            "activate_digital_io": self.activate_digital_io,
            "deactivate_digital_io": self.deactivate_digital_io,
        }
        self._gpio = None

    @staticmethod
    def get_type():
        return "electromagnet"

    def validate_command(self, cmd):
        if type(cmd.activate) is not bool:
            raise ToolValidationException("Electromagnet activation command should be boolean")

    def setup_digital_io(self, cmd):
        self._gpio = cmd.gpio
        status, message = self.ros_command_interface.digital_output_tool_setup(cmd.gpio)
        if status == CommandStatus.SUCCESS:
            return True, 'Successfully setup digital output PIN  ' + str(cmd.gpio) + ' for electromagnet'
        else:
            return False, message

    def activate_digital_io(self, cmd):
        if cmd.gpio > 0:
            self._gpio = cmd.gpio
        status, message = self.ros_command_interface.digital_output_tool_activate(self._gpio, 1)
        if status == CommandStatus.SUCCESS:
            return True, 'Successfully activated eletromagnet on PIN  ' + str(cmd.gpio)
        else:
            return False, message

    def deactivate_digital_io(self, cmd):
        if cmd.gpio > 0:
            self._gpio = cmd.gpio
        status, message = self.ros_command_interface.digital_output_tool_activate(self._gpio, 0)
        if status == CommandStatus.SUCCESS:
            return True, 'Successfully deactivated eletromagnet on PIN  ' + str(cmd.gpio)
        else:
            return False, message


class VacuumPump(Tool):
    def __init__(self, tool_id, tool_name, tool_transformation, tools_state, ros_command_interface, specs):
        super(VacuumPump, self).__init__(tool_id, tool_name, tool_transformation, tools_state, ros_command_interface)
        self.__pull_air_position = specs["pull_air_position"]
        self.__pull_air_hold_torque = specs["pull_air_hold_torque"]
        self.__push_air_position = specs["push_air_position"]

        self._functions_dict = {
            "pull_air_vacuum_pump": self.pull_air_vacuum_pump,
            "push_air_vacuum_pump": self.push_air_vacuum_pump,
        }

    @staticmethod
    def get_type():
        return "vacuum_pump"

    def validate_command(self, cmd):
        if type(cmd.activate) is not bool:
            raise ToolValidationException("Vacuum Pump activation command should be boolean")

    def is_connected(self):
        state = self.ros_command_interface.ping_dxl_tool(self._id, self._name)
        return state == self._tools_state.PING_OK

    def return_vaccump_pump_state(self, state):
        if state == self._tools_state.VACUUM_PUMP_PULLED:
            return True, 'Successfully pulled air'
        elif state == self._tools_state.VACUUM_PUMP_PUSHED:
            return True, 'Successfully pushed air'
        elif state == self._tools_state.PING_OK:
            return True, 'Vacuum Pump is connected'
        elif state == self._tools_state.PING_ERROR:
            return False, 'Vacuum pump not detected'
        elif state == self._tools_state.TIMEOUT:
            return False, 'Vacump pump action - Timeout'
        elif state == self._tools_state.WRONG_ID:
            return False, 'This vacuum pump is not the one attached'
        elif state == self._tools_state.ROS_COMMUNICATION_PROBLEM:
            return False, "A communication problem occured, please retry"
        else:
            return False, "Error : Unknown vacuum pump return" + str(state)

    def pull_air_vacuum_pump(self, _):
        state = self.ros_command_interface.pull_air_vacuum_pump(self._id, self.__pull_air_position,
                                                                self.__pull_air_hold_torque)
        return self.return_vaccump_pump_state(state)

    def push_air_vacuum_pump(self, _):
        state = self.ros_command_interface.push_air_vacuum_pump(self._id, self.__push_air_position)
        return self.return_vaccump_pump_state(state)

    def update_params(self, pull_air_position, pull_air_hold_torque, push_air_position):
        self.__pull_air_position = pull_air_position
        self.__pull_air_hold_torque = pull_air_hold_torque
        self.__push_air_position = push_air_position
