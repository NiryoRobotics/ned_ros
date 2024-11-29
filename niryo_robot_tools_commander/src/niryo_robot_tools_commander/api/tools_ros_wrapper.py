#!/usr/bin/env python
# Lib

from niryo_robot_utils import NiryoRosWrapperException, NiryoActionClient, NiryoTopicValue, AbstractNiryoRosWrapper

# Command Status
from niryo_robot_msgs.msg import CommandStatus, SoftwareVersion

# Messages
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Int32
from niryo_robot_msgs.msg import RPY
from niryo_robot_tools_commander.msg import ToolCommand, TCP

# Services
from niryo_robot_msgs.srv import SetBool, SetInt, Trigger

# Actions
from niryo_robot_tools_commander.msg import ToolGoal, ToolAction

# Enums
from .tools_ros_wrapper_enums import ToolID


class ToolsRosWrapper(AbstractNiryoRosWrapper):

    def __init__(self, service_timeout=0.2):
        super(ToolsRosWrapper, self).__init__(service_timeout)

        # -- Subscribers
        self.__current_tool_id_ntv = NiryoTopicValue('/niryo_robot_tools_commander/current_id', Int32)
        self.__tcp_ntv = NiryoTopicValue('/niryo_robot_tools_commander/tcp', TCP)

        # -- Tool action
        self.__tool_action_nac = NiryoActionClient('/niryo_robot_tools_commander/action_server', ToolAction, ToolGoal)

    def get_current_tool_id(self):
        """
        Uses /niryo_robot_tools_commander/current_id  topic to get current tool id

        :return: Tool Id
        :rtype: ToolID
        """
        return self.__current_tool_id_ntv.value.data

    def update_tool(self):
        """
        Calls service niryo_robot_tools_commander/update_tool to update tool

        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_tools_commander/update_tool', Trigger)
        return self._classic_return_w_check(result)

    def grasp_with_tool(self, pin_id=""):
        """
        Grasps with the tool linked to tool_id.
        This action corresponds to
        - Close gripper for Grippers
        - Pull Air for Vacuum pump
        - Activate for Electromagnet

        :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
        :type pin_id: PinID
        :return: status, message
        :rtype: (int, str)
        """
        tool_id = self.get_current_tool_id()

        if tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4):
            return self.close_gripper()
        elif tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2):
            return self.pull_air_vacuum_pump()
        elif tool_id == ToolID.ELECTROMAGNET_1:
            return self.activate_electromagnet(pin_id)

    def release_with_tool(self, pin_id=""):
        """
        Releases with the tool associated to tool_id.
        This action corresponds to
        - Open gripper for Grippers
        - Push Air for Vacuum pump
        - Deactivate for Electromagnet

        :param pin_id: [Only required for electromagnet] Pin ID of the electromagnet
        :type pin_id: PinID
        :return: status, message
        :rtype: (int, str)
        """
        tool_id = self.get_current_tool_id()

        if tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3, ToolID.GRIPPER_4):
            return self.open_gripper()
        elif tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2):
            return self.push_air_vacuum_pump()
        elif tool_id == ToolID.ELECTROMAGNET_1:
            return self.deactivate_electromagnet(pin_id)

    # - Gripper
    def open_gripper(self, speed=500, max_torque_percentage=100, hold_torque_percentage=20):
        """
        Opens gripper with a speed 'speed'

        :param speed: Default -> 500
        :type speed: int
        :param max_torque_percentage: Default -> 100
        :type max_torque_percentage: int
        :param hold_torque_percentage: Default -> 20
        :type hold_torque_percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_gripper(ToolCommand.OPEN_GRIPPER, speed, max_torque_percentage, hold_torque_percentage)

    def close_gripper(self, speed=500, max_torque_percentage=100, hold_torque_percentage=50):
        """
        Closes gripper with a speed 'speed'

        :param speed: Default -> 500
        :type speed: int
        :param max_torque_percentage: Default -> 100
        :type max_torque_percentage: int
        :param hold_torque_percentage: Default -> 20
        :type hold_torque_percentage: int
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_gripper(ToolCommand.CLOSE_GRIPPER, speed, max_torque_percentage, hold_torque_percentage)

    def __deal_with_gripper(self, command_int, speed=500, max_torque_percentage=100, hold_torque_percentage=100):
        goal = ToolGoal()
        goal.cmd.tool_id = self.get_current_tool_id()
        goal.cmd.cmd_type = command_int
        goal.cmd.max_torque_percentage = max_torque_percentage
        goal.cmd.hold_torque_percentage = hold_torque_percentage
        if command_int == ToolCommand.OPEN_GRIPPER:
            goal.cmd.speed = speed
        else:
            goal.cmd.speed = speed
        return self.__tool_action_nac.execute(goal)

    # - Vacuum
    def pull_air_vacuum_pump(self):
        """
        Pulls air

        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_vacuum_pump(ToolCommand.PULL_AIR_VACUUM_PUMP)

    def push_air_vacuum_pump(self):
        """
        Pulls air

        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_vacuum_pump(ToolCommand.PUSH_AIR_VACUUM_PUMP)

    def __deal_with_vacuum_pump(self, command_int):
        goal = ToolGoal()
        goal.cmd.tool_id = self.get_current_tool_id()
        goal.cmd.cmd_type = command_int

        return self.__tool_action_nac.execute(goal)

    # - Electromagnet
    def setup_electromagnet(self, pin_id):
        """
        Setups electromagnet on pin

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_tools_commander/equip_electromagnet', SetInt, ToolID.ELECTROMAGNET_1)

        if result.status != CommandStatus.SUCCESS:
            return result.status, result.message

        return self.__deal_with_electromagnet(pin_id, ToolCommand.SETUP_DIGITAL_IO)

    def activate_electromagnet(self, pin_id):
        """
        Activates electromagnet associated to electromagnet_id on pin_id

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_electromagnet(pin_id, ToolCommand.ACTIVATE_DIGITAL_IO)

    def deactivate_electromagnet(self, pin_id):
        """
        Deactivates electromagnet associated to electromagnet_id on pin_id

        :param pin_id: Pin ID
        :type pin_id:  PinID
        :return: status, message
        :rtype: (int, str)
        """
        return self.__deal_with_electromagnet(pin_id, ToolCommand.DEACTIVATE_DIGITAL_IO)

    def __deal_with_electromagnet(self, pin_id, command_int):
        goal = ToolGoal()
        goal.cmd.tool_id = ToolID.ELECTROMAGNET_1
        goal.cmd.cmd_type = command_int
        goal.cmd.gpio = pin_id
        return self.__tool_action_nac.execute(goal)

    # - TCP
    def enable_tcp(self, enable=True):
        """
        Enables or disables the TCP function (Tool Center Point).
        If activation is requested, the last recorded TCP value will be applied.
        The default value depends on the gripper equipped.
        If deactivation is requested, the TCP will be coincident with the tool_link.

        :param enable: True to enable, False otherwise.
        :type enable: bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_tools_commander/enable_tcp', SetBool, enable)
        return self._classic_return_w_check(result)

    def get_tcp(self):
        """
        Returns the TCP state
        :return: the tcp (enabled, position and orientation)
        :rtype: Tool msg object
        """
        return self.__tcp_ntv.value

    def set_tcp(self, x, y, z, roll, pitch, yaw):
        """
        Activates the TCP function (Tool Center Point)
        and defines the transformation between the tool_link frame and the TCP frame.

        :param x:
        :type x: float
        :param y:
        :type y: float
        :param z:
        :type z: float
        :param roll:
        :type roll: float
        :param pitch:
        :type pitch: float
        :param yaw:
        :type yaw: float
        :return: status, message
        :rtype: (int, str)
        """
        from niryo_robot_tools_commander.srv import SetTCP, SetTCPRequest
        req = SetTCPRequest(position=Point(x, y, z), rpy=RPY(roll, pitch, yaw))
        result = self._call_service('/niryo_robot_tools_commander/set_tcp', SetTCP, req)
        return self._classic_return_w_check(result)

    def reset_tcp(self):
        """
        Resets the TCP (Tool Center Point) transformation.
        The TCP will be reset according to the tool equipped.

        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot_tools_commander/reset_tcp', Trigger)
        return self._classic_return_w_check(result)

    def tool_reboot(self):
        """
        Reboots the motor of the tool equipped. Useful when an Overload error occurs. (cf HardwareStatus)

        :return: status, message
        :rtype: (int, str)
        """
        result = self._call_service('/niryo_robot/tools/reboot', Trigger)
        return self._classic_return_w_check(result)
