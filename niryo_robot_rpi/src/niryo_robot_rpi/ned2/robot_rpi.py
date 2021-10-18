#!/usr/bin/env python

import rospy

from niryo_robot_status.msg import RobotStatus

from niryo_robot_rpi.commun.ros_log_manager import RosLogManager

from hardware.MCP23017 import MCP23017

from .top_button import TopButton
from .io_panel import IOPanel
from .fans_manager import FansManager
from .end_effector_panel import NiryoEndEffectorPanel
from .shutdown_manager import ShutdownManager


class RobotRpi:
    def __init__(self):
        self.__mcp = MCP23017(address=rospy.get_param("~mcp/address"),
                              busnum=rospy.get_param("~mcp/i2c_bus"))

        self.__io_panel = IOPanel(mcp=self.__mcp)
        self.__fans_manager = FansManager(mcp=self.__mcp)
        self.__niryo_robot_button = TopButton()
        self.__end_effector_panel = NiryoEndEffectorPanel()
        self.__shutdown_manager = ShutdownManager()

        self.__ros_log_manager = RosLogManager()

        rospy.on_shutdown(self.shutdown)

        self.robot_status_subscriber = rospy.Subscriber('/niryo_robot_status/robot_status',
                                                        RobotStatus, self.__callback_robot_status, queue_size=10)

    def __callback_robot_status(self, msg):
        if msg.robot_status == RobotStatus.SHUTDOWN:
            self.shutdown()

    def shutdown(self):
        self.__fans_manager.shutdown()
        self.__io_panel.shutdown()
        rospy.sleep(2)
