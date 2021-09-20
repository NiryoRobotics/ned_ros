#!/usr/bin/env python

import rospy
import logging

from ros_log_manager import RosLogManager
from shutdown_manager import ShutdownManager


class NiryoRobotRpi:
    def __init__(self):
        rospy.logdebug("NiryoRobotRpi - Entering in Init")

        if rospy.get_param("~hardware_version") == "ned2":
            from niryo_robot_rpi.MCP23017 import MCP23017
            from button_manager import NiryoButtonNed2
            from io_panel import McpIOPanel
            from fans_manager import FansManagerNed2
            from end_effector_panel import NiryoEndEffectorPanel

            self.__mcp = MCP23017(address=rospy.get_param("~mcp/address"),
                                  busnum=rospy.get_param("~mcp/i2c_bus"))

            self.__io_panel = McpIOPanel(mcp=self.__mcp)
            self.__fans_manager = FansManagerNed2(mcp=self.__mcp)
            self.__niryo_robot_button = NiryoButtonNed2()
            self.__end_effector_panel = NiryoEndEffectorPanel()

        else:
            from button_manager import NiryoButton
            from io_panel import DigitalRpiIOPanel
            from led_manager import LEDManager
            from fans_manager import FansManagerNedOne

            self.__io_panel = DigitalRpiIOPanel()
            self.__fans_manager = FansManagerNedOne()
            self.__led_manager = LEDManager()
            self.__niryo_robot_button = NiryoButton()

        self.__ros_log_manager = RosLogManager()
        self.__shutdown_manager = ShutdownManager()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_rpi', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = NiryoRobotRpi()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
