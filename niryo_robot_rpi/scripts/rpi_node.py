#!/usr/bin/env python


import rospy

from button_manager import NiryoButton
from digital_io_panel import DigitalIOPanel
from led_manager import LEDManager
from ros_log_manager import RosLogManager
from shutdown_manager import ShutdownManager
from niryo_robot_rpi.fans_manager import FansManager


class NiryoRobotRpi:

    def __init__(self):
        self.__digital_io_panel = DigitalIOPanel()
        self.__fans_manager = FansManager()
        self.__led_manager = LEDManager()
        self.__niryo_robot_button = NiryoButton()
        self.__ros_log_manager = RosLogManager()
        self.__shutdown_manager = ShutdownManager()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_rpi', anonymous=False, log_level=rospy.INFO)
    try:
        node = NiryoRobotRpi()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
