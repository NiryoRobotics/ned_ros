#!/usr/bin/env python
import rospy
from robot_status_handler import RobotStatusHandler


class RobotStatus:

    def __init__(self):
        self.__robot_status_handler = RobotStatusHandler()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_status', anonymous=False, log_level=rospy.INFO)
    try:
        node = RobotStatus()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
