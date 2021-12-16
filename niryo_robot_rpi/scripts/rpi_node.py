#!/usr/bin/env python

import rospy
import logging


class NiryoRobotRpiNode:
    def __init__(self):
        rospy.logdebug("NiryoRobotRpi - Entering in Init")

        if rospy.get_param("~hardware_version") == "ned2":
            from niryo_robot_rpi.ned2.robot_rpi import RobotRpi
        else:
            from niryo_robot_rpi.ned_one.robot_rpi import RobotRpi

        self.__niryo_robot_rpi = RobotRpi()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_rpi', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = NiryoRobotRpiNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
