#!/usr/bin/env python

import rospy
import logging

from niryo_robot_utils import sentry_init


class NiryoRobotRpiNode:

    def __init__(self):
        rospy.logdebug("NiryoRobotRpi - Entering in Init")

        hardware_version = rospy.get_param('~hardware_version')

        if hardware_version in ["ned2", "ned3pro"]:
            from niryo_robot_rpi.panel_v2.robot_rpi import RobotRpi
        else:
            from niryo_robot_rpi.panel_v1.robot_rpi import RobotRpi

        self.__niryo_robot_rpi = RobotRpi()

        # Set a bool to mention this node is initialized
        rospy.set_param('~initialized', True)


if __name__ == '__main__':
    sentry_init()

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
