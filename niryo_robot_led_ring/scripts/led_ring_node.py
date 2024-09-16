#!/usr/bin/env python

# Lib
# import sys
import rospy
import logging
import rosnode
from niryo_robot_utils import sentry_init

from threading import Thread
from niryo_robot_led_ring.led_ring_commander import LedRingCommander


class LedRingNode:
    def __init__(self):
        self.led_ring_commander = LedRingCommander()


if __name__ == '__main__':
    sentry_init()

    rospy.init_node('niryo_robot_led_ring_commander', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = LedRingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
