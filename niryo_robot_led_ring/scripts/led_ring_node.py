#!/usr/bin/env python

# Lib
import rospy
import logging
import rosnode

from threading import Thread
from niryo_robot_led_ring.led_ring_commander import LedRingCommander


class LedRingNode:
    def __init__(self):
        self.led_ring_commander = LedRingCommander()

        self.__shutdown_watcher_thread = Thread(target=self.shutdown_thread)
        self.__shutdown_watcher_thread.start()

    def shutdown_thread(self):
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.5)
                rosnode.get_node_names()
        except rosnode.ROSNodeIOException:
            self.led_ring_commander.shutdown()
            rospy.signal_shutdown("shutdown")

if __name__ == '__main__':
    rospy.init_node('niryo_robot_led_ring_commander', anonymous=False, log_level=rospy.INFO, disable_signals=True)
    node = None

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = LedRingNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
