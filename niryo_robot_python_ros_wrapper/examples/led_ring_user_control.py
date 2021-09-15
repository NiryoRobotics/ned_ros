#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy
import time
import math

rospy.init_node('niryo_robot_example_python_ros_wrapper')

print "--- Start"

n = NiryoRosWrapper()

try:
    time.sleep(1)

    print n.led_ring_solid([0, 255, 0], wait=True)
    print n.led_ring_flash([20, 255, 78], iterations=2, wait=True)
    print n.led_ring_alternate([[0, 255, 0], [255, 0, 0], [0, 0, 255]],
                               wait=True)  # wait = True but no iterations specified => won't wait
    rospy.sleep(4)
    print n.led_ring_chase([178, 78, 100])
    rospy.sleep(4)
    print n.led_ring_wipe([98, 78, 190])
    rospy.sleep(4)
    print n.led_ring_rainbow()
    rospy.sleep(4)
    print n.led_ring_rainbow_cycle()
    rospy.sleep(4)
    print n.led_ring_rainbow_chase()
    rospy.sleep(4)
    print n.led_ring_go_up([189, 96, 113])
    rospy.sleep(4)
    print n.led_ring_go_up_down([70, 167, 97])
    rospy.sleep(4)
    print n.led_ring_turn_off()
    rospy.sleep(4)

except NiryoRosWrapperException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"
