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
    # Calibrate robot first
    n.calibrate_auto()
    print "Calibration finished !"

    # Move
    n.set_arm_max_velocity(100)

    n.move_pose(0.3, 0, 0.4, 0, 0, 0)

    trajectory = [[0.3, 0.1, 0.3, 0, 0, 0],
                  [0.3, 0, 0.2, 0, 0, 0],
                  [0.3, -0.1, 0.3, 0, 0, 0],
                  [0.3, 0, 0.4, 0, 0, 0], ]

    time.sleep(1)
    n.execute_trajectory_from_poses(trajectory, dist_smoothing=0.0)
    time.sleep(1)
    n.move_circle(0.3, 0, 0.3)
    time.sleep(1)
    n.move_to_sleep_pose()
    n.set_learning_mode(True)


except NiryoRosWrapperException as e:
    print e
    # handle exception here
    # you can also make a try/except for each command separately

print "--- End"
