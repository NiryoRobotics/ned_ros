#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy
import time
import math

WHITE = [255, 255, 255]
GREEN = [50, 255, 0]
BLACK = [0, 0, 0]
BLUE = [15, 50, 255]
PURPLE = [153, 51, 153]
PINK = [255, 0, 255]
RED = [255, 0, 0]
CYAN = [0, 255, 255]

rospy.init_node('niryo_robot_example_python_ros_wrapper')

print "--- Start"

n = NiryoRosWrapper()
# n.request_new_calibration()
n.set_arm_max_velocity(100)
n.calibrate_auto()

print "--- Prepare traj --"

print ("-- Compute traj 2 --")
n.move_pose(*[0.25, 0, 0.25, 0, 0, 0])
traj2 = n.compute_trajectory_from_poses_and_joints(
    [[0.18, 0, 0.17, 0, 0, 0], [0.35, 0, 0.17, 0, 0, 0], [0.181, 0, 0.171, 0, 0, 0],
     [0.065, -0.196, 0.341, -0.099, 0.016, 0.06], [0.096, -0.06, 0.478, -1.695, 0.024, 0.029],
     [0.075, 0.117, 0.289, -0.116, 0.087, -0.013], [0.047, 0.221, 0.189, 2.689, 1.522, 2.826],
     [0.047, 0.23, 0.35, 2.689, 1.522, 2.826], [0, 0.5, -1.25, 0, 0, 0]],
    ["pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose", "joint"], 0)
# n.execute_moveit_robot_trajectory(traj2)

print ("-- Compute traj 3 --")
n.move_pose(*[0.1, -0.01, 0.531, -0.045, 0.011, -0.074])
traj3 = n.compute_trajectory_from_poses_and_joints(
    [[0.1, -0.01, 0.531, -0.045, 0.011, -0.074], [0.152, -0.005, 0.301, -0.063, -0.073, -0.036],
     [-0.011, -0.215, 0.229, -0.307, 0.045, -0.551], [0.009, -0.117, 0.369, 0.437, 0.497, -0.223],
     [-0.018, -0.2, 0.15, -1.57, 0, -1.57], [-0.018, -0.2, 0.3, -1.57, 0, -1.57]],
    ["pose", "pose", "pose", "pose", "pose", "pose"], 0)
# n.execute_moveit_robot_trajectory(traj3)

print ("-- Compute traj 4 --")
pose_list = [
    [0.3, -0.16, 0.325, 0, 0, 0],
    [0.3, -0.13, 0.32, 0, 0, 0],
    [0.3, -0.09, 0.30, 0, 0, 0],
    [0.3, -0.06, 0.26, 0, 0, 0],
    [0.3, -0.03, 0.21, 0, 0, 0],
    [0.3, 0, 0.2, 0, 0, 0],
    [0.3, 0.03, 0.21, 0, 0, 0],
    [0.3, 0.06, 0.26, 0, 0, 0],
    [0.3, 0.09, 0.30, 0, 0, 0],
    [0.3, 0.13, 0.32, 0, 0, 0],
    [0.3, 0.16, 0.325, 0, 0, 0],
]
full_traj = pose_list[:]
pose_list.reverse()
pose_list[-1][1] += 0.01
full_traj += pose_list[:]

n.move_pose(*full_traj[0])
niryo_wave_traj = n.compute_trajectory_from_poses(full_traj, 0.001)
# n.execute_moveit_robot_trajectory(niryo_wave_traj)


print("-- Go --")

while True:
    try:
        # Calibrate robot first
        # n.request_new_calibration()
        n.wait(0.1)
        n.calibrate_auto()
        print "Calibration finished !"

        print("Wave")
        n.led_ring.breath(BLUE)
        n.move_joints(*(6 * [0]))
        # Niryo wave
        n.move_pose(*full_traj[0])
        n.set_arm_max_velocity(50)
        n.execute_moveit_robot_trajectory(niryo_wave_traj)
        n.wait(0.5)
        n.set_arm_max_velocity(100)

        print("Moves")
        n.led_ring.rainbow_cycle()
        n.execute_moveit_robot_trajectory(traj2)
        n.move_pose(*[0.179, 0.001, 0.264, 2.532, 1.532, 2.618])

        n.execute_moveit_robot_trajectory(traj3)
        n.move_pose(*[0.179, 0.001, 0.264, 2.532, 1.532, 2.618])

    except NiryoRosWrapperException as e:
        print e
        # handle exception here
        # you can also make a try/except for each command separately

print "--- End"
