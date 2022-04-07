#!/usr/bin/env python

# To use the API, copy these 4 lines on each Python file you create
import sys

from niryo_robot_python_ros_wrapper.ros_wrapper import *
from niryo_robot_arm_commander.msg import PausePlanExecution
import rospy
import threading
import random

WHITE = [255, 255, 255]
GREEN = [50, 255, 0]
BLACK = [0, 0, 0]
BLUE = [15, 50, 255]
PURPLE = [153, 51, 153]
PINK = [255, 0, 255]
RED = [255, 0, 0]
CYAN = [0, 255, 255]

state = PausePlanExecution.STANDBY


def callback_pause_movement(msg):
    global state
    state = msg.state


rospy.Subscriber('/niryo_robot_rpi/pause_state', PausePlanExecution, callback_pause_movement)
rospy.init_node('niryo_robot_example_python_ros_wrapper')
print "--- Start"

n = NiryoRosWrapper()
# n.request_new_calibration()
n.set_arm_max_velocity(100)
n.calibrate_auto()
n.wait(1)


def french_flag():
    colors = []
    colors += [[255, 255, 255] for _ in range(2)]
    colors += [[0, 0, 255] for _ in range(11)]
    colors += [[255, 255, 255] for _ in range(4)]
    colors += [[255, 0, 0] for _ in range(11)]
    colors += [[255, 255, 255] for _ in range(2)]

    n.led_ring.custom(colors)


run_flag = True


def french_flag_moving():
    colors = []
    colors += [[255, 255, 255] for _ in range(2)]
    colors += [[0, 0, 255] for _ in range(11)]
    colors += [[255, 255, 255] for _ in range(4)]
    colors += [[255, 0, 0] for _ in range(11)]
    colors += [[255, 255, 255] for _ in range(2)]

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and run_flag:
        for i in range(len(colors)):
            n.led_ring.custom(colors[i:] + colors[:i])
            rate.sleep()
            if not run_flag:
                return


def start_run_flag():
    global run_flag
    run_flag = True
    t = threading.Thread(target=french_flag_moving)
    t.start()


def stop_flag():
    global run_flag
    run_flag = False


def close_gripper():
    try:
        n.close_gripper(max_torque_percentage=100, hold_torque_percentage=100)
    except Exception:
        pass


def open_gripper():
    try:
        n.open_gripper(max_torque_percentage=100, hold_torque_percentage=100)
    except Exception:
        pass


def random_sounds():
    random_sounds = ["connected.wav", "start2b.wav", "start4abis.wav"]
    global state
    while not (rospy.is_shutdown() or state == PausePlanExecution.CANCEL):
        if not (state == PausePlanExecution.PAUSE):
            n.sound.play(random_sounds[random.randint(0, len(random_sounds) - 1)], wait_end=False)
        rospy.sleep(random.randint(5, 15))


t_sound = threading.Thread(target=random_sounds)
t_sound.start()

print "--- Prepare traj --"

n.move_joints(*(6 * [0]))
french_flag()
n.move_pose(*[0.029, 0.217, 0.3, 2.254, 1.476, -2.38])
print ("-- Compute traj 1 --")
traj1 = n.compute_trajectory_from_poses_and_joints(
    [[0.029, 0.217, 0.15, 2.254, 1.476, -2.38],
     [0.03, 0.217, 0.3, 2.254, 1.476, -2.38], [0.159, 0.126, 0.3, 0.062, 1.535, 0.96],
     [0.159, 0.126, 0.15, 0.062, 1.535, 0.96], [0.16, 0.126, 0.3, 0.062, 1.535, 0.96],
     [0.219, -0.019, 0.3, -1.555, 1.544, -1.526], [0.219, -0.019, 0.15, -1.555, 1.544, -1.526],
     [0.22, -0.019, 0.3, -1.555, 1.544, -1.526], [0.16, -0.175, 0.3, -2.693, 1.529, 2.976],
     [0.16, -0.175, 0.15, -2.693, 1.529, 2.976], [0.17, -0.175, 0.3, -2.693, 1.529, 2.976],
     [-0.015, -0.229, 0.3, -2.552, 1.563, 2.298], [-0.015, -0.229, 0.15, -2.552, 1.563, 2.298],
     [-0.016, -0.229, 0.3, -2.552, 1.563, 2.298]],
    ["pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose", "pose",
     "pose"], 0.01)
# n.execute_moveit_robot_trajectory(traj1)

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
    [[0.152, -0.005, 0.301, -0.063, -0.073, -0.036],
     [-0.011, -0.215, 0.229, -0.307, 0.045, -0.551], [0.009, -0.117, 0.369, 0.437, 0.497, -0.223],
     [-0.018, -0.2, 0.15, -1.57, 0, -1.57], [-0.018, -0.2, 0.3, -1.57, 0, -1.57]],
    ["pose", "pose", "pose", "pose", "pose"], 0)
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


# Slalome
print ("-- Compute traj 5 --")
start_pose = [0.2, -0.2, 0.15, 0, 1.57, 0]
pose_list = [start_pose]
for _ in range(2):
    new_pose = pose_list[-1][:]
    new_pose[0] += 0.1
    pose_list.append(new_pose[:])
    new_pose[1] += 0.1
    pose_list.append(new_pose[:])
    new_pose[0] -= 0.1
    pose_list.append(new_pose[:])
    new_pose[1] += 0.1
    pose_list.append(new_pose[:])
for _ in range(2):
    new_pose = pose_list[-1][:]
    new_pose[0] += 0.1
    pose_list.append(new_pose[:])
    new_pose[1] -= 0.1
    pose_list.append(new_pose[:])
    new_pose[0] -= 0.1
    pose_list.append(new_pose[:])
    new_pose[1] -= 0.1
    pose_list.append(new_pose[:])

n.move_pose(*pose_list[0])
slalome_traj = n.compute_trajectory_from_poses(pose_list, 0.001)
# n.execute_moveit_robot_trajectory(slalome_traj)

print("-- Go --")

while True:
    try:
        # Calibrate robot first
        # n.request_new_calibration()
        n.wait(0.1)
        n.calibrate_auto()
        n.update_tool()
        n.grasp_with_tool()
        print "Calibration finished !"

        print("Wave")
        open_gripper()
        n.led_ring.breath(BLUE)
        n.move_joints(*(6 * [0]))
        # Niryo wave
        n.move_pose(*full_traj[0])
        n.set_arm_max_velocity(50)
        n.execute_moveit_robot_trajectory(niryo_wave_traj)
        n.wait(0.5)
        n.set_arm_max_velocity(100)

        print("Moves")
        n.led_ring.solid(GREEN)
        threading.Timer(2, close_gripper).start()
        threading.Timer(3.4, close_gripper).start()
        threading.Timer(6, open_gripper).start()
        n.move_pose(*[0.029, 0.217, 0.3, 2.254, 1.476, -2.38])
        n.execute_moveit_robot_trajectory(traj1)
        n.move_pose(*[0.179, 0.001, 0.264, 2.532, 1.532, 2.618])

        print("Spiral")
        n.led_ring.snake(BLUE)
        n.move_pose(0.3, 0, 0.3, 0, 0, 0)
        n.move_spiral(0.1, 5, 216, 1)

        print("Moves")
        n.led_ring.breath(BLUE)
        threading.Timer(2.5, close_gripper).start()
        threading.Timer(4.8, close_gripper).start()
        threading.Timer(5.4, open_gripper).start()

        n.execute_moveit_robot_trajectory(traj2)

        print("Circle")
        french_flag()
        n.move_pose(0.3, 0, 0.4, 0, 0, 0)
        n.move_circle(0.3, 0, 0.3)

        print("Moves")
        n.led_ring.rainbow()
        threading.Timer(1, close_gripper).start()
        threading.Timer(2.1, close_gripper).start()
        threading.Timer(6.4, open_gripper).start()
        n.execute_moveit_robot_trajectory(traj3)

        print("Linear")
        n.led_ring.go_up_down(PURPLE)
        n.move_pose(0.2, 0.2, 0.2, 0, 1.57, 0)
        n.move_linear_pose(0.2, -0.2, 0.2, 0, 1.57, 0)

        print("Moves")
        n.led_ring.chase(WHITE)
        threading.Timer(3, close_gripper).start()
        threading.Timer(3.5, close_gripper).start()
        threading.Timer(4, open_gripper).start()
        n.execute_moveit_robot_trajectory(traj2)

        # Slalome
        print("Slalome")
        start_run_flag()
        close_gripper()
        n.move_pose(*pose_list[0])
        n.execute_moveit_robot_trajectory(slalome_traj)

        print("Moves")
        threading.Timer(3, close_gripper).start()
        threading.Timer(5, close_gripper).start()
        threading.Timer(6, open_gripper).start()
        n.execute_moveit_robot_trajectory(traj3)
        stop_flag()

        print("Pick and place")
        n.led_ring.snake(CYAN)
        pick_1 = [
            [0.25, -0.15, 0.25, 0, 1.57, 0],
            [0.25, -0.15, 0.145, 0, 1.57, 0],
        ]
        place_1 = [
            [0.25, -0.15, 0.35, 0, 1.57, 0],
            [0.25, 0.05, 0.35, 0, 1.57, 0],
            [0.25, 0.05, 0.2, 0, 1.57, 0],
        ]

        pick_2 = [
            [0.25, 0.05, 0.25, 0, 1.57, 0],
            [0.25, 0.15, 0.25, 0, 1.57, 0],
            [0.25, 0.15, 0.145, 0, 1.57, 0],
        ]
        place_2 = [
            [0.25, 0.15, 0.35, 0, 1.57, 0],
            [0.25, -0.05, 0.35, 0, 1.57, 0],
            [0.25, -0.05, 0.2, 0, 1.57, 0],
        ]

        end_pose = [
            [0.25, -0.05, 0.25, 0, 1.57, 0],
            [0.2, 0, 0.3, 0, 1.57, 0],
        ]

        n.release_with_tool()
        n.execute_trajectory_from_poses(pick_1, 0.002)
        open_gripper()
        n.execute_trajectory_from_poses(place_1, 0.002)
        n.release_with_tool()
        n.execute_trajectory_from_poses(pick_2, 0.002)
        close_gripper()
        n.execute_trajectory_from_poses(place_2, 0.002)
        n.execute_trajectory_from_poses(end_pose, 0.002)

        print("Wave")
        french_flag()
        n.move_joints(*(6 * [0]))
        # Niryo wave
        n.move_pose(*full_traj[0])
        n.set_arm_max_velocity(50)
        n.execute_moveit_robot_trajectory(niryo_wave_traj)

        n.wait(0.5)
        n.set_arm_max_velocity(100)

    except NiryoRosWrapperException as e:
        print e
        break
        # handle exception here
        # you can also make a try/except for each command separately

print "--- End"
sys.exit()
