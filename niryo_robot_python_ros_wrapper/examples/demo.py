#!/usr/bin/env python3

import rospy

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_sound.api.sound_ros_wrapper import SoundRosWrapperException
from niryo_robot_utils.dataclasses.JointsPosition import JointsPosition
from niryo_robot_utils.dataclasses.Pose import Pose
from niryo_robot_utils.dataclasses.PoseMetadata import PoseMetadata

BLUE = [15, 50, 255]
YELLOW = [255, 255, 0]


def say(robot: NiryoRosWrapper, text):
    try:
        robot.sound.say(text, 1)
    except SoundRosWrapperException:
        rospy.logwarn('Text To Speech is not available')


if __name__ == '__main__':
    rospy.init_node('niryo_demo_ros_wrapper')
    robot = NiryoRosWrapper()

    say(robot, "Programme demo")

    robot.request_new_calibration()
    robot.calibrate_auto()
    robot.move_to_sleep_pose()

    robot.move(Pose(0.3, 0, 0.2, 0, 1.57, 0, metadata=PoseMetadata.v1()))
    robot.move_spiral(0.15, 5, 216, 3)

    robot.led_ring.rainbow_cycle()

    default_joint_pose = [0, 0, 0, 0, 0, 0]

    if robot.get_hardware_version() == 'ned2':
        j_limit_1m = -2.86
        j_limit_1M = 2.86
        j_limit_2m = -1.57
        j_limit_2M = 0.54
        j_limit_3m = -1.29
        j_limit_3M = 1.50
        j_limit_4m = -2.02
        j_limit_4M = 2.02
        j_limit_5m = -1.83
        j_limit_5M = 1.83
        j_limit_6m = -2.51
        j_limit_6M = 2.51
    elif robot.get_hardware_version() == 'ned3pro':
        j_limit_1m = -2.96
        j_limit_1M = 2.95
        j_limit_2m = -1.75
        j_limit_2M = 0.44
        j_limit_3m = -1.22
        j_limit_3M = 1.57
        j_limit_4m = -2.09
        j_limit_4M = 2.09
        j_limit_5m = -1.92
        j_limit_5M = 1.92
        j_limit_6m = -3.14
        j_limit_6M = 3.14
    else:
        raise ValueError(f"Unsupported hardware version: {robot.get_hardware_version()}")

    first_target = [j_limit_1m, 0, 0, j_limit_4m, j_limit_5m, j_limit_6m]
    second_target = [0, j_limit_2M, j_limit_3m, 0, 0, 0]
    third_target = [j_limit_1M, 0, 0, j_limit_4M, j_limit_5M, j_limit_6M]
    last_target = [0, 0, j_limit_3M, 0, j_limit_5m, 0]

    poses = [
        (default_joint_pose, 1, 3),
        (first_target, 1, 4),
        (default_joint_pose, 1, 4),
        (second_target, 1, 3),
        (default_joint_pose, 1, 3),
        (third_target, 1, 4),
        (last_target, 1, 4),
    ]

    for joint_position, precision, duration in poses:
        robot.move_without_moveit(joint_position, duration)

    robot.led_ring.rainbow_cycle()

    waypoints = [
        [0.16, 0.00, -0.75, -0.56, 0.60, -2.26],
        [2.25, -0.25, -0.90, 1.54, -1.70, 1.70],
        [1.40, 0.35, -0.34, -1.24, -1.23, -0.10],
        [0.00, 0.60, 0.46, -1.55, -0.15, 2.50],
        [-1.0, 0.00, -1.00, -1.70, -1.35, -0.14],
    ]

    for waypoint_index, waypoint in enumerate(waypoints):
        robot.move_without_moveit(waypoint, 4)

    robot.move(JointsPosition(0, 0, 0, 0, 0, 0))

    robot.update_tool()
    robot.enable_tcp(True)

    z_offset = 0.15 if robot.get_current_tool_id() <= 0 else 0.02
    sleep_pose = Pose(0.3, 0, 0.3, 0, 1.57, 0, metadata=PoseMetadata.v1())
    home_pose = Pose(0.3, 0, 0.3, 0, 0, 0, metadata=PoseMetadata.v1())
    pick_1 = Pose(0, 0.2, z_offset, 0, 1.57, 0, metadata=PoseMetadata.v1())
    pick_2 = Pose(0, -0.2, z_offset, 0, 1.57, 0, metadata=PoseMetadata.v1())
    place_1 = Pose(0.15, 0, z_offset, 0, 1.57, 0, metadata=PoseMetadata.v1())
    place_2 = Pose(0.22, 0, z_offset, 0, 1.57, 0, metadata=PoseMetadata.v1())

    robot.move(sleep_pose)

    robot.pick(pick_1)
    robot.place(place_1)

    robot.move(sleep_pose)

    robot.pick(pick_2)
    robot.place(place_2)

    robot.move(sleep_pose)

    robot.pick(place_1)
    pick_1.yaw = 1.57
    robot.place(pick_1)

    robot.move(sleep_pose)

    robot.pick(place_2)
    pick_2.yaw = -1.57
    robot.place(pick_2)

    robot.grasp_with_tool()

    robot.move(home_pose)

    robot.enable_tcp(False)

    say(robot, "Fin de la demo")

    robot.led_ring.solid(BLUE)
    robot.move_to_sleep_pose()
    robot.set_arm_max_velocity(100)
    robot.set_arm_max_acceleration(100)
