#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, Pose, PoseMetadata, ArmMoveCommand

import math

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Create a frame
frame_name = "dynamic_demo_frame"
frame_description = "This is my fancy dynamic demo frame"
point_o = [0.15, 0.15, 0]  # Frame origin
point_x = [0.25, 0.2, 0]  # x-axis
point_y = [0.2, 0.25, 0]  # y-axis

robot.save_dynamic_frame_from_points(frame_name, frame_description, [point_o, point_x, point_y])

# Get the list of saved frames
dynamic_frames = robot.get_saved_dynamic_frame_list()
print(dynamic_frames)

# Get informations about an existing frame (name, description, tranform relative to the base world frame)
info = robot.get_saved_dynamic_frame(frame_name)
print(info)

# Move to the frame pose
frame_pose = Pose(0, 0, 0, 0, math.pi, -math.pi / 2, metadata=PoseMetadata.v2(frame=frame_name))
robot.move(frame_pose)

robot.move(Pose(0.2, 0.1, 0.14, 0.0, math.pi, -math.pi / 2, metadata=PoseMetadata.v2(frame=frame_name)))

# Move relatively to the frame
relative_pose1 = Pose(0, 0, 0.1, 0, 0, 0, metadata=PoseMetadata.v2(frame=frame_name))
robot.move(relative_pose1, move_cmd=ArmMoveCommand.LINEAR_POSE)

relative_pose2 = Pose(0.1, 0, 0, 0, 0, 0, metadata=PoseMetadata.v2(frame=frame_name))
robot.move(relative_pose2)

relative_pose3 = Pose(0, 0, -0.1, 0, 0, 0, metadata=PoseMetadata.v2(frame=frame_name))
robot.move(relative_pose3, move_cmd=ArmMoveCommand.LINEAR_POSE)

# Go to home position
robot.move_to_sleep_pose()

# Delete frame
robot.delete_dynamic_frame(frame_name)
