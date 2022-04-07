#!/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy
import random
import copy
import sys


def create_wks_gazebo(niryo):
    workspace_name = "gazebo_1"
    if workspace_name in niryo.get_workspace_list():
        niryo.delete_workspace(workspace_name)

    workspace_pose = [0.25, 0.0, 0.001]
    point_1 = [workspace_pose[0] + 0.087, workspace_pose[1] + 0.087, workspace_pose[2]]
    point_2 = [workspace_pose[0] + 0.087, workspace_pose[1] - 0.087, workspace_pose[2]]
    point_3 = [workspace_pose[0] - 0.087, workspace_pose[1] - 0.087, workspace_pose[2]]
    point_4 = [workspace_pose[0] - 0.087, workspace_pose[1] + 0.087, workspace_pose[2]]

    niryo.save_workspace_from_points(workspace_name, [point_1, point_2, point_3, point_4])


def create_wks2_gazebo(niryo):
    workspace_name = "gazebo_2"

    if workspace_name in niryo.get_workspace_list():
        niryo.delete_workspace(workspace_name)

    workspace_pose = [0.0, 0.25, 0.001]
    point_1 = [workspace_pose[0] - 0.0915, workspace_pose[1] + 0.0915, workspace_pose[2]]
    point_2 = [workspace_pose[0] + 0.0915, workspace_pose[1] + 0.0915, workspace_pose[2]]
    point_3 = [workspace_pose[0] + 0.0915, workspace_pose[1] - 0.0915, workspace_pose[2]]
    point_4 = [workspace_pose[0] - 0.0915, workspace_pose[1] - 0.0915, workspace_pose[2]]

    niryo.save_workspace_from_points(workspace_name, [point_1, point_2, point_3, point_4])


if __name__ == '__main__':
    print "--- Start"
    try:
        rospy.init_node('niryo_robot_example_python_ros_wrapper')
        n = NiryoRosWrapper()

        n.calibrate_auto()
        print "Calibration finished !"
        n.update_tool()

        print "creating workspaces ..."
        create_wks_gazebo(n)
        create_wks2_gazebo(n)
        print "Workspaces created ..."

        center_pose_g2 = n.get_target_pose_from_rel("gazebo_2", 0.005, 0.5, 0.5, 0.0)

        g2_poses = []
        for delta_x in [-0.0325, 0, 0.0325]:
            for delta_y in [-0.0325, 0, 0.0325]:
                tmp = copy.deepcopy(center_pose_g2)
                tmp.position.x += delta_x
                tmp.position.y += delta_y
                tmp.position.z += 0.13
                g2_poses.append(copy.deepcopy(tmp))

        obs_joints = [0, 0.4, -0.4, 0, -1.57, 0]
        obs_joints2 = [1.57, 0.4, -0.4, 0, -1.57, 0]

        while True:
            try:
                available_poses = g2_poses[:]
                while n.vision_pick_w_obs_joints("gazebo_1", 0.002, ObjectShape.ANY, ObjectColor.ANY, obs_joints)[0]:
                    print "Object picked ..."
                    random_index = random.choice(range(len(available_poses)))
                    print "Place on grid ..."
                    place_pose = available_poses.pop(random_index)
                    n.place_from_pose(place_pose.position.x, place_pose.position.y, place_pose.position.z,
                                      place_pose.rpy.roll, place_pose.rpy.pitch, place_pose.rpy.yaw)

                while n.vision_pick_w_obs_joints("gazebo_2", 0.002, ObjectShape.ANY, ObjectColor.ANY, obs_joints2)[0]:
                    print "Object picked ..."
                    pose = n.get_target_pose_from_rel("gazebo_1", 0.005, 0.15 + random.random() * 0.7,
                                                      0.15 + random.random() * 0.7, 0.0)
                    print "Place randomly on the restricted workspace area ..."
                    n.place_from_pose(pose.position.x, pose.position.y, pose.position.z,
                                      pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw)
            except NiryoRosWrapperException:
                break
            except Exception as e:
                print e

    except Exception as e:
        print e

    print "--- End"
