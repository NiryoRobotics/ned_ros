#!/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy


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

        obs_joints = [0, 0.4, -0.4, 0, -1.57, 0]
        cubes_cpt = 0
        while n.vision_pick_w_obs_joints("gazebo_1", 0.002, ObjectShape.ANY, ObjectColor.ANY, obs_joints)[0]:
            print "Object picked ..."
            pose = n.get_target_pose_from_rel("gazebo_2", 0.005 + cubes_cpt*0.02, 0.5, 0.5, 0.0)
            n.place_from_pose(pose.position.x, pose.position.y, pose.position.z,
                              pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw)
            cubes_cpt += 1

    except NiryoRosWrapperException as e:
        print e

    print "--- End"
