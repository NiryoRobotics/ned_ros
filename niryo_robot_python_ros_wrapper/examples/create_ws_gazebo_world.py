#!/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy


def create_wks_gazebo(niryo):
    workspace_name = "gazebo_1"
    workspace_h = 0.001
    point_1 = [0.3369, 0.087, workspace_h]
    point_2 = [point_1[0], -point_1[1], workspace_h]
    point_3 = [0.163, -point_1[1], workspace_h]
    point_4 = [point_3[0], point_1[1], workspace_h]
    niryo.save_workspace_from_points(workspace_name, [point_1, point_2, point_3, point_4])


if __name__ == '__main__':
    print "--- Start"
    try:
        rospy.init_node('niryo_robot_example_python_ros_wrapper')
        n = NiryoRosWrapper()
        # Calibrate robot first
        # n.calibrate_auto()
        # print "Calibration finished !"

        print "creating workspace ..."
        create_wks_gazebo(n)
        # print "Looking down ..."
        # n.move_joints([0, 0, 0, 0, -1.57, 0])

        # n.vision_pick(workspace_name, 0.1, ObjectShape.ANY, ObjectColor.BLUE)
    except NiryoRosWrapperException as e:
        print e

    print "--- End"
