#!/usr/bin/env python

import rospy

from pipeline_test_ros_n_pure_python import test_pipeline

if __name__ == '__main__':
    rospy.init_node('unit_tests_node')

    conveyor = rospy.get_param('~conveyor')
    simulation_mode = not rospy.get_param('~rpi')
    headless = rospy.get_param('~headless')
    gripper_n_camera = rospy.get_param('~gripper_n_camera')
    verbose = rospy.get_param('~verbose')

    gripper_n_camera_str = "true" if gripper_n_camera else "false"
    conveyor_str = "true" if conveyor else "false"
    gui = "false" if headless else "true"
    optional_args = [
        "conveyor:={}".format(conveyor_str),
        "gripper_n_camera:={}".format(gripper_n_camera_str),
        "gui:={}".format(gui),
    ]

    success_bool = test_pipeline(show_errors=verbose, use_simulation=simulation_mode,
                                 optional_args=optional_args)
