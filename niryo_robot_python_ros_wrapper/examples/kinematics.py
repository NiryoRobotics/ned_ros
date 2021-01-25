#!/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import rospy

rospy.init_node('niryo_robot_example_python_ros_wrapper')

print "--- Start"
n = NiryoRosWrapper()

try:
    start_pose = n.get_pose_as_list()
    print "Starting Pose " + str(start_pose)
    pose = n.forward_kinematics(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    n.move_pose(*pose)
    joints_target = n.inverse_kinematics(*start_pose)
    print "Joints targets " + str(joints_target)
    n.move_joints(*joints_target)
except NiryoRosWrapperException as e:
    print e

print "--- End"
