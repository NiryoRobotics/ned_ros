#!/usr/bin/env python3

import rospy

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper

if __name__ == '__main__':
    # This script is used to wait for the Niryo Robot stack to be ready before building the documentation.
    rospy.init_node('wait_for_nodes_initialization', anonymous=True)
    NiryoRosWrapper.wait_for_nodes_initialization(simulation_mode=True)