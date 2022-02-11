# !/usr/bin/env python

from niryo_robot_python_ros_wrapper.ros_wrapper import *
import sys
import rospy

rospy.init_node('niryo_blockly_interpreted_code')
n = NiryoRosWrapper()

n.calibrate_auto()

try:
    n.move_pose(*n.get_pose_saved("backhome ++"))
    n.wait(3)
    n.move_pose(*n.get_pose_saved("Test min J1"))
    n.wait(3)
    n.move_joints(*[(-1.5), (-0.9), 0.301, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[2.5, (-0.9), 0.301, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[2.5, 0.43, 0.301, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[2.5, 0.2, 0.301, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[2.5, 0.2, (-0.91), 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[2.5, 0.2, (-0.6), 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[0, (-0.9), 0.301, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[0, 0.15, (-0.28), 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[0, 0.3, 0.7, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[0, (-1.14), 0.5, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[(-2), (-0.9), 0.301, 0, (-1), 0.005])
    n.wait(3)
    n.move_joints(*[0, 0.3, (-0.28), 0, (-1), 0.005])

except NiryoRosWrapperException as e:
    sys.stderr.write(str(e))
