#!/usr/bin/env python

import rospy
import actionlib

# Messages
from niryo_robot_commander.msg import RobotCommand
from niryo_robot_user_interface.msg import MatlabMoveResult

# Action
from niryo_robot_commander.msg import RobotMoveAction
from niryo_robot_commander.msg import RobotMoveGoal


class MatlabManager:

    def __init__(self):
        rospy.Subscriber('/niryo_robot_matlab/command', RobotCommand,
                         self.__callback_matlab_node)
        self.__matlab_move_publisher = rospy.Publisher('/niryo_robot_matlab/result', MatlabMoveResult,
                                                       queue_size=10)
        self.__action_client_matlab = actionlib.SimpleActionClient('/niryo_robot_commander/robot_action',
                                                                   RobotMoveAction)
        rospy.logdebug('Matlab Manager - Started')

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def __callback_matlab_node(self, request):
        rospy.loginfo("Matlab Manager - Receive Goal")
        cmd = request.joints
        cmd_type = request.cmd_type
        response = self.__send_matlab_goal(cmd, cmd_type)
        msg = MatlabMoveResult()
        msg.status = response.status
        msg.message = response.message
        rospy.loginfo(msg)
        rospy.sleep(0.2)
        self.__matlab_move_publisher.publish(msg)
        rospy.loginfo("Matlab Manager - Move Result Published")

    def __send_matlab_goal(self, cmd, cmd_type):
        self.__action_client_matlab.wait_for_server()
        goal = RobotMoveGoal()
        goal.cmd.joints = cmd
        goal.cmd.cmd_type = cmd_type
        self.__action_client_matlab.send_goal(goal)
        rospy.loginfo("Matlab Manager - Waiting for result")
        self.__action_client_matlab.wait_for_result()
        response = self.__action_client_matlab.get_result()
        rospy.logdebug("Matlab Manager - Result : {}".format(response))
        return response
