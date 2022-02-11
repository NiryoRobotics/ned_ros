#!/usr/bin/env python

import rospy

from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_utils.niryo_ros_wrapper_exceptions import NiryoRosWrapperException


class AbstractNiryoRosWrapper(object):
    LOGS_LEVELS = {
        rospy.INFO: 'INFO',
        rospy.WARN: 'WARNING',
        rospy.ERROR: 'ERROR',
        rospy.FATAL: 'FATAL',
        rospy.DEBUG: 'DEBUG'
    }

    def __init__(self, service_timeout=0.2):
        # - Getting ROS parameters
        self.__service_timeout = service_timeout

    def __del__(self):
        del self

    # -- Service & Action executors
    def _call_service(self, service_name, service_msg_type, *args):
        """
        Waits for the service called service_name
        Then calls the service with args

        :param service_name:
        :param service_msg_type:
        :param args: Tuple of arguments
        :raises NiryoRosWrapperException: Timeout during waiting of services
        :return: Response
        """
        # Connect to service
        try:
            rospy.wait_for_service(service_name, self.__service_timeout)
        except rospy.ROSException as e:
            raise NiryoRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise NiryoRosWrapperException(e)

    # --- Functions interface
    def _classic_return_w_check(self, result):
        self._check_result_status(result)
        return result.status, result.message

    @staticmethod
    def _check_result_status(result):
        if result.status < 0:
            raise NiryoRosWrapperException("Error Code : {}\nMessage : {}".format(result.status, result.message))

    @staticmethod
    def return_success(message=""):
        return CommandStatus.SUCCESS, message
