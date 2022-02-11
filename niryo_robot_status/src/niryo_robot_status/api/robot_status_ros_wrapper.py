import rospy

# - Services
from niryo_robot_msgs.srv import AdvertiseShutdown, AdvertiseShutdownRequest


# Command Status
# from niryo_robot_msgs.msg import CommandStatus

class RobotStatusRosWrapperException(Exception):
    pass


class RobotStatusRosWrapper(object):
    def __init__(self, service_timeout=1):
        self.__service_timeout = service_timeout

    def shutdown(self):
        res = self.__call_service('/niryo_robot_rpi/shutdown_rpi', AdvertiseShutdown,
                                  AdvertiseShutdownRequest.SHUTDOWN)
        return self.__classic_return_w_check(res)

    def reboot(self):
        res = self.__call_service('/niryo_robot_rpi/shutdown_rpi', AdvertiseShutdown,
                                  AdvertiseShutdownRequest.REBOOT)
        return self.__classic_return_w_check(res)

    def prepare_update(self):
        try:
            res = self.__call_service('/niryo_robot_status/advertise_shutdown', AdvertiseShutdown,
                                      AdvertiseShutdownRequest.UPDATE)

        except RobotStatusRosWrapperException:
            return False

        return res.status < 0

    # --- Functions interface
    def __call_service(self, service_name, service_msg_type, *args):
        """
        Wait for the service called service_name
        Then call the service with args

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
            raise RobotStatusRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise RobotStatusRosWrapperException(e)

    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise RobotStatusRosWrapperException("Error Code : {}\nMessage : {}".format(result.status, result.message))
