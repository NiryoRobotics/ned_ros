import rospy

# - Services
from niryo_robot_reports.srv import CheckConnection


class ReportsRosWrapperException(Exception):
    pass


class ReportsRosWrapper(object):
    def __init__(self, service_timeout=1):
        self.__service_timeout = service_timeout

    # - Cloud
    def check_connection(self, service='test_reports'):
        """
        Check if a cloud service is reachable

        Example ::

            reports.check_connection('test_reports')

        :param service: the name of the service
        :type name: str
        :return: status, success
        :rtype: (int, bool)
        """
        result = self.__call_service('/niryo_robot_reports/check_connection', CheckConnection, service)
        return self.__classic_return_w_check(result)

    # --- Functions interface
    def __call_service(self, service_name, service_msg_type, *args):
        """
        Wait for the service called service_name
        Then call the service with args

        :param service_name:
        :param service_msg_type:
        :param args: Tuple of arguments
        :raises DatabaseRosWrapperException: Timeout during waiting of services
        :return: Response
        """
        # Connect to service
        try:
            rospy.wait_for_service(service_name, self.__service_timeout)
        except rospy.ROSException as e:
            raise ReportsRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise ReportsRosWrapperException(e)

    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise ReportsRosWrapperException("Error Code : {}\nMessage : {}".format(result.status, result.message))
