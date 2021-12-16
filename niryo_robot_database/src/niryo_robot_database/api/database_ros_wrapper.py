import rospy
from pydoc import locate

# - Services
from niryo_robot_database.srv import GetSettings, SetSettings

# Command Status
from niryo_robot_msgs.msg import CommandStatus


class DatabaseRosWrapperException(Exception):
    pass


class DatabaseRosWrapper(object):
    def __init__(self, service_timeout=1):
        self.__service_timeout = service_timeout

    # - Settings
    def get_setting(self, name):
        """
        Retrieve a setting from the database

        Example ::

            database.get_setting('purge_ros_logs_on_startup')

        :param name: the name of the setting
        :type name: str
        :return: the value of the setting
        :rtype: object
        """
        result = self.__call_service('/niryo_robot_database/settings/get', GetSettings, name)
        if result.status == CommandStatus.DATABASE_SETTINGS_UNKNOWN:
            return None
        if result.type == 'bool':
            casted_type = result.value in ['True', 'true']
        else:
            casted_type = locate(result.type)(result.value)
        return casted_type

    def set_setting(self, name, value):
        """
        Set a setting in the database

        Example ::

            database.set_setting('purge_ros_logs_on_startup', True)

        :param name: the name of a setting
        :type name: str
        :param value: the value of the setting
        :type value: object
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_database/settings/set', SetSettings,
                                     name, str(value), type(value).__name__)
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
            raise DatabaseRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise DatabaseRosWrapperException(e)

    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise DatabaseRosWrapperException("Error Code : {}\nMessage : {}".format(result.status, result.message))
