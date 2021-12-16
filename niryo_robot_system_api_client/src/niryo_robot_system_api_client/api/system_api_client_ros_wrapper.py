import rospy

# - Services
from niryo_robot_system_api_client.srv import ManageEthernet, ManageEthernetRequest


class SystemApiClientRosWrapperException(Exception):
    pass


class SystemApiClientRosWrapper(object):
    def __init__(self, service_timeout=1):
        self.__service_timeout = service_timeout

    # - Ethernet
    def __manage_ethernet(self, profile, ip, mask, gateway, dns):
        result = self.__call_service('/niryo_robot/ethernet/manage', ManageEthernet, profile, ip, mask, gateway, dns)
        return self.__classic_return_w_check(result)

    def set_ethernet_static(self):
        """
        Change the ethernet profile to static (169.254.200.200)

        Example ::

            system_api_client.set_ethernet_static()

        :return: status, message
        :rtype: (int, str)
        """
        return self.__manage_ethernet(ManageEthernetRequest.STATIC, '', '', '', '')

    def set_ethernet_auto(self):
        """
        Change the ethernet profile to auto (DHCP).

        Example ::

            system_api_client.set_ethernet_auto()

        :return: status, message
        :rtype: (int, str)
        """
        return self.__manage_ethernet(ManageEthernetRequest.AUTO, '', '', '', '')

    def set_ethernet_custom(self, ip, mask, gateway, dns):
        """
        Change the ethernet profile to custom.

        Example ::

            system_api_client.set_ethernet_static('192.168.1.20', '255.255.255.0', '192.168.1.1', '192.168.1.1')

        :param ip: the ip address of the robot
        :type ip: str
        :param mask: the netmask of the ip address
        :type mask: str
        :param gateway: the gateway's ip address of the network
        :type gateway: str
        :param dns: the dns server's ip address, usually the same as the gateway
        :type dns: str
        :return: status, message
        :rtype: (int, str)
        """
        return self.__manage_ethernet(ManageEthernetRequest.CUSTOM, ip, mask, gateway, dns)

    # --- Functions interface
    def __call_service(self, service_name, service_msg_type, *args):
        """
        Wait for the service called service_name
        Then call the service with args

        :param service_name:
        :param service_msg_type:
        :param args: Tuple of arguments
        :raises SystemApiClientRosWrapperException: Timeout during waiting of services
        :return: Response
        """
        # Connect to service
        try:
            rospy.wait_for_service(service_name, self.__service_timeout)
        except rospy.ROSException as e:
            raise SystemApiClientRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise SystemApiClientRosWrapperException(e)

    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise SystemApiClientRosWrapperException(
                "Error Code : {}\nMessage : {}".format(result.status, result.message)
            )
