#!/usr/bin/env python

# Libs
import rospy
import logging

from niryo_robot_metrics.TuptimeWrapper import TuptimeWrapper
from niryo_robot_metrics.DataSender import DataSender

# msg
from niryo_robot_msgs.msg import CommandStatus

# srv
from niryo_robot_metrics.srv import SaveAll, SendAll

from niryo_robot_credentials.srv import GetCredential
from niryo_robot_database.srv import SetMetric, GetAllMetrics, AddLog, GetAllLogs


class MetricsNode:
    def __init__(self):
        rospy.logdebug("Metrics Node - Entering in Init")

        self.__tuptime_wrapper = TuptimeWrapper()

        get_serial_number = rospy.ServiceProxy(
            '/niryo_robot_credentials/get_serial', GetCredential
        )
        get_api_key = rospy.ServiceProxy(
            '/niryo_robot_credentials/get_api_key', GetCredential
        )

        cloud_domain = rospy.get_param('~cloud_domain')
        serial_number_response = get_serial_number()
        api_key_response = get_api_key()

        self.__able_to_send = serial_number_response.status == api_key_response.status == CommandStatus.SUCCESS
        if not self.__able_to_send:
            rospy.logwarn('Metrics Node - Unable to retrieve the serial number')

        self.__data_sender = DataSender(
            cloud_domain,
            serial_number_response.message,
            api_key_response.message,
        )

        rospy.Service('~save_all', SaveAll, self.__callback_save_all)
        rospy.Service('~send_all', SendAll, self.__callback_send_all)

        self.__save_metric = rospy.ServiceProxy(
            '/niryo_robot_database/metrics/set', SetMetric
        )
        self.__get_all_metrics = rospy.ServiceProxy(
            '/niryo_robot_database/metrics/get_all', GetAllMetrics
        )
        self.__get_all_logs = rospy.ServiceProxy(
            '/niryo_robot_database/logs/get_all', GetAllLogs
        )

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Metrics Node - Node Started")

    # - callbacks

    def __callback_save_all(self, _req):
        self.fetch_and_save_metrics()
        return CommandStatus.SUCCESS, 'Metrics successfully saved'

    def __callback_send_all(self, _req):
        self.send_datas()
        return CommandStatus.SUCCESS, 'Metrics successfully sent'

    # - functions

    def fetch_and_save_metrics(self):
        updated_metrics = self.__tuptime_wrapper.get_tuptime_datas()
        for key, value in updated_metrics.iteritems():
            self.__save_metric(key, str(value))

    def send_datas(self):
        if not self.__able_to_send:
            rospy.logerr(
                'Metrics Node - Unable to send the datas because there was a problem during the initialization'
            )
            return CommandStatus.METRICS_UNABLE_TO_SEND, 'Unable to send the datas'
        metrics_response = self.__get_all_metrics()
        logs_response = self.__get_all_logs()
        if metrics_response.status == CommandStatus.SUCCESS:
            self.__data_sender.send_metrics(metrics_response.metrics)
        else:
            rospy.logwarn('Metrics Node - Unable to retrieve the metrics')
        if logs_response.status == CommandStatus.SUCCESS:
            self.__data_sender.send_logs(logs_response.logs)
        else:
            rospy.logwarn('Metrics Node - Unable to retrieve the logs')


if __name__ == "__main__":
    rospy.init_node('niryo_robot_metrics', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = MetricsNode()
        rospy.Timer(rospy.Duration(600), node.fetch_and_save_metrics)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
