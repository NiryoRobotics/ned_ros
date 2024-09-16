#!/usr/bin/env python

# Libs
import rospy
from niryo_robot_utils import sentry_init

from niryo_robot_metrics.PsutilWrapper import PsutilWrapper
from niryo_robot_metrics.TuptimeWrapper import TuptimeWrapper

# msg
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_metrics.msg import Metric

# srv
from niryo_robot_msgs.srv import GetStringList
from niryo_robot_metrics.srv import GetMetric


def metric_to_ros_msg(metric):
    return Metric(name=metric['name'], value=str(metric['value']), unit=metric['unit'])


class MetricsNode:

    def __init__(self):
        rospy.logdebug("Metrics Node - Entering in Init")

        self.__metrics_wrappers = [PsutilWrapper(), TuptimeWrapper()]

        rospy.Service('~get_metric', GetMetric, self.__get_metric_callback)
        rospy.Service('~available_metrics', GetStringList, self.__available_metrics_callback)

        # Set a bool to mention that this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Reports Node - Node Started")

    def __get_metric_callback(self, req):
        for metric_wrapper in self.__metrics_wrappers:
            try:
                metric = metric_wrapper[req.name]
                return CommandStatus.SUCCESS, metric_to_ros_msg(metric)
            except ValueError:
                pass
        return CommandStatus.FAILURE, Metric()

    def __available_metrics_callback(self, _):
        available_metrics = []
        for metric_wrapper in self.__metrics_wrappers:
            available_metrics += metric_wrapper.available_metrics
        return available_metrics


if __name__ == "__main__":
    sentry_init()

    rospy.init_node('niryo_robot_metrics', anonymous=False, log_level=rospy.INFO)

    try:
        node = MetricsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
