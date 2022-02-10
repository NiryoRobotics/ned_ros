#!/usr/bin/env python
# Lib
import rospy

# Enums
from niryo_robot_utils.niryo_ros_wrapper_exceptions import NiryoRosWrapperException


class NiryoTopicValue(object):

    def __init__(self, topic_name, topic_type, timeout=2, queue_size=None):
        self.__topic_name = topic_name
        self.__topic_type = topic_type
        self.__topic_queue_size = queue_size
        self.__timeout = timeout

        self.__topic = None
        self.__value = None

    def __call__(self, *args, **kwargs):
        return self.value

    def __launch_topic(self):
        self.__topic = rospy.Subscriber(self.__topic_name, self.__topic_type, self.__internal_callback,
                                        queue_size=self.__topic_queue_size)

    @property
    def topic(self):
        return self.__topic

    @property
    def value(self):
        if self.__topic is None:
            self.__launch_topic()

        if self.__value is None:
            return self.wait_for_message(timeout=self.__timeout)

        return self.__value

    def get_value(self):
        return self.value

    def wait_for_message(self, timeout=5):
        if self.__topic is None:
            self.__launch_topic()

        try:
            value = rospy.wait_for_message(self.__topic_name, self.__topic_type, timeout=timeout)
        except rospy.ROSException:
            raise NiryoRosWrapperException("Could not get data on the {} topic".format(self.__topic))
        return value

    def __internal_callback(self, msg):
        self.__value = msg
