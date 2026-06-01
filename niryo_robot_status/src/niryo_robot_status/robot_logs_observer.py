import json

import rospy

from .robot_status_enums import *

# - Messages
from rosgraph_msgs.msg import Log

BLACKLIST = [
    (Log.WARN, "Inbound TCP/IP connection failed"),
]


class RobotLogsObserver(object):

    def __init__(self, robot_status_handler):
        """

        :type robot_status_handler: RobotStatusHandler
        """
        self.__robot_status_handler = robot_status_handler

        self.__log_nodes = rospy.get_param("~node_logs")
        self.__log_status = LOG_LEVEL_TO_MSG[Log.INFO]
        self.__log_msg = ""

        # - Subscribers
        self.__log_sub = rospy.Subscriber('/rosout_agg', Log, self.__callback_logs)

    def __callback_logs(self, msg):
        if msg.name not in self.__log_nodes:
            return

        if msg.level < Log.WARN:
            return

        for level, txt in BLACKLIST:
            if msg.level == level and txt in msg.msg:
                return

        if self.__log_status == LOG_LEVEL_TO_MSG[msg.level]:
            self.__log_msg = ""
            return

        self.__log_msg = f'[{LOG_LEVEL_TO_STR[msg.level]}] {msg.name}:{msg.file}.{msg.function}:{msg.line}: {msg.msg}'
        self.__log_status = LOG_LEVEL_TO_MSG[msg.level]
        self.__robot_status_handler.advertise_new_logs()

    @property
    def log_status(self):
        return self.__log_status

    @property
    def log_message(self):
        return self.__log_msg
