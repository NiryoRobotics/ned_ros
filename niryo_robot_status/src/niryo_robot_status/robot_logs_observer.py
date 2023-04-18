
import rospy

from .robot_status_enums import *

# - Messages
from rosgraph_msgs.msg import Log


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
        if msg.name in self.__log_nodes:
            if msg.level in [Log.WARN, Log.ERROR, Log.FATAL]:
                # Filter out warnings that don't matter
                if msg.level == Log.WARN and "Inbound TCP/IP connection failed" in msg.msg:
                    return

                self.__log_msg = 'level: {}\nnode: {}\nmsg: {}\nfile: {}\nfunction: {}\nline: {}\n' \
                                 ''.format(LOG_LEVEL_TO_STR[msg.level], msg.name, msg.msg,
                                           msg.file, msg.function, msg.line)

            elif self.__log_status == LOG_LEVEL_TO_MSG[msg.level]:
                self.__log_msg = ""
                return

            self.__log_status = LOG_LEVEL_TO_MSG[msg.level]
            self.__robot_status_handler.advertise_new_logs()

    @property
    def log_status(self):
        return self.__log_status

    @property
    def log_message(self):
        return self.__log_msg
