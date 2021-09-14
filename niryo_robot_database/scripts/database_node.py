#!/usr/bin/env python

# Libs
import os
import rospy
from sqlite3 import OperationalError

from niryo_robot_database.SQLiteDAO import SQLiteDAO
from niryo_robot_database.Logs import Logs
from niryo_robot_database.Metrics import Metrics
from niryo_robot_database.Settings import Settings

# msg
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_database.msg import Log, Metric

# srv
from niryo_robot_database.srv import AddLog, GetAllLogs, GetAllMetrics, RmLogSinceDate, SetMetric, SetSettings, GetSettings


class DatabaseNode:
    def __init__(self):
        rospy.logdebug("Database Node - Entering in Init")

        db_path = os.path.expanduser(rospy.get_param('~sqlite_db_file_path'))

        try:
            sqlite_dao = SQLiteDAO(db_path)
        except IOError:
            rospy.logerr(
                'Database Node - Unable to open the database. Did you run sql/init.sh ?'
            )

        self.__settings = Settings(sqlite_dao)
        self.__logs = Logs(sqlite_dao)
        self.__metrics = Metrics(sqlite_dao)

        rospy.Service('~logs/add', AddLog, self.__callback_add_log)
        rospy.Service('~logs/get_all', GetAllLogs, self.__callback_get_all_logs)
        rospy.Service(
            '~metrics/get_all', GetAllMetrics, self.__callback_get_all_metrics
        )
        rospy.Service(
            '~logs/rm_since_date', RmLogSinceDate,
            self.__callback_rm_logs_since_date
        )
        rospy.Service('~metrics/set', SetMetric, self.__callback_set_metric)

        rospy.Service(
            '~settings/set', SetSettings, self.__callback_set_settings
        )
        rospy.Service(
            '~settings/get', GetSettings, self.__callback_get_settings
        )

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Database Node - Node Started")

    def __callback_add_log(self, req):
        log = req.log
        try:
            self.__logs.add(log.date, log.severity, log.origin, log.message)
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, 'Log added'

    def __callback_get_all_logs(self, _req):
        try:
            logs = [
                Log(
                    x['id'], x['date'], x['severity'], x['origin'], x['message']
                ) for x in self.__logs.get_all()
            ]
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, logs

    def __callback_get_all_metrics(self, _req):
        try:
            metrics = [
                Metric(x['id'], x['name'], x['value'], x['update_date'])
                for x in self.__metrics.get_all()
            ]
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, metrics

    def __callback_rm_logs_since_date(self, req):
        try:
            self.__logs.rm_all_since_date(req.date)
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, 'Logs deleted'

    def __callback_set_metric(self, req):
        try:
            self.__metrics.set(req.name, req.value)
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, 'Metric successfully set'

    def __callback_set_settings(self, req):
        try:
            self.__settings.set(req.name, req.value, req.type)
        except TypeError as e:
            return CommandStatus.DATABASE_SETTINGS_TYPE_MISMATCH, e.__str__()
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, 'Settings successfully set'

    def __callback_get_settings(self, req):
        try:
            value, value_type = self.__settings.get(req.name)
        except Settings.UnknownSettingsException():
            return CommandStatus.DATABASE_SETTINGS_UNKNOWN, 'Unknown settings', None
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, value, value_type


if __name__ == "__main__":
    rospy.init_node(
        'niryo_robot_database', anonymous=False, log_level=rospy.INFO
    )
    try:
        node = DatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
