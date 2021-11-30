#!/usr/bin/env python

# Libs
import os
import subprocess

import rospy
import rospkg
from sqlite3 import OperationalError

from niryo_robot_database.SQLiteDAO import SQLiteDAO
from niryo_robot_database.Metrics import Metrics, UnknownMetricException
from niryo_robot_database.Settings import Settings, UnknownSettingsException
from niryo_robot_database.FilePath import FilePath, UnknownFilePathException

# msg
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_database.msg import Metric, FilePath as FilePathMsg

# srv
from niryo_robot_database.srv import GetAllMetrics, SetMetric, SetSettings, GetSettings, AddFilePath, GetAllByType


class DatabaseNode:
    def __init__(self):
        rospy.logdebug("Database Node - Entering in Init")

        db_path = os.path.expanduser(rospy.get_param('~sqlite_db_file_path'))

        if not os.path.isfile(db_path):
            package_path = rospkg.RosPack().get_path('niryo_robot_database')
            file_path = package_path + '/sql/init.sh'
            subprocess.call(['bash', file_path])
            if not os.path.isfile(db_path):
                rospy.logerr(
                    'Database Node - Unable to open the database. Did you run sql/init.sh ?'
                )

        sqlite_dao = SQLiteDAO(db_path)

        self.__settings = Settings(sqlite_dao)
        self.__metrics = Metrics(sqlite_dao)
        self.__file_paths = FilePath(sqlite_dao)

        rospy.Service(
            '~metrics/get_all', GetAllMetrics, self.__callback_get_all_metrics
        )
        rospy.Service('~metrics/set', SetMetric, self.__callback_set_metric)

        rospy.Service(
            '~settings/set', SetSettings, self.__callback_set_settings
        )
        rospy.Service(
            '~settings/get', GetSettings, self.__callback_get_settings
        )
        rospy.Service(
            '~file_paths/add', AddFilePath, self.__callback_add_file_path
        )
        rospy.Service(
            '~file_paths/get_all_by_type',
            GetAllByType,
            self.__callback_get_all_by_type,
        )

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Database Node - Node Started")

    def __callback_get_all_metrics(self, _req):
        try:
            metrics = [
                Metric(x['id'], x['name'], x['value'], x['update_date'])
                for x in self.__metrics.get_all()
            ]
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, metrics

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
        except UnknownSettingsException:
            return CommandStatus.DATABASE_SETTINGS_UNKNOWN, 'Unknown settings', None
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, value, value_type

    def __callback_add_file_path(self, req):
        try:
            self.__file_paths.add_file_path(req.type, req.name, req.path)
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, 'File path successfully added'

    def __callback_get_all_by_type(self, req):
        try:
            filepaths = self.__file_paths.get_all_by_type(req.type)
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        res = [FilePathMsg(x['id'], x['type'], x['name'], x['date'], x['path']) for x in filepaths]
        return CommandStatus.SUCCESS, res


if __name__ == "__main__":
    rospy.init_node(
        'niryo_robot_database', anonymous=False, log_level=rospy.INFO
    )
    try:
        node = DatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
