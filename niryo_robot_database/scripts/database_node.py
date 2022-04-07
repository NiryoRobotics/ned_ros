#!/usr/bin/env python

# Libs
import os
import subprocess

import rospy
import rospkg
from sqlite3 import OperationalError

from niryo_robot_database.SQLiteDAO import SQLiteDAO
from niryo_robot_database.Settings import Settings, UnknownSettingsException
from niryo_robot_database.FilePath import FilePath, UnknownFilePathException
from niryo_robot_database.RobotDatabase import RobotDatabase

# msg
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_database.msg import FilePath as FilePathMsg, Setting as SettingMsg

# srv
from niryo_robot_database.srv import SetSettings, GetSettings, AddFilePath, GetAllByType, RmFilePath


class DatabaseNode:
    def __init__(self):
        rospy.logdebug("Database Node - Entering in Init")

        sqlite_db_file_path = os.path.join(rospy.get_param('~database_path'),
                                           rospy.get_param('~hardware_version') + ".db")
        db_path = os.path.expanduser(sqlite_db_file_path)

        if not os.path.isfile(db_path):
            package_path = rospkg.RosPack().get_path('niryo_robot_database')
            file_path = os.path.join(package_path, 'sql', 'init.sh')
            subprocess.call(['bash', file_path])
            if not os.path.isfile(db_path):
                rospy.logerr('Database Node - Unable to open the database. Did you run sql/init.sh ?')

        sqlite_dao = SQLiteDAO(db_path)

        self.__settings = Settings(sqlite_dao)
        self.__file_paths = FilePath(sqlite_dao)
        self.__robot_db = RobotDatabase(sqlite_dao)

        rospy.Service('~settings/set', SetSettings, self.__callback_set_settings)
        rospy.Service('~settings/get', GetSettings, self.__callback_get_settings)
        rospy.Service('~file_paths/add', AddFilePath, self.__callback_add_file_path)
        rospy.Service('~file_paths/rm', RmFilePath, self.__callback_rm_file_path)
        rospy.Service('~file_paths/get_all_by_type', GetAllByType, self.__callback_get_all_by_type, )

        self.__setting_update_publisher = rospy.Publisher('~setting_update', SettingMsg, queue_size=5)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Database Node - Node Started")

    def __callback_set_settings(self, req):
        try:
            self.__settings.set(req.name, req.value, req.type)
        except TypeError as e:
            return CommandStatus.DATABASE_SETTINGS_TYPE_MISMATCH, e.__str__()
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)

        self.__setting_update_publisher.publish(req.name, req.value, req.type)
        return CommandStatus.SUCCESS, 'Settings successfully set'

    def __callback_get_settings(self, req):
        try:
            value, value_type = self.__settings.get(req.name)
        except UnknownSettingsException:
            return CommandStatus.DATABASE_SETTINGS_UNKNOWN, 'Unknown settings', None
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e), None
        return CommandStatus.SUCCESS, value, value_type

    def __callback_add_file_path(self, req):
        try:
            row_id = self.__file_paths.add_file_path(
                req.type, req.name, req.path
            )
        except OperationalError as e:
            return CommandStatus.DATABASE_DB_ERROR, str(e)
        return CommandStatus.SUCCESS, str(row_id)

    def __callback_get_all_by_type(self, req):
        try:
            filepaths = self.__file_paths.get_all_by_type(req.type)
        except OperationalError as e:
            rospy.logwarn('[Database Node] OperationalError: {}'.format(str(e)))
            return CommandStatus.DATABASE_DB_ERROR, []

        res = [
            FilePathMsg(id=x['id'], type=x['type'], name=x['name'], date=x['date'], path=x['path'])
            for x in filepaths
        ]

        return CommandStatus.SUCCESS, res

    def __callback_rm_file_path(self, req):
        self.__file_paths.rm_file_path(req.id)
        return CommandStatus.SUCCESS, 'Successfully deleted'


if __name__ == "__main__":
    rospy.init_node('niryo_robot_database', anonymous=False, log_level=rospy.INFO)
    try:
        node = DatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
