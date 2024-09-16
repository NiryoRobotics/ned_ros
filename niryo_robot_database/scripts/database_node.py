#!/usr/bin/env python

# Libs
import os
import time

import rospy
from sqlite3 import OperationalError
from niryo_robot_utils import sentry_init

from niryo_robot_system_api_client import system_api_client

from niryo_robot_database.SQLiteDAO import SQLiteDAO
from niryo_robot_database.FilePath import FilePath, UnknownFilePathException
from niryo_robot_database.Version import Version, UnknownVersionException

# msg
from niryo_robot_msgs.msg import CommandStatus, SoftwareVersion
from niryo_robot_database.msg import FilePath as FilePathMsg, Setting as SettingMsg

# srv
from niryo_robot_database.srv import SetSettings, GetSettings, AddFilePath, GetAllByType, RmFilePath
from niryo_robot_msgs.srv import GetString


class DatabaseNode:

    def __init__(self):
        rospy.logdebug("Database Node - Entering in Init")

        sqlite_db_file_path = os.path.join(rospy.get_param('~database_path'),
                                           rospy.get_param('~hardware_version') + ".db")
        self.db_path = os.path.expanduser(sqlite_db_file_path)

        if not os.path.isfile(self.db_path):
            raise RuntimeError('Database Node - Unable to open the database.')

        sqlite_dao = SQLiteDAO(self.db_path)

        rospy.Service('~settings/set', SetSettings, self.__callback_set_settings)
        rospy.Service('~settings/get', GetSettings, self.__callback_get_settings)
        self.__setting_update_publisher = rospy.Publisher('~setting_update', SettingMsg, queue_size=5)

        self.__file_paths = FilePath(sqlite_dao)
        rospy.Service('~file_paths/add', AddFilePath, self.__callback_add_file_path)
        rospy.Service('~file_paths/rm', RmFilePath, self.__callback_rm_file_path)
        rospy.Service(
            '~file_paths/get_all_by_type',
            GetAllByType,
            self.__callback_get_all_by_type,
        )
        rospy.Service('~get_db_file_path', GetString, lambda _: self.db_path)

        self.__version = Version(sqlite_dao)

        rospy.Subscriber('/niryo_robot_hardware_interface/software_version',
                         SoftwareVersion,
                         self.__sw_callback,
                         queue_size=1)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Database Node - Node Started")

    def __callback_set_settings(self, req):
        response = system_api_client.set_setting(req.name, req.value)
        if not response.success:
            return CommandStatus.DATABASE_DB_ERROR, response.detail

        self.__setting_update_publisher.publish(req.name, req.value, req.type)
        return CommandStatus.SUCCESS, 'Settings successfully set'

    def __callback_get_settings(self, req):
        response = system_api_client.get_setting(req.name)
        if not response.success:
            return CommandStatus.DATABASE_DB_ERROR, response.detail, ''
        value = response.data[req.name]
        value_type = response.data['type']

        return CommandStatus.SUCCESS, value, value_type

    def __callback_add_file_path(self, req):
        try:
            row_id = self.__file_paths.add_file_path(req.type, req.name, req.path)
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
            FilePathMsg(id=x['id'], type=x['type'], name=x['name'], date=x['date'], path=x['path']) for x in filepaths
        ]

        return CommandStatus.SUCCESS, res

    def __callback_rm_file_path(self, req):
        self.__file_paths.rm_file_path(req.id)
        return CommandStatus.SUCCESS, 'Successfully deleted'

    def __sw_callback(self, msg):
        motors_names = ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6', 'end_effector']
        for motor_name, motor_version in zip(motors_names, msg.stepper_firmware_versions):
            self.__version.set(motor_name, motor_version)
        time.sleep(5)


if __name__ == "__main__":
    sentry_init()

    rospy.init_node('niryo_robot_database', anonymous=False, log_level=rospy.INFO)
    try:
        node = DatabaseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
