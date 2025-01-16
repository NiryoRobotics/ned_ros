#!/usr/bin/env python3

# Libs
import os
import rospy
from distutils.dir_util import mkpath

from niryo_robot_utils import sentry_init, async_init

from niryo_robot_reports.CloudAPI import CloudAPI, MicroServiceError
from niryo_robot_reports.AlertReportHandler import AlertReportHandler
from niryo_robot_reports.DailyReportHandler import DailyReportHandler
from niryo_robot_reports.TestReportHandler import TestReportHandler
from niryo_robot_reports.AutoDiagnosisReportHandler import AutoDiagnosisReportHandler

# msg
from niryo_robot_database.msg import Setting
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_reports.msg import Service

# srv
from niryo_robot_database.srv import GetSettings, GetAllByType, AddFilePath, RmFilePath, SetSettings
from niryo_robot_reports.srv import CheckConnection


class ReportsNode:

    def __init__(self):
        rospy.logdebug("Reports Node - Entering in Init")

        async_init.PromiseServiceProxy('/niryo_robot_database/settings/get',
                                       GetSettings,
                                       self.__on_get_settings_available)

        rospy.Service('~check_connection', CheckConnection, self.__check_connection_callback)

        # Set a bool to mention that this node is initialized
        rospy.set_param('~initialized', True)

        rospy.Subscriber('/niryo_robot_reports/setting_update', Setting, self.__setting_update_callback)

        rospy.logdebug("Reports Node - Node Started")

    @property
    def __cloud_api(self):
        if self.__lazy_loaded_cloud_api is None:
            raise RuntimeError("Cloud API is not initialized yet")
        return self.__lazy_loaded_cloud_api

    def __on_get_settings_available(self, get_settings_proxy):
        settings = {}
        for setting in ['cloud_domain', 'serial_number', 'rasp_id', 'api_key', 'sharing_allowed']:
            response = get_settings_proxy(setting)
            setting_value = response.value
            if response.status != CommandStatus.SUCCESS:
                rospy.logerr(f'Unable to get setting "{setting}"')
                setting_value = None
            settings[setting] = setting_value

        self.__lazy_loaded_cloud_api = CloudAPI(**settings, https=True)
        try:
            self.__cloud_api.authentification.ping()
        except MicroServiceError:
            try:
                api_key = self.__cloud_api.authentification.authenticate()
                self.__cloud_api.set_api_key(api_key)
                async_init.PromiseServiceProxy('/niryo_robot_database/settings/set',
                                               SetSettings,
                                               lambda set_settings_proxy: set_settings_proxy('api_key', api_key, 'str'))
            except MicroServiceError as microservice_error:
                rospy.logerr(str(microservice_error))

        get_report_path_response = get_settings_proxy('reports_path')
        if get_report_path_response.status != CommandStatus.SUCCESS:
            rospy.logerr('Unable to retrieve the reports directory path from the database')
        reports_path = os.path.expanduser(get_report_path_response.value)
        if not os.path.isdir(reports_path):
            mkpath(reports_path)

        get_all_files_paths = rospy.ServiceProxy('/niryo_robot_database/file_paths/get_all_by_type', GetAllByType)
        add_report_db = rospy.ServiceProxy('/niryo_robot_database/file_paths/add', AddFilePath)
        rm_report_db = rospy.ServiceProxy('/niryo_robot_database/file_paths/rm', RmFilePath)

        DailyReportHandler(self.__cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths)
        TestReportHandler(self.__cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths)
        AlertReportHandler(self.__cloud_api)
        AutoDiagnosisReportHandler(self.__cloud_api)

    def __check_connection_callback(self, req):
        rospy.logdebug('service called: ' + str(req.service.to_test))
        try:
            {
                Service.TEST_REPORTS: self.__cloud_api.test_reports.ping,
                Service.DAILY_REPORTS: self.__cloud_api.daily_reports.ping,
                Service.ALERT_REPORTS: self.__cloud_api.alert_reports.ping,
                Service.AUTO_DIAGNOSIS_REPORTS: self.__cloud_api.auto_diagnosis_reports.ping
            }[req.service.to_test]()
        except KeyError:
            return CommandStatus.REPORTS_SERVICE_UNREACHABLE, False

        return CommandStatus.SUCCESS, True

    def __setting_update_callback(self, req):
        try:
            {
                'serial_number': self.__cloud_api.set_serial_number,
                'api_key': self.__cloud_api.set_api_key,
                'sharing_allowed': lambda v: self.__cloud_api.set_sharing_allowed(v == 'True'),
                'rasp_id': self.__cloud_api.set_rasp_id
            }[req.name](req.value)
        except KeyError:
            pass


if __name__ == "__main__":
    sentry_init()

    rospy.init_node('niryo_robot_reports', anonymous=False, log_level=rospy.INFO)

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
