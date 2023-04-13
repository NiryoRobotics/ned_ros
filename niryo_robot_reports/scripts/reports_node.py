#!/usr/bin/env python3

# Libs
import os
import rospy
from distutils.dir_util import mkpath

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
        self.__wait_booting()
        rospy.logdebug("Reports Node - Entering in Init")

        rospy.wait_for_service('/niryo_robot_database/settings/get', 20)
        self.__get_setting = rospy.ServiceProxy('/niryo_robot_database/settings/get', GetSettings)

        settings = {}
        for setting in ['cloud_domain', 'serial_number', 'rasp_id', 'api_key', 'sharing_allowed']:
            response = self.__get_setting(setting)
            setting_value = response.value
            if response.status != CommandStatus.SUCCESS:
                rospy.logerr(f'Unable to get setting "{setting}"')
                setting_value = None
            settings[setting] = setting_value

        self.__cloud_api = CloudAPI(**settings, https=True)

        try:
            self.__cloud_api.authentification.ping()
        except MicroServiceError:
            try:
                api_key = self.__cloud_api.authentification.authenticate()
                self.__cloud_api.set_api_key(api_key)

                rospy.wait_for_service('/niryo_robot_database/settings/set', 20)
                set_setting = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
                set_setting('api_key', api_key, 'str')
            except MicroServiceError as microservice_error:
                rospy.logerr(str(microservice_error))

        get_report_path_response = self.__get_setting('reports_path')
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

        rospy.Service('~check_connection', CheckConnection, self.__check_connection_callback)

        # Set a bool to mention that this node is initialized
        rospy.set_param('~initialized', True)

        rospy.Subscriber('/niryo_robot_reports/setting_update', Setting, self.__setting_update_callback)

        rospy.logdebug("Reports Node - Node Started")

    @staticmethod
    def __wait_booting():
        from niryo_robot_status.msg import RobotStatus

        for i in range(1, 20):
            try:
                msg = rospy.wait_for_message('/niryo_robot_status/robot_status', RobotStatus, 20 / i)
            except rospy.exceptions.ROSInterruptException:
                return
            except rospy.exceptions.ROSException:
                pass
            else:
                if msg.robot_status not in [RobotStatus.BOOTING, RobotStatus.UNKNOWN]:
                    return

            rospy.sleep(1)

    def __check_connection_callback(self, req):
        rospy.logdebug('service called: ' + str(req.service.to_test))
        if req.service.to_test == Service.TEST_REPORTS:
            success = self.__cloud_api.test_reports.ping()
        elif req.service.to_test == Service.DAILY_REPORTS:
            success = self.__cloud_api.daily_reports.ping()
        elif req.service.to_test == Service.ALERT_REPORTS:
            success = self.__cloud_api.alert_reports.ping()
        elif req.service.to_test == Service.AUTO_DIAGNOSIS_REPORTS:
            success = self.__cloud_api.auto_diagnosis_reports.ping()
        else:
            return CommandStatus.REPORTS_SERVICE_UNREACHABLE, False

        return CommandStatus.SUCCESS, success

    def __setting_update_callback(self, req):
        if req.name == 'serial_number':
            self.__cloud_api.set_identifier(req.value)
        elif req.name == 'api_key':
            self.__cloud_api.set_api_key(req.value)
        elif req.name == 'sharing_allowed':
            self.__cloud_api.set_sharing_allowed(req.value == 'True')
        elif req.name == 'rasp_id':
            self.__cloud_api.set_identifier(req.value)


if __name__ == "__main__":
    rospy.init_node('niryo_robot_reports', anonymous=False, log_level=rospy.INFO)

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
