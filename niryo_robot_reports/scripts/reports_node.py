#!/usr/bin/env python

# Libs
import os
import rospy
from distutils.dir_util import mkpath

from niryo_robot_reports.CloudAPI import CloudAPI
from niryo_robot_reports.DailyReportHandler import DailyReportHandler
from niryo_robot_reports.TestReportHandler import TestReportHandler
from niryo_robot_reports.AlertReportHandler import AlertReportHandler
from niryo_robot_reports.AutoDiagnosisReportHandler import AutoDiagnosisReportHandler

# msg
from niryo_robot_database.msg import Setting
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_reports.msg import Service

# srv
from niryo_robot_database.srv import GetSettings, GetAllByType, AddFilePath, RmFilePath
from niryo_robot_reports.srv import CheckConnection


class ReportsNode:
    def __init__(self):
        self.__wait_booting()
        rospy.logdebug("Reports Node - Entering in Init")

        rospy.wait_for_service('/niryo_robot_database/settings/get', 20)
        self.__get_setting = rospy.ServiceProxy('/niryo_robot_database/settings/get', GetSettings)

        get_cloud_domain_response = self.__get_setting('cloud_domain')
        get_serial_number_response = self.__get_setting('serial_number')
        get_api_key_response = self.__get_setting('api_key')
        get_sharing_allowed_response = self.__get_setting('sharing_allowed')

        if get_serial_number_response.status != get_api_key_response.status != CommandStatus.SUCCESS:
            rospy.logerr('Unable to fetch either the serial number or the api key')

        if get_sharing_allowed_response.status != CommandStatus.SUCCESS:
            rospy.logwarn('Unable to fetch sharing allowed')
            get_sharing_allowed_response.value = False

        self.__cloud_api = CloudAPI(
            get_cloud_domain_response.value,
            get_serial_number_response.value,
            get_api_key_response.value,
            get_sharing_allowed_response.value,
        )

        get_report_path_response = self.__get_setting('reports_path')
        if get_report_path_response.status != CommandStatus.SUCCESS:
            rospy.logerr(
                'Unable to retrieve the reports directory path from the database'
            )
        reports_path = os.path.expanduser(get_report_path_response.value)
        if not os.path.isdir(reports_path):
            mkpath(reports_path)

        get_all_files_paths = rospy.ServiceProxy('/niryo_robot_database/file_paths/get_all_by_type', GetAllByType)
        add_report_db = rospy.ServiceProxy('/niryo_robot_database/file_paths/add', AddFilePath)
        rm_report_db = rospy.ServiceProxy('/niryo_robot_database/file_paths/rm', RmFilePath)

        DailyReportHandler(self.__cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths)
        TestReportHandler(self.__cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths)
        # TODO to be reactivated if needed
        # AlertReportHandler(self.__cloud_api)
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
            self.__cloud_api.set_serial_number(req.value)
        elif req.name == 'api_key':
            self.__cloud_api.set_api_key(req.value)
        elif req.name == 'sharing_allowed':
            self.__cloud_api.set_sharing_allowed(req.value == 'True')


if __name__ == "__main__":
    rospy.init_node(
        'niryo_robot_reports', anonymous=False, log_level=rospy.INFO
    )

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
