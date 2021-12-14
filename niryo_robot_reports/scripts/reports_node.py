#!/usr/bin/env python

# Libs
import os
import rospy
from distutils.dir_util import mkpath

from niryo_robot_reports.CloudAPI import CloudAPI
from DailyReportHandler import DailyReportHandler
from TestReportHandler import TestReportHandler

# msg
from niryo_robot_msgs.msg import CommandStatus

# srv
from niryo_robot_database.srv import GetSettings, GetAllByType, AddFilePath, RmFilePath


class ReportsNode:
    def __init__(self):
        rospy.logdebug("Reports Node - Entering in Init")

        self.__get_setting = rospy.ServiceProxy(
            '/niryo_robot_database/settings/get', GetSettings
        )

        cloud_domain = rospy.get_param('~cloud_domain')
        get_serial_number_response = self.__get_setting('serial_number')
        get_api_key_response = self.__get_setting('api_key')

        if get_serial_number_response.status != get_api_key_response.status != 200:
            rospy.logerr('Unable to fetch either the serial number or the api key')

        cloud_api = CloudAPI(
            cloud_domain, get_serial_number_response.value,
            get_api_key_response.value
        )

        get_report_path_response = get_setting('reports_path')
        if get_report_path_response.status != CommandStatus.SUCCESS:
            rospy.logerr(
                'Unable to retrieve the reports directory path from the database'
            )
        reports_path = os.path.expanduser(get_report_path_response.value)
        if not os.path.isdir(reports_path):
            mkpath(reports_path)

        get_all_files_paths = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/get_all_by_type', GetAllByType
        )
        add_report_db = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/add', AddFilePath
        )
        rm_report_db = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/rm', RmFilePath
        )

        DailyReportHandler(cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths)
        TestReportHandler(cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Reports Node - Node Started")


if __name__ == "__main__":
    rospy.init_node(
        'niryo_robot_reports', anonymous=False, log_level=rospy.INFO
    )

    rospy.wait_for_service('/niryo_robot_database/settings/get')
    get_setting = rospy.ServiceProxy(
        '/niryo_robot_database/settings/get', GetSettings
    )
    sharing_allowed_response = get_setting('sharing_allowed')
    if sharing_allowed_response.status != CommandStatus.SUCCESS or sharing_allowed_response.value == 'False':
        rospy.logwarn('Sharing not allowed, the niryo_robot_reports won\'t start')
        exit()

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
