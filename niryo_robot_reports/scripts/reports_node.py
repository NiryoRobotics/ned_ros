#!/usr/bin/env python

# Libs
import json
import os
import rospy
from datetime import date, datetime
from StringIO import StringIO
from distutils.dir_util import mkpath

from niryo_robot_reports.CloudAPI import CloudAPI
from niryo_robot_reports.DailyReport import DailyReport

# msg
from std_msgs.msg import Empty, String
from niryo_robot_status.msg import RobotStatus
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

        self.__able_to_send = get_serial_number_response.status == get_api_key_response.status == CommandStatus.SUCCESS
        if not self.__able_to_send:
            rospy.logwarn('Reports Node - Unable to retrieve the serial number')

        self.__cloud_api = CloudAPI(
            cloud_domain, get_serial_number_response.value,
            get_api_key_response.value
        )

        get_report_path_response = self.__get_setting('reports_path')
        if get_report_path_response.status != CommandStatus.SUCCESS:
            rospy.logerr(
                'Unable to retrieve the reports directory path from the database'
            )
        self.__reports_path = os.path.expanduser(get_report_path_response.value)
        if not os.path.isdir(self.__reports_path):
            mkpath(self.__reports_path)

        self.__current_date = str(date.today())

        get_all_files_paths = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/get_all_by_type', GetAllByType
        )
        self.__add_report_db = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/add', AddFilePath
        )
        self.__rm_report_db = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/rm', RmFilePath
        )
        daily_reports_response = get_all_files_paths('daily_report')
        if daily_reports_response.status == CommandStatus.SUCCESS:
            for report in daily_reports_response.filepaths:
                if report.date == self.__current_date:
                    continue
                report_handler = DailyReport(report.path)
                rospy.loginfo('Sending the report of {}'.format(report.date))
                success = self.__cloud_api.daily_reports.send({
                    'date':
                    report.date,
                    'report':
                    report_handler.content
                })
                if success:
                    report_handler.delete()
                    self.__rm_report_db(report.id)
                else:
                    rospy.logerr('Unable to send the report')

        report_name = '{}.json'.format(self.__current_date)
        report_path = '{}/{}'.format(self.__reports_path, report_name)
        self.__report_handler = DailyReport(report_path)
        add_report_response = self.__add_report_db(
            'daily_report', report_name, report_path
        )
        self.__current_id = add_report_response.message

        rospy.Subscriber('~new_day', Empty, self.__new_day_callback)
        rospy.Subscriber(
            '/niryo_robot_status/robot_status', RobotStatus,
            self.__robot_status_callback
        )

        rospy.Subscriber('~test_report', String, self.__test_report_callback)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Reports Node - Node Started")

    # - callbacks

    def __new_day_callback(self, _req):
        current_day = str(date.today())
        if current_day == self.__current_date:
            return
        success = self.__cloud_api.daily_reports.send({
            'date':
            self.__current_date,
            'report':
            self.__report_handler.content
        })
        if success:
            self.__report_handler.delete()
            self.__rm_report_db(self.__current_id)
        self.__current_date = current_day
        new_path = '{}/{}.json'.format(self.__reports_path, self.__current_date)
        self.__report_handler.set_path(new_path)

    def __robot_status_callback(self, req):
        if req.logs_status_str.lower() not in ['error', 'critical']:
            return
        log_io = StringIO(req.logs_message)
        level, from_node, msg, from_file, function, line = map(
            lambda x: x[x.index(':') + 2:], log_io.readlines()
        )
        formatted_log = '{} - {}: {} in {}.{}:{}'.format(
            level, from_node, msg, from_file, function, line
        )
        self.__report_handler.add_log(formatted_log, 'ROS', str(datetime.now()))

    def __test_report_callback(self, req):
        try:
            parsed_json = json.loads(req.date)
        except ValueError as e:
            rospy.logerr('Malformed json: ' + str(e))
            return
        self.__cloud_api.test_report.send(parsed_json)


if __name__ == "__main__":
    rospy.init_node(
        'niryo_robot_reports', anonymous=False, log_level=rospy.INFO
    )

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
