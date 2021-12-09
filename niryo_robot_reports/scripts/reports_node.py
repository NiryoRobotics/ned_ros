#!/usr/bin/env python

# Libs
import json
import os
import rospy
from datetime import date, datetime
from StringIO import StringIO
from distutils.dir_util import mkpath

from niryo_robot_reports.CloudAPI import CloudAPI
from niryo_robot_reports.TestReport import TestReport
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

        rospy.wait_for_service('/niryo_robot_database/settings/get')
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
        self.__get_all_files_paths = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/get_all_by_type', GetAllByType
        )
        self.__add_report_db = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/add', AddFilePath
        )
        self.__rm_report_db = rospy.ServiceProxy(
            '/niryo_robot_database/file_paths/rm', RmFilePath
        )

        self.__send_failed_daily_reports()
        self.__send_failed_test_reports()

        report_name = '{}.json'.format(self.__current_date)
        report_path = '{}/{}'.format(self.__reports_path, report_name)
        self.__daily_report = DailyReport(report_path)
        self.__daily_report.set_date(self.__current_date)
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
            self.__daily_report.content
        })
        if success:
            self.__daily_report.delete()
            self.__rm_report_db(self.__current_id)
        self.__current_date = current_day
        new_path = '{}/{}.json'.format(self.__reports_path, self.__current_date)
        self.__daily_report.set_path(new_path)
        self.__daily_report.set_date(self.__current_date)

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
        self.__daily_report.add_log(formatted_log, 'ROS', str(datetime.now()))

    def __test_report_callback(self, req):
        try:
            parsed_json = json.loads(req.data)
        except ValueError as e:
            rospy.logerr('Malformed json: ' + str(e))
            return
        parsed_json['date'] = datetime.now().isoformat()
        success = self.__cloud_api.test_report.send(parsed_json)
        if not success:
            report_name = 'test_{}.json'.format(parsed_json['date'])
            report_path = '{}/{}'.format(self.__reports_path, report_name)
            report_handler = TestReport(report_path)
            report_handler.set_content(parsed_json)
            self.__add_report_db(
                'test_report', report_name, report_path
            )

    def __send_failed_daily_reports(self):
        daily_reports_response = self.__get_all_files_paths('daily_report')
        if daily_reports_response.status == CommandStatus.SUCCESS:
            for report in daily_reports_response.filepaths:
                if report.date == self.__current_date:
                    continue
                report_handler = DailyReport(report.path)
                rospy.loginfo('Sending the report of {}'.format(report.date))
                success = self.__cloud_api.daily_reports.send(report_handler.content)
                if success:
                    report_handler.delete()
                    self.__rm_report_db(report.id)
                else:
                    rospy.logerr('Unable to send the report')

    def __send_failed_test_reports(self):
        test_reports_response = self.__get_all_files_paths('test_report')
        if test_reports_response.status == CommandStatus.SUCCESS:
            for report in test_reports_response.filepaths:
                report_handler = TestReport(report.path)
                rospy.loginfo('Sending the test report of {}'.format(report_handler.content['date']))
                success = self.__cloud_api.test_report.send(report_handler.content)
                if success:
                    report_handler.delete()
                    self.__rm_report_db(report.id)
                else:
                    rospy.logerr('Unable to send the report')


if __name__ == "__main__":
    rospy.init_node(
        'niryo_robot_reports', anonymous=False, log_level=rospy.INFO
    )

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
