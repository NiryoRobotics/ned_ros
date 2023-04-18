import json
import rospy
from datetime import datetime
from dateutil import parser

from niryo_robot_reports.TestReport import TestReport
from niryo_robot_reports.CloudAPI import MicroServiceError

# msg
from std_msgs.msg import String
from niryo_robot_msgs.msg import CommandStatus


class TestReportHandler:

    def __init__(self, cloud_api, reports_path, add_report_db, rm_report_db, get_all_files_paths_db):
        self.__cloud_api = cloud_api
        self.__reports_path = reports_path
        self.__add_report_db = add_report_db
        self.__rm_report_db = rm_report_db
        self.__get_all_files_paths_db = get_all_files_paths_db

        self.__send_failed_test_reports()

        rospy.Subscriber('~test_report', String, self.__test_report_callback)

    def __test_report_callback(self, req):
        rospy.logdebug('report received')
        try:
            parsed_json = json.loads(req.data)
        except ValueError as e:
            rospy.logerr('Malformed json: ' + str(e))
            return
        parsed_json['date'] = datetime.now().isoformat()
        try:
            self.__cloud_api.test_reports.send(parsed_json)
        except MicroServiceError as microservice_error:
            rospy.logerr(str(microservice_error))
            report_name = 'test_{}.json'.format(parsed_json['date'])
            report_path = '{}/{}'.format(self.__reports_path, report_name)
            report_handler = TestReport(report_path)
            report_handler.set_content(parsed_json)
            self.__add_report_db('test_report', report_name, report_path)

    def __send_failed_test_reports(self):
        test_reports_response = self.__get_all_files_paths_db('test_report')
        if test_reports_response.status == CommandStatus.SUCCESS:
            for report in test_reports_response.filepaths:
                report_handler = TestReport(report.path)
                rospy.loginfo('Sending the test report of {}'.format(report_handler.content['date']))

                try:
                    self.__cloud_api.test_reports.send(report_handler.content)
                    report_handler.delete()
                    self.__rm_report_db(report.id)
                except MicroServiceError as microservice_error:
                    rospy.logerr(str(microservice_error))
                    if (datetime.now() - parser.parse(report_handler.content['date'])).days > 2:
                        rospy.loginfo('Deleting the outdated test report of {}'.format(report_handler.content['date']))
                        report_handler.delete()
                        self.__rm_report_db(report.id)
