import json
import rospy
from datetime import datetime, date

from niryo_robot_reports.TestReport import TestReport
from niryo_robot_reports.CloudAPI import MicroServiceError
from niryo_robot_system_api_client import system_api_client

# msg
from std_msgs.msg import String


class TestReportHandler:

    def __init__(self, cloud_api, reports_path):
        self.__cloud_api = cloud_api
        self.__reports_path = reports_path

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
            system_api_client.add_file_path('test_report', report_name, report_path)

    def __send_failed_test_reports(self):
        test_reports_response = system_api_client.get_file_paths('test_report')
        if test_reports_response.success:
            for report in test_reports_response.data:
                report_handler = TestReport(report['path'])
                rospy.loginfo('Sending the test report of {}'.format(report_handler.content['date']))

                try:
                    self.__cloud_api.test_reports.send(report_handler.content)
                    report_handler.delete()
                    system_api_client.rm_file_path(report['id'])
                except MicroServiceError as microservice_error:
                    rospy.logerr(str(microservice_error))
                    if (date.today() - date.fromisoformat(report['date'])).days > 2:
                        rospy.loginfo('Deleting the outdated test report of {}'.format(report_handler.content['date']))
                        report_handler.delete()
                        system_api_client.rm_file_path(report['id'])
