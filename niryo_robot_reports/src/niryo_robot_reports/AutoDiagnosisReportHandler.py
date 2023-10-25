import actionlib
import rospy
import json
import rospkg
import os
from datetime import datetime

# msg
from niryo_robot_programs_manager.msg import ExecuteProgramAction, ExecuteProgramGoal
from niryo_robot_msgs.msg import CommandStatus

# srv
from niryo_robot_reports.srv import RunAutoDiagnosis

from niryo_robot_reports.CloudAPI import MicroServiceError

from niryo_robot_metrics.TuptimeWrapper import TuptimeWrapper
from niryo_robot_metrics.PsutilWrapper import PsutilWrapper


class AutoDiagnosisReportHandler:

    def __init__(self, cloud_api):
        self.__cloud_api = cloud_api

        self.__tuptime_wrapper = TuptimeWrapper()
        self.__psutil_wrapper = PsutilWrapper()

        self.__execute_program_action_client = actionlib.SimpleActionClient(
            '/niryo_robot_programs_manager/execute_program', ExecuteProgramAction)

        self.__auto_diagnosis_file = os.path.join(rospkg.RosPack().get_path('niryo_robot_reports'),
                                                  'scripts',
                                                  rospy.get_param('~auto_diagnosis'))

        rospy.Service('~run_auto_diagnosis', RunAutoDiagnosis, self.__run_auto_diagnosis_callback)

    def __run_auto_diagnosis_callback(self, _req):
        if not os.path.isfile(self.__auto_diagnosis_file):
            return CommandStatus.FILE_NOT_FOUND, \
                   'Auto-diagnosis file not found at {}'.format(self.__auto_diagnosis_file)

        with open(self.__auto_diagnosis_file, 'r') as f:
            code_string = f.read()

        rospy.logdebug('Executing the auto-diagnosis script...')
        output = ''

        def feedback_cb(feedback):
            output = feedback.output[-1]

        self.__execute_program_action_client.send_goal(ExecuteProgramGoal(execute_from_string=True,
                                                                          code_string=code_string),
                                                       feedback_cb=feedback_cb)
        self.__execute_program_action_client.wait_for_result()

        serialized_json = output[output.index('{')::].rstrip()
        try:
            report = json.loads(serialized_json)
        except ValueError:
            rospy.logerr('json parsing failed:\n' + serialized_json)
            report = {'details': 'Unable to retrieve the details'}

        rospy.logdebug('Fetching metrics...')
        report['metrics'] = list(self.__tuptime_wrapper.data.values()) + list(self.__psutil_wrapper.data.values())

        report['date'] = datetime.now().isoformat()
        rospy.logdebug(report)
        try:
            self.__cloud_api.auto_diagnosis_reports.send(report)
        except MicroServiceError as microservice_error:
            rospy.logerr(str(microservice_error))
            return CommandStatus.REPORTS_SENDING_FAIL, 'Unable to send the report'

        return CommandStatus.SUCCESS, 'Report sent successfully'
