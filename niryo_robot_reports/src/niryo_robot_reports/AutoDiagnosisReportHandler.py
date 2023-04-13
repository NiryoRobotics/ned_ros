import rospy
import json
import rospkg
import os
from datetime import datetime

# msg
from niryo_robot_programs_manager.msg import ProgramLanguage
from niryo_robot_msgs.msg import CommandStatus

# srv
from niryo_robot_programs_manager.srv import ExecuteProgram, ExecuteProgramRequest
from niryo_robot_reports.srv import RunAutoDiagnosis

from niryo_robot_reports.CloudAPI import MicroServiceError

from niryo_robot_metrics.TuptimeWrapper import TuptimeWrapper
from niryo_robot_metrics.PsutilWrapper import PsutilWrapper


class AutoDiagnosisReportHandler:

    def __init__(self, cloud_api):
        self.__cloud_api = cloud_api

        self.__tuptime_wrapper = TuptimeWrapper()
        self.__psutil_wrapper = PsutilWrapper()

        rospy.wait_for_service('/niryo_robot_programs_manager/execute_program', 5)
        self.__execute_program_service = rospy.ServiceProxy('/niryo_robot_programs_manager/execute_program',
                                                            ExecuteProgram)

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

        req = ExecuteProgramRequest(execute_from_string=True, name='', code_string=code_string)
        req.language.used = ProgramLanguage.PYTHON3
        rospy.logdebug('Executing the auto-diagnosis script...')
        res = self.__execute_program_service(req)

        serialized_json = res.output[res.output.index('{')::].rstrip()
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
