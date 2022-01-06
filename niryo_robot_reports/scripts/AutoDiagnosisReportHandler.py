import rospy
import json
from datetime import datetime

# msg
from niryo_robot_programs_manager.msg import ProgramLanguage
from niryo_robot_msgs.msg import CommandStatus

# srv
from niryo_robot_programs_manager.srv import ExecuteProgram, ExecuteProgramRequest
from niryo_robot_reports.srv import RunAutoDiagnosis

from niryo_robot_reports.metrics.TuptimeWrapper import TuptimeWrapper
from niryo_robot_reports.metrics.PsutilWrapper import PsutilWrapper


class AutoDiagnosisReportHandler:
    def __init__(self, cloud_api):
        self.__cloud_api = cloud_api

        rospy.wait_for_service('/niryo_robot_programs_manager/execute_program', 5)
        self.__execute_program_service = rospy.ServiceProxy('/niryo_robot_programs_manager/execute_program',
                                                            ExecuteProgram)

        rospy.Service('~run_auto_diagnosis', RunAutoDiagnosis, self.__run_auto_diagnosis_callback)

    def __run_auto_diagnosis_callback(self, _req):
        req = ExecuteProgramRequest()
        req.execute_from_string = False
        req.name = 'auto_diagnosis'
        req.code_string = ''
        req.language.used = ProgramLanguage.PYTHON2
        rospy.logdebug('Executing the auto-diagnosis script...')
        res = self.__execute_program_service(req)
        serialized_json = res.output[res.output.index('{')].rstrip()
        try:
            report = json.loads(serialized_json)
        except ValueError:
            rospy.logerr('json parsing failed:\n' + serialized_json)
            report = {'details': 'Unable to retrieve the details'}

        rospy.logdebug('Fetching metrics...')
        report['metrics'] = []
        report['metrics'] += [{'name': x, 'value': y} for x, y in PsutilWrapper.get_data().items()]
        report['metrics'] += [{'name': x, 'value': y} for x, y in TuptimeWrapper.get_data().items()]

        report['date'] = datetime.now().isoformat()
        success = self.__cloud_api.auto_diagnosis_reports.send(report)

        rospy.logdebug(report)

        if not success:
            return CommandStatus.REPORTS_SENDING_FAIL, 'Unable to send the report'

        return CommandStatus.SUCCESS, 'Report sent successfully'
