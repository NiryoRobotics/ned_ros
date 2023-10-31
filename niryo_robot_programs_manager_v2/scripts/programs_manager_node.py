#!/usr/bin/env python
from threading import Thread, Event
from typing import Optional

import actionlib
from actionlib import ServerGoalHandle
import rospy
import logging
import os

from actionlib_msgs.msg import GoalStatus
from niryo_robot_programs_manager_v2.ProgramsManager import ProgramsManager

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from niryo_robot_programs_manager_v2.msg import (ProgramList,
                                                 ExecuteProgramResult,
                                                 Program,
                                                 ExecuteProgramFeedback,
                                                 ExecuteProgramGoal)
from niryo_robot_programs_manager_v2.msg import ExecuteProgramAction

# Services
from niryo_robot_msgs.srv import Trigger, GetString
from niryo_robot_database.srv import GetSettings, SetSettings

from niryo_robot_programs_manager_v2.srv import GetProgram, GetProgramResponse
from niryo_robot_programs_manager_v2.srv import GetProgramAutorunInfos
from niryo_robot_programs_manager_v2.srv import CreateProgram, DeleteProgram
from niryo_robot_programs_manager_v2.srv import SetProgramAutorun, SetProgramAutorunRequest


class ProgramManagerNode:

    def __init__(self):
        rospy.logdebug("Programs Manager - Entering in Init")

        rospy.wait_for_service('/niryo_robot_database/get_db_file_path', 20)
        get_db_path_service = rospy.ServiceProxy('/niryo_robot_database/get_db_file_path', GetString)

        database_path = get_db_path_service().value
        programs_base_dir = os.path.expanduser(rospy.get_param("~programs_dir"))

        self.__programs_manager = ProgramsManager(database_path, programs_base_dir)

        rospy.logdebug("Programs Manager - Managers created !")

        # Autorun
        self.__set_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
        get_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/get', GetSettings)

        get_setting_response = get_setting_service('autorun_id')
        if get_setting_response.status < 0:
            self.__autorun_id = ''
            self.__set_setting_service('autorun_id', self.__autorun_id, 'str')
        else:
            self.__autorun_id = get_setting_response.value

        self.__autorun_mode_to_str = {
            SetProgramAutorunRequest.DISABLE: "DISABLE",
            SetProgramAutorunRequest.ONE_SHOT: "ONE_SHOT",
            SetProgramAutorunRequest.LOOP: "LOOP",
        }
        self.__str_to_autorun_mode = {string: mode for mode, string in self.__autorun_mode_to_str.items()}
        self.__stop_autorun_event = Event()

        get_setting_response = get_setting_service('autorun_mode')
        if get_setting_response.status < 0:
            self.__autorun_mode = SetProgramAutorunRequest.DISABLE
            mode_str = self.__autorun_mode_to_str[self.__autorun_mode]
            self.__set_setting_service('autorun_mode', mode_str, 'str')
        else:
            self.__autorun_mode = self.__str_to_autorun_mode[get_setting_response.value]

        # Action Server
        self.__execute_program_action_server = actionlib.ActionServer('~execute_program',
                                                                      ExecuteProgramAction,
                                                                      goal_cb=self.__callback_execute_program_goal,
                                                                      cancel_cb=self.__execute_program_cancel_cb,
                                                                      auto_start=False)
        self.__execute_program_action_server.start()

        self.__execute_program_action_client = actionlib.ActionClient('~execute_program', ExecuteProgramAction)

        # Services
        rospy.Service('~create_program', CreateProgram, self.__callback_create_program)
        rospy.Service('~delete_program', DeleteProgram, self.__callback_delete_program)
        rospy.Service('~get_program', GetProgram, self.__callback_get_program)
        rospy.Service('~stop_execution', Trigger, self.__callback_stop_execution)

        # Publisher
        self.__program_list_publisher = rospy.Publisher('~program_list', ProgramList, latch=True, queue_size=1)
        self.__publish_program_list()

        # - Autorun
        rospy.Service('~set_program_autorun', SetProgramAutorun, self.__callback_set_program_autorun)
        rospy.Service('~get_program_autorun_infos', GetProgramAutorunInfos, self.__callback_get_program_autorun_infos)
        rospy.Service('~execute_program_autorun', Trigger, self.__callback_execute_program_autorun)

        rospy.on_shutdown(self.stop_program)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Programs Manager - Started")

    # -- ROS CALLBACKS
    # Program
    def __callback_create_program(self, req):
        program_id = self.__programs_manager.create_program(req.name,
                                                            req.description,
                                                            req.python_code,
                                                            req.blockly_code)
        self.__publish_program_list()
        return CommandStatus.SUCCESS, f'Program "{req.name}" successfully created', program_id

    def __callback_delete_program(self, req):
        self.__programs_manager.delete_program(req.program_id)
        self.__publish_program_list()
        return CommandStatus.SUCCESS, 'Program successfully deleted'

    def __callback_get_program(self, req):
        resp = GetProgramResponse()
        resp.status = CommandStatus.SUCCESS
        resp.program = self.program_msg_from_program_manager(self.__programs_manager.get(req.program_id))
        return resp

    def __callback_execute_program_goal(self, goal_handle: ServerGoalHandle):
        if self.__programs_manager.execution_is_running:
            status = CommandStatus.PROGRAMS_MANAGER_PROGRAM_ALREADY_RUNNING
            message = "Program is already running"
            goal_handle.set_rejected(ExecuteProgramResult(status=status, message=message))

        goal = goal_handle.goal.goal

        if not goal.execute_from_string and not self.__programs_manager.exists(goal.program_id):
            status = CommandStatus.PROGRAMS_MANAGER_FILE_DOES_NOT_EXIST
            message = "Program does not exist"
            goal_handle.set_rejected(ExecuteProgramResult(status=status, message=message))

        goal_handle.set_accepted()

        Thread(target=self.__execute_program, args=[goal_handle, goal]).start()

    def __execute_program(self, goal_handle: ServerGoalHandle, goal: ExecuteProgramGoal):
        if goal.execute_from_string:
            self.__programs_manager.execute_from_code(goal.code_string)
        else:
            self.__programs_manager.execute_from_id(goal.program_id)

        feedback = ExecuteProgramFeedback(output=self.__programs_manager.execution_output)
        while self.__programs_manager.execution_is_running:
            if len(self.__programs_manager.execution_output) > len(feedback.output):
                feedback.output = self.__programs_manager.execution_output
                goal_handle.publish_feedback(feedback)

        if goal_handle.get_goal_status().status == GoalStatus.ACTIVE:
            if self.__programs_manager.execution_is_success:
                goal_handle.set_succeeded()
            else:
                goal_handle.set_aborted()

    def __execute_program_cancel_cb(self, goal_handle: ServerGoalHandle):
        self.stop_program()
        goal_handle.set_canceled()

    # Autorun
    def __callback_set_program_autorun(self, req):
        if req.mode:
            self.__autorun_mode = req.mode
            mode_str = self.__autorun_mode_to_str[req.mode]
            self.__set_setting_service('autorun_mode', mode_str, 'str')

        if req.program_id:
            self.__autorun_id = req.program_id
            self.__set_setting_service('autorun_id', req.program_id, 'str')

        return CommandStatus.SUCCESS, 'Successfully set autorun'

    def __callback_get_program_autorun_infos(self, _req):
        return CommandStatus.SUCCESS, 'OK', self.__autorun_id, self.__autorun_mode

    def __callback_execute_program_autorun(self, _req):
        if self.__programs_manager.execution_is_running:
            return CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED, "Program is already running"

        if self.__autorun_mode == SetProgramAutorunRequest.DISABLE:
            rospy.loginfo("Programs Manager - Autorun disable")
            return CommandStatus.PROGRAMS_MANAGER_AUTORUN_FAILURE, "Autorun disable"

        def execute_autorun(loop_mode=False):
            client_goal_handle = self.__execute_program_action_client.send_goal(
                ExecuteProgramGoal(program_id=self.__autorun_id))
            while client_goal_handle.get_comm_state() != actionlib.CommState.DONE:
                rospy.sleep(0.1)

            if not self.__stop_autorun_event.is_set() and loop_mode:
                execute_autorun(loop_mode)
            else:
                self.__stop_autorun_event.clear()

        autorun_thread = Thread(target=execute_autorun,
                                daemon=True,
                                args=[self.__autorun_mode == SetProgramAutorunRequest.LOOP])
        autorun_thread.start()

        message = "Autorun execution started"
        rospy.loginfo("Programs Manager - {}".format(message))
        return CommandStatus.SUCCESS, message

    def __callback_stop_execution(self, _):
        self.stop_program()
        self.__execute_program_action_client.cancel_all_goals()
        return CommandStatus.SUCCESS, 'Execution successfully stopped'

    # - Functions

    @staticmethod
    def program_msg_from_program_manager(program):
        return Program(program_id=program['id'],
                       name=program['name'],
                       description=program['description'],
                       has_blockly=program['has_blockly'],
                       saved_at=program['saved_at'],
                       python_code=program['python_code'],
                       blockly_code=program['blockly_code'])

    def __publish_program_list(self):
        programs_list = self.__programs_manager.get_all()
        ros_programs = [self.program_msg_from_program_manager(program) for program in programs_list]
        self.__program_list_publisher.publish(ros_programs)

    # - Others functions
    def stop_program(self):
        if not self.__programs_manager.execution_is_running:
            return

        self.__stop_autorun_event.set()
        self.__programs_manager.stop_execution()
        self.__stop_robot_action()

    @staticmethod
    def __stop_robot_action():
        # Stop current move command
        try:
            rospy.wait_for_service('/niryo_robot_arm_commander/stop_command', 1)
            stop_cmd = rospy.ServiceProxy('/niryo_robot_arm_commander/stop_command', Trigger)
            stop_cmd()
        except (rospy.ServiceException, rospy.ROSException):
            pass


if __name__ == "__main__":
    rospy.init_node('niryo_programs_manager_v2', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = ProgramManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
