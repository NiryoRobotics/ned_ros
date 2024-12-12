#!/usr/bin/env python
from threading import Thread, Event

import actionlib
from actionlib import ServerGoalHandle
import rospy
import logging
import os

from niryo_robot_utils import sentry_init, async_init

from actionlib_msgs.msg import GoalStatus
from niryo_robot_programs_manager_v2.ProgramsManager import ProgramsManager

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from niryo_robot_programs_manager_v2.msg import Program, ProgramList
from niryo_robot_programs_manager_v2.msg import ExecuteProgramResult, ExecuteProgramFeedback, ExecuteProgramGoal
from niryo_robot_programs_manager_v2.msg import ExecuteProgramAction

# Services
from niryo_robot_msgs.srv import Trigger, GetString
from niryo_robot_database.srv import GetSettings, SetSettings

from niryo_robot_programs_manager_v2.srv import GetProgram, GetProgramList
from niryo_robot_programs_manager_v2.srv import GetProgramAutorunInfos
from niryo_robot_programs_manager_v2.srv import CreateProgram, DeleteProgram, UpdateProgram
from niryo_robot_programs_manager_v2.srv import SetProgramAutorun, SetProgramAutorunRequest


class ProgramManagerNode:

    def __init__(self):
        rospy.logdebug("Programs Manager - Entering in Init")

        # Autorun
        self.__get_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/get', GetSettings)
        self.__set_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)

        self.__lazy_loaded_autorun_id = None
        self.__lazy_loaded_autorun_mode = None
        self.__autorun_mode_to_str = {
            SetProgramAutorunRequest.DISABLE: "DISABLE",
            SetProgramAutorunRequest.ONE_SHOT: "ONE_SHOT",
            SetProgramAutorunRequest.LOOP: "LOOP",
        }
        self.__str_to_autorun_mode = {v: k for k, v in self.__autorun_mode_to_str.items()}

        self.__stop_autorun_event = Event()

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
        rospy.Service('~update_program', UpdateProgram, self.__callback_update_program)
        rospy.Service('~get_program', GetProgram, self.__callback_get_program)
        rospy.Service('~stop_execution', Trigger, self.__callback_stop_execution)
        rospy.Service('~get_program_list', GetProgramList, self.__callback_get_program_list)

        # Publisher
        self.__program_list_publisher = rospy.Publisher('~program_list', ProgramList, latch=True, queue_size=1)

        # - Autorun
        rospy.Service('~set_program_autorun', SetProgramAutorun, self.__callback_set_program_autorun)
        rospy.Service('~get_program_autorun_infos', GetProgramAutorunInfos, self.__callback_get_program_autorun_infos)
        rospy.Service('~execute_program_autorun', Trigger, self.__callback_execute_program_autorun)

        self.__lazy_loaded_programs_manager = None
        async_init.PromiseServiceProxy('/niryo_robot_database/get_db_file_path',
                                       GetString,
                                       self.__on_get_db_file_path_available)

        rospy.on_shutdown(self.stop_program)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Programs Manager - Started")

    def __on_get_db_file_path_available(self, get_db_file_path_proxy):
        database_path = get_db_file_path_proxy().value
        programs_base_dir = os.path.expanduser(rospy.get_param("~programs_dir"))

        self.__lazy_loaded_programs_manager = ProgramsManager(database_path, programs_base_dir)
        self.__publish_program_list()

    @property
    def __programs_manager(self):
        if self.__lazy_loaded_programs_manager is None:
            raise RuntimeError("Programs Manager is not initialized yet")
        return self.__lazy_loaded_programs_manager

    def __get_setting_from_db(self, setting_name: str, default_value: str) -> str:
        try:
            self.__get_setting_service.wait_for_service(2)
        except rospy.ROSException:
            rospy.logerr("Programs Manager - Impossible to connect to the database get setting service")
            raise

        response = self.__get_setting_service(setting_name)
        if response.status < 0:
            rospy.logwarn(
                f'The setting "{setting_name}" has not been found in database. Setting it to "{default_value}"')
            self.__set_setting_service(setting_name, default_value, 'str')
            return default_value
        return response.value

    @property
    def __autorun_id(self) -> str:
        if self.__lazy_loaded_autorun_id is None:
            self.__lazy_loaded_autorun_id = self.__get_setting_from_db('autorun_id', '')
        return self.__lazy_loaded_autorun_id

    @property
    def __autorun_mode(self) -> str:
        if self.__lazy_loaded_autorun_mode is None:
            autorun_mode_str = self.__get_setting_from_db('autorun_mode', 'DISABLE')
            self.__lazy_loaded_autorun_mode = self.__str_to_autorun_mode[autorun_mode_str]
        return self.__lazy_loaded_autorun_mode

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

    def __callback_update_program(self, req):
        if not self.__programs_manager.exists(req.program_id):
            return CommandStatus.PROGRAMS_MANAGER_FILE_DOES_NOT_EXIST, f'No program with id {req.program_id}'
        self.__programs_manager.update_program(req.program_id,
                                               req.name,
                                               req.description,
                                               req.python_code,
                                               req.blockly_code)
        return CommandStatus.SUCCESS, 'Program successfully updated'

    def __callback_get_program(self, req):
        program = self.program_msg_from_program_manager(self.__programs_manager.get(req.program_id))
        return CommandStatus.SUCCESS, f'Successfully retrieved program "{program.program_id}"', program

    def __callback_get_program_list(self, _):
        ros_programs = [self.program_msg_from_program_manager(program) for program in self.__programs_manager.programs]
        return CommandStatus.SUCCESS, 'Successfully retrieved programs', ros_programs

    def __callback_execute_program_goal(self, goal_handle: ServerGoalHandle):
        rospy.logdebug(f'Received goal "{goal_handle.get_goal_id()}"')
        if self.__programs_manager.execution_is_running:
            status = CommandStatus.PROGRAMS_MANAGER_PROGRAM_ALREADY_RUNNING
            message = "Program is already running"
            goal_handle.set_rejected(ExecuteProgramResult(status=status, message=message), message)
            return

        goal = goal_handle.goal.goal

        if goal.code_string == '' and not self.__programs_manager.exists(goal.program_id):
            status = CommandStatus.PROGRAMS_MANAGER_FILE_DOES_NOT_EXIST
            message = "Program does not exist"
            goal_handle.set_rejected(ExecuteProgramResult(status=status, message=message), message)
            return

        rospy.logdebug('Goal accepted')
        goal_handle.set_accepted()
        Thread(target=self.__execute_program, args=[goal_handle], daemon=True).start()

    def __execute_program(self, goal_handle: ServerGoalHandle):
        goal = goal_handle.goal.goal
        try:
            # execute the program
            if goal.code_string != '':
                self.__programs_manager.execute_from_code(goal.code_string)
            else:
                self.__programs_manager.execute_from_id(goal.program_id)

            # handle the feedback
            feedback = ExecuteProgramFeedback(output=self.__programs_manager.execution_output)
            while self.__programs_manager.execution_is_running:
                if len(self.__programs_manager.execution_output) > len(feedback.output):
                    feedback.output = self.__programs_manager.execution_output
                    goal_handle.publish_feedback(feedback)
                rospy.sleep(0.1)

            # set the final goal status
            if goal_handle.get_goal_status().status == GoalStatus.ACTIVE:
                if self.__programs_manager.execution_is_success:
                    message = 'Execution success'
                    goal_handle.set_succeeded(ExecuteProgramResult(status=CommandStatus.SUCCESS, message=message),
                                              message)
                else:
                    message = 'Execution failed'
                    goal_handle.set_aborted(
                        ExecuteProgramResult(
                            status=CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED,
                            message=message,
                        ),
                        message)
            else:
                rospy.logwarn('The goal status is not "active" anymore. It\'s most likely it has been canceled.')
        except Exception as e:
            message = f'An unknown error happened during program execution: {e}'
            rospy.logerr(message)
            goal_handle.set_aborted(
                ExecuteProgramResult(
                    status=CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED,
                    message=message,
                ),
                message)

    def __execute_program_cancel_cb(self, goal_handle: ServerGoalHandle):
        self.stop_program()
        goal_handle.set_canceled(
            ExecuteProgramResult(status=CommandStatus.SUCCESS, message='The execution has been stopped'))

    # Autorun
    def __callback_set_program_autorun(self, req):
        self.__lazy_loaded_autorun_mode = req.mode
        mode_str = self.__autorun_mode_to_str[req.mode]
        self.__set_setting_service('autorun_mode', mode_str, 'str')

        self.__lazy_loaded_autorun_id = req.program_id
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
        ros_programs = [self.program_msg_from_program_manager(program) for program in self.__programs_manager.programs]
        self.__program_list_publisher.publish(ros_programs)

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
    sentry_init()

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
