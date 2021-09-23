#!/usr/bin/env python

import rospy
import os
import yaml

from threading import Thread

from niryo_robot_programs_manager.ProgramsFileManager import FileAlreadyExistException, FileDoesNotExistException
from niryo_robot_programs_manager.Python2Manager import Python2FileManager
from niryo_robot_programs_manager.BlocklyManager import BlocklyManager

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from niryo_robot_programs_manager.msg import ProgramList
from niryo_robot_programs_manager.msg import ProgramLanguage, ProgramLanguageList
from std_msgs.msg import Bool

# Services
from niryo_robot_msgs.srv import Trigger

from niryo_robot_programs_manager.srv import ExecuteProgram
from niryo_robot_programs_manager.srv import GetProgram, GetProgramResponse
from niryo_robot_programs_manager.srv import GetProgramAutorunInfos, GetProgramAutorunInfosResponse
from niryo_robot_programs_manager.srv import GetProgramList
from niryo_robot_programs_manager.srv import ManageProgram
from niryo_robot_programs_manager.srv import SetProgramAutorun, SetProgramAutorunRequest


class ProgramManagerNode:

    def __init__(self):
        rospy.logdebug("Programs Manager - Entering in Init")

        self.__programs_base_dir = os.path.expanduser(rospy.get_param("~programs_dir"))
        if not os.path.isdir(self.__programs_base_dir):
            os.makedirs(self.__programs_base_dir)

        self.__python2_manager = Python2FileManager(self.__programs_base_dir)
        self.__blockly_manager = BlocklyManager(self.__programs_base_dir)

        rospy.logdebug("Programs Manager - Managers created !")

        self.__standalone = rospy.get_param("~standalone")

        self.__autorun_file_path = os.path.join(self.__programs_base_dir, rospy.get_param("~autorun_file_name"))

        self._manager_map = {
            ProgramLanguage.PYTHON2: self.__python2_manager,
            ProgramLanguage.BLOCKLY: self.__blockly_manager,
        }

        self.__str_to_program_language_map = {
            "NONE": ProgramLanguage.NONE,
            "PYTHON2": ProgramLanguage.PYTHON2,
            "BLOCKLY": ProgramLanguage.BLOCKLY,
        }
        self.__program_language_to_str_map = {index: string for string, index
                                              in self.__str_to_program_language_map.iteritems()}
        self.__autorun_mode_to_str = {
            SetProgramAutorunRequest.DISABLE: "DISABLE",
            SetProgramAutorunRequest.ONE_SHOT: "ONE_SHOT",
            SetProgramAutorunRequest.LOOP: "LOOP",
        }
        self._str_to_autorun_mode = {index: string for string, index
                                     in self.__autorun_mode_to_str.iteritems()}

        # Services
        rospy.Service('~manage_program', ManageProgram,
                      self.__callback_manage_program)
        rospy.Service('~get_program_list', GetProgramList,
                      self.__callback_get_program_list)
        rospy.Service('~get_program', GetProgram,
                      self.__callback_get_program)

        rospy.Service('~execute_program', ExecuteProgram,
                      self.__callback_execute_program)
        self.__execute_from_string_program_name = rospy.get_param("~execute_from_string_program_name")

        rospy.Service('~stop_program', Trigger,
                      self.__callback_stop_program)

        self.__program_is_running = False

        # Publisher
        self.__program_list_publisher = rospy.Publisher(
            '~program_list', ProgramList, latch=True, queue_size=1)
        self.__publish_program_list()

        self.__program_is_running_publisher = rospy.Publisher(
            '~program_is_running', Bool, latch=True, queue_size=1)
        self.__program_is_running_publisher.publish(self.__program_is_running)

        # - Autorun
        self.__set_program_autorun_service = rospy.Service(
            '~set_program_autorun', SetProgramAutorun,
            self.__callback_set_program_autorun)

        self.__get_program_autorun_infos_service = rospy.Service(
            '~get_program_autorun_infos', GetProgramAutorunInfos,
            self.__callback_get_program_autorun_infos)

        self.__execute_program_autorun_service = rospy.Service(
            '~execute_program_autorun', Trigger,
            self.__callback_execute_program_autorun)

        self.__loop_autorun = False

        self.__execution_thread = None
        self.__execution_stopped = False

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Programs Manager - Started")

    # -- ROS CALLBACKS
    # Program
    def __callback_manage_program(self, req):
        language_used = req.language.used
        if language_used not in self._manager_map:
            return CommandStatus.PROGRAMS_MANAGER_UNKNOWN_LANGUAGE, "Unknown Language"
        manager = self._manager_map[language_used]
        try:
            if req.cmd == req.SAVE:
                manager.create(req.name, req.code, req.description, req.allow_overwrite)
                self.__publish_program_list()

                message = "Created program '{}'".format(req.name)
                rospy.loginfo("Programs Manager - {}".format(message))

                return CommandStatus.SUCCESS, message
            elif req.cmd == req.DELETE:
                manager.remove(req.name)
                self.__publish_program_list()

                message = "Removed program '{}'".format(req.name)
                rospy.loginfo("Programs Manager - {}".format(message))

                return CommandStatus.SUCCESS, message
            else:
                return CommandStatus.UNKNOWN_COMMAND, "Programs Manager - Command '{}' not found.".format(req.cmd)
        except FileAlreadyExistException as e:
            rospy.logerr("Programs Manager - File already exists !")
            return CommandStatus.PROGRAMS_MANAGER_FILE_ALREADY_EXISTS, str(e)
        except Exception as e:
            return CommandStatus.PROGRAMS_MANAGER_WRITING_FAILURE, str(e)

    def __callback_get_program_list(self, req):
        language_msg = req.language
        return self.___get_program_list(language_msg)

    def __callback_get_program(self, req):
        resp = GetProgramResponse()
        language_used = req.language.used
        if language_used not in self._manager_map:
            resp.status = CommandStatus.PROGRAMS_MANAGER_UNKNOWN_LANGUAGE
            resp.message = "Unknown Language"
            return resp
        manager = self._manager_map[language_used]
        try:
            code, description = manager.read(req.name)
            return CommandStatus.SUCCESS, "Success", code, description
        except Exception as e:
            resp.status = CommandStatus.PROGRAMS_MANAGER_READ_FAILURE
            resp.message = str(e)
            return resp

    def __callback_execute_program(self, req):
        if self.__program_is_running:
            return CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED, "Program is already running"
        language = req.language.used
        if req.execute_from_string:
            name = None
            code_string = req.code_string
        else:
            name = req.name
            code_string = None
        return self.__execute_program(language, name=name, code_string=code_string)

    def __callback_stop_program(self, _req):
        if not self.__program_is_running:
            return CommandStatus.SUCCESS, "No program is running"

        try:
            self.__loop_autorun = False
            ret, message = self.__python2_manager.stop_execution()
            if not self.__standalone:
                self.__stop_robot_action()

            if self.__program_is_running:
                self.__program_is_running = False
                self.__program_is_running_publisher.publish(self.__program_is_running)

        except Exception as e:
            return CommandStatus.PROGRAMS_MANAGER_FAILURE, str(e)

        if ret:
            self.__execution_stopped = True
            return CommandStatus.SUCCESS, "Stop program success"
        else:
            return CommandStatus.PROGRAMS_MANAGER_STOPPING_FAILED, message

    # Autorun
    def __callback_set_program_autorun(self, req):
        mode = req.mode

        if mode == SetProgramAutorunRequest.DISABLE:
            return self.__disable_autorun()

        language_used = req.language.used
        if language_used not in self._manager_map:
            return CommandStatus.PROGRAMS_MANAGER_UNKNOWN_LANGUAGE, "Unknown Language"

        manager = self._manager_map[language_used]
        if not manager.runnable:
            return CommandStatus.PROGRAMS_MANAGER_NOT_RUNNABLE_LANGUAGE, "Not Runnable Language"

        name = req.name

        if not manager.exists(name):
            return CommandStatus.PROGRAMS_MANAGER_FILE_DOES_NOT_EXIST, "File Doesn't Exist"

        return self.__save_autorun_infos(language_used, name, mode)

    def __callback_get_program_autorun_infos(self, _req):
        language, name, mode = self.__get_autorun_infos()

        if language is None:
            resp = GetProgramAutorunInfosResponse()
            resp.status = CommandStatus.PROGRAMS_MANAGER_AUTORUN_FAILURE
            resp.message = "Failed to get autorun infos"
            return resp

        return CommandStatus.SUCCESS, "OK", language, name, mode

    def __callback_execute_program_autorun(self, _req):
        if self.__program_is_running:
            return CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED, "Program is already running"

        language_obj, name, mode = self.__get_autorun_infos()

        if language_obj is None or language_obj == ProgramLanguage.NONE:
            rospy.loginfo("Programs Manager - Autorun file is not set")
            return CommandStatus.PROGRAMS_MANAGER_AUTORUN_FAILURE, "Autorun file is not set"

        if mode == self._str_to_autorun_mode["ONE_SHOT"]:
            self.__loop_autorun = False
        elif mode == self._str_to_autorun_mode["DISABLE"]:
            rospy.loginfo("Programs Manager - Autorun disable")
            return CommandStatus.PROGRAMS_MANAGER_AUTORUN_FAILURE, "Autorun disable"
        else:
            self.__loop_autorun = True

        self.__program_is_running = True
        self.__program_is_running_publisher.publish(self.__program_is_running)

        self.__execution_thread = Thread(target=self.__execute_program, name="execution_thread_programs_manager",
                                         args=[language_obj.used, name, None])
        self.__execution_thread.setDaemon(True)
        self.__execution_thread.start()

        message = "Executing autorun program ..."
        rospy.loginfo("Programs Manager - {}".format(message))
        return CommandStatus.SUCCESS, message

    # - Functions
    def __publish_program_list(self):
        programs_list = self.___get_program_list(ProgramLanguage(ProgramLanguage.ALL))
        self.__program_list_publisher.publish(*programs_list)

    def __execute_program(self, language_used, name=None, code_string=None):
        if language_used not in self._manager_map:
            return CommandStatus.PROGRAMS_MANAGER_UNKNOWN_LANGUAGE, "Unknown Language"
        manager = self._manager_map[language_used]
        if not manager.runnable:
            return CommandStatus.PROGRAMS_MANAGER_NOT_RUNNABLE_LANGUAGE, "Not Runnable Language"

        if code_string:
            prog_name = self.__execute_from_string_program_name
            manager.create(prog_name, code_string, description="", allow_overwrite=True)
        else:
            prog_name = name

        self.__execution_stopped = False

        try:
            if not self.__program_is_running:
                self.__program_is_running = True
                self.__program_is_running_publisher.publish(self.__program_is_running)

            ret, message = manager.execute(prog_name)
            while self.__loop_autorun and ret:
                ret, message = manager.execute(prog_name)

        except FileDoesNotExistException:
            self.__program_is_running = False
            self.__program_is_running_publisher.publish(self.__program_is_running)
            return CommandStatus.PROGRAMS_MANAGER_FILE_DOES_NOT_EXIST, "File Doesn't Exist"
        except Exception as e:
            self.__program_is_running = False
            self.__program_is_running_publisher.publish(self.__program_is_running)
            return CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED, str(e)

        self.__program_is_running = False
        self.__program_is_running_publisher.publish(self.__program_is_running)
        if ret:
            message = "Program execution success"
            rospy.loginfo("Programs Manager - {}".format(message))
            return CommandStatus.SUCCESS, message
        elif self.__execution_stopped:
            return CommandStatus.PREEMPTED, "Program successfully stopped"
        else:
            return CommandStatus.PROGRAMS_MANAGER_EXECUTION_FAILED, message

    def __get_autorun_infos(self):
        if not os.path.isfile(self.__autorun_file_path):
            return None, None, None
        with open(self.__autorun_file_path, "r") as input_file:
            yaml_file = yaml.load(input_file)

        mode = self._str_to_autorun_mode[yaml_file["mode"]]

        language = ProgramLanguage(self.__str_to_program_language_map[yaml_file["language"]])

        return language, yaml_file["name"], mode

    def __disable_autorun(self):
        self.__save_autorun_infos(ProgramLanguage.NONE, "", SetProgramAutorunRequest.DISABLE)
        return CommandStatus.SUCCESS, "Programs Manager - Disable Autorun Program"

    def __save_autorun_infos(self, language, name, mode):
        language_str = self.__program_language_to_str_map[language]
        mode_str = self.__autorun_mode_to_str[mode]

        json_dict = {"language": language_str, "name": name, "mode": mode_str}
        with open(self.__autorun_file_path, "w") as output_file:
            yaml.dump(json_dict, output_file, default_flow_style=False)

        message = "Programs Manager - Change autorun " \
                  "setting to file {} in {} mode".format(name,
                                                         mode_str.replace("_", " ").title())
        rospy.loginfo(message)

        return CommandStatus.SUCCESS, message

    def ___get_program_list(self, language_msg):
        language_used = language_msg.used
        if language_used != language_msg.ALL:
            # If language_used is unknown
            if language_used not in self._manager_map:
                return [], [], []
            else:
                manager = self._manager_map[language_used]
                prog_list, description_list = manager.get_all_names_with_description()

                programs_names = prog_list
                list_of_language_list = [ProgramLanguageList([language_msg])]
                programs_description = description_list
        else:
            prog_dict = {}
            for language, manager in self._manager_map.iteritems():
                # Get all names from a manager
                all_names = manager.get_all_names()
                # Adding them to dictionary
                for name in all_names:
                    if name[0] == ".":
                        # Secret file
                        continue
                    elif name not in prog_dict:
                        prog_dict[name] = [language]
                    else:
                        prog_dict[name].append(language)

            languages_list_list = []
            for lang_list in prog_dict.values():
                ros_lang_list = ProgramLanguageList([ProgramLanguage(lang) for lang in lang_list])
                languages_list_list.append(ros_lang_list)

            description_list = []
            for name, languages in prog_dict.iteritems():
                if ProgramLanguage.PYTHON2 not in languages:
                    description_list.append("")
                else:
                    description = self.__python2_manager.read_description(name)
                    description_list.append(description)

            programs_names = prog_dict.keys()
            list_of_language_list = languages_list_list
            programs_description = description_list

        return programs_names, list_of_language_list, programs_description

    # - Others functions

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
    rospy.init_node('niryo_programs_manager', anonymous=False, log_level=rospy.INFO)
    try:
        node = ProgramManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
