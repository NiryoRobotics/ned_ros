#!/usr/bin/env python

# Lib
import threading
import rosgraph_msgs.msg
import rospy

# Command Status
from niryo_robot_msgs.msg import CommandStatus, SoftwareVersion

# Messages
from std_msgs.msg import Int32, String

# Services
from niryo_robot_msgs.srv import GetNameDescriptionList, SetBool, SetInt, Trigger, SetString

from niryo_robot_programs_manager.srv import SetProgramAutorun, SetProgramAutorunRequest, GetProgramAutorunInfos, \
    GetProgramList, ManageProgram, ManageProgramRequest, GetProgram, GetProgramRequest, ExecuteProgram, \
    ExecuteProgramRequest
from niryo_robot_database.srv import GetSettings, SetSettings

# Enums
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import *


class NiryoRosWrapper2Exception(Exception):
    pass


class NiryoRosWrapper2:
    LOGS_LEVELS = {
        rospy.INFO: 'INFO',
        rospy.WARN: 'WARNING',
        rospy.ERROR: 'ERROR',
        rospy.FATAL: 'FATAL',
        rospy.DEBUG: 'DEBUG'
    }

    def __init__(self):
        # - Getting ROS parameters
        self.__service_timeout = rospy.get_param("/niryo_robot/python_ros_wrapper/service_timeout")
        self.__simulation_mode = rospy.get_param("/niryo_robot/simulation_mode")
        self.__can_enabled = rospy.get_param("/niryo_robot_hardware_interface/can_enabled")
        self.__ttl_enabled = rospy.get_param("/niryo_robot_hardware_interface/ttl_enabled")

        # -- Subscribers
        self.__software_version = None
        rospy.Subscriber('/niryo_robot_hardware_interface/software_version', SoftwareVersion,
                         self.__callback_software_version)

        self.__highlighted_block = None
        rospy.Subscriber('/niryo_robot_blockly/highlight_block', String, self.__callback_highlight_block)

        self.__save_current_position_event = threading.Event()
        rospy.Subscriber('/niryo_robot/blockly/save_current_point', Int32, self.__callback_save_current_position)

        self.__save_trajectory_event = threading.Event()
        rospy.Subscriber('/niryo_robot/blockly/save_trajectory', Int32, self.__callback_save_trajectory)

        self.__logs = []
        rospy.Subscriber('/rosout_agg', rosgraph_msgs.msg.Log, self.__callback_rosout_agg)

        # - Action server

    def __del__(self):
        del self

    # -- Subscribers callbacks

    def __callback_software_version(self, msg):
        self.__software_version = msg

    def __callback_sub_hardware_status(self, hw_status):
        self.__hw_status = hw_status

    def __callback_sub_digital_io_state(self, digital_io_state):
        self.__digital_io_state = digital_io_state

    def __callback_sub_analog_io_state(self, analog_io_state):
        self.__analog_io_state = analog_io_state

    def __callback_sub_stream_video(self, compressed_image_message):
        self.__compressed_image_message = compressed_image_message

    def __callback_camera_intrinsics(self, camera_info_message):
        self.__camera_intrinsics_message = camera_info_message

    def __callback_sub_conveyors_feedback(self, conveyors_feedback):
        self.__conveyors_feedback = conveyors_feedback

    def __callback_rosout_agg(self, log):
        formatted_log = '[{}] [{}.{}]: {} - {}'.format(
            NiryoRosWrapper.LOGS_LEVELS[log.level],
            log.header.stamp.secs,
            log.header.stamp.nsecs,
            log.name,
            log.msg,
        )
        self.__logs.append(formatted_log)

    def __callback_highlight_block(self, block):
        self.__highlighted_block = block.data

    def __callback_save_current_position(self, _res):
        self.__save_current_position_event.set()

    def __callback_save_trajectory(self, _res):
        self.__save_trajectory_event.set()

    # -- Service & Action executors
    def __call_service(self, service_name, service_msg_type, *args):
        """
        Wait for the service called service_name
        Then call the service with args

        :param service_name:
        :param service_msg_type:
        :param args: Tuple of arguments
        :raises NiryoRosWrapper2Exception: Timeout during waiting of services
        :return: Response
        """
        # Connect to service
        try:
            rospy.wait_for_service(service_name, self.__service_timeout)
        except rospy.ROSException as e:
            raise NiryoRosWrapper2Exception(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise NiryoRosWrapper2Exception(e)

    # --- Functions interface
    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise NiryoRosWrapper2Exception("Error Code : {}\nMessage : {}".format(result.status, result.message))

    # - Hardware

    def get_can_enabled(self):
        """
        Get can_enabled
        """
        return self.__can_enabled

    def get_ttl_enabled(self):
        """
        Get ttl_enabled
        """
        return self.__ttl_enabled

    def debug_motors(self):
        """
        Debug the motors by going to each stop

        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_commander/motor_debug_start', SetInt, 0)
        return self.__classic_return_w_check(result)

    # - Button

    def __change_button_mode(self, mode):
        result = self.__call_service('/niryo_robot/rpi/change_button_mode', SetInt, mode)
        return self.__classic_return_w_check(result)

    def set_button_do_nothing(self):
        """
        Disable the button
        :return: status, message
        :rtype: (int, str)
        """
        return self.__change_button_mode(0)

    def set_button_trigger_sequence_autorun(self):
        """
        Set the button in trigger sequence autorun mode
        :return: status, message
        :rtype: (int, str)
        """
        return self.__change_button_mode(1)

    def set_button_blockly_save_point(self):
        """
        Set the button in blockly save point mode
        :return: status, message
        :rtype: (int, str)
        """
        return self.__change_button_mode(2)

    # - Software

    def get_software_version(self):
        """
        Get the robot software version

        :return: rpi_image_version, ros_niryo_robot_version, motor_names, stepper_firmware_versions
        :rtype: (str, str, list[str], list[str])
        """
        return self.__software_version

    def set_robot_name(self, name):
        """
        Set the robot name

        :param name: the new name of the robot
        :type name: str
        :return: status, message
        :rtype: int, str
        """
        req = SetString()
        req.data = name
        result = self.__call_service('/niryo_robot/wifi/set_robot_name', SetString, req)
        return self.__classic_return_w_check(result)

    def __call_shutdown_rpi(self, value):
        result = self.__call_service('/niryo_robot_rpi/shutdown_rpi', SetInt, value)
        return self.__classic_return_w_check(result)

    def shutdown_rpi(self):
        """
        Shutdown the rpi
        :return: status, message
        :rtype: (int, str)
        """
        return self.__call_shutdown_rpi(1)

    def reboot_rpi(self):
        """
        Shutdown the rpi
        :return: status, message
        :rtype: (int, str)
        """
        return self.__call_shutdown_rpi(2)

    # - Logs

    def get_logs(self):
        """
        Returns a generator iterating over all the logs published

        :return: the last logs
        :rtype: generator[str]
        """
        while len(self.__logs) > 0:
            yield self.__logs.pop(0)

    def purge_logs(self):
        """
        Purge the ros logs and discard the following
        Restart the robot to have logs again

        :return: status, message
        :rtype: (int, str)
        """
        # The request data is ignored by the service
        result = self.__call_service('/niryo_robot_rpi/purge_ros_logs', SetInt, 0)
        return self.__classic_return_w_check(result)

    def purge_logs_on_startup(self, value):
        """
        Purge the ros logs at rpi startup

        :param value: If the rpi have to purge the logs at startup
        :type value: bool
        :return: status, message
        :rtype: (int, str)
        """
        value = 1 if value is True else 0
        result = self.__call_service('/niryo_robot_rpi/set_purge_ros_log_on_startup', SetInt, value)
        return self.__classic_return_w_check(result)

    # - Blockly

    def get_highlighted_block(self):
        """
        Returns the blockly highlighted block

        :return: the highlighted block id
        :rtype: str
        """
        return self.__highlighted_block

    def get_save_point_event(self):
        """
        Returns an event which is set when a pose must be saved

        :return: the event
        :rtype: Event
        """
        return self.__save_current_position_event

    # - Programs

    def get_programs_list(self):
        """
        Get all the programs stored in the robot

        :return: names, descriptions
        :rtype: list[str], list[str]
        """
        result = self.__call_service('/niryo_robot_programs_manager/get_program_list', GetProgramList)
        return result.programs_names, result.programs_description

    def add_program(self, name, language, description, code):
        """
        Create a program

        :param name: the program's name
        :type name: str
        :param language: the program's language
        :type language: ProgramLanguage
        :param description: the program's description
        :type description: str
        :param code: the program's code
        :type code: str

        :return: status, message
        :rtype: (int, str)
        """
        req = ManageProgramRequest()
        req.cmd = 1
        req.name = name
        req.language.used = language
        req.description = description
        req.code = code
        result = self.__call_service('/niryo_robot_programs_manager/manage_program', ManageProgram, req)
        return self.__classic_return_w_check(result)

    def get_program(self, name, language):
        """
        Get a program's code

        :param name: the program's name
        :type name: str
        :param language: the program's language
        :type language: ProgramLanguage
        :return: the program's code
        :rtype: str
        """
        req = GetProgramRequest()
        req.name = name
        req.language.used = language
        result = self.__call_service('/niryo_robot_programs_manager/get_program', GetProgram, req)
        self.__check_result_status(result)
        return result.code

    def set_autorun(self, name, language, mode):
        """
        Set a program as the autorun

        :param name: the name of the program
        :type name: str
        :param language: the language of the program
        :type language: ProgramLanguage
        :param mode: the mode of the autorun
        :type mode: AutorunMode
        """
        req = SetProgramAutorunRequest()
        req.name = name
        req.language.used = language
        req.mode = mode
        result = self.__call_service('/niryo_robot_programs_manager/set_program_autorun', SetProgramAutorun, req)
        return self.__classic_return_w_check(result)

    def start_program(self, name, language):
        """
        Start a program

        :param name: The program's name
        :type name: str
        :param language: the program's language
        :type language: ProgramLanguage
        :return: status, message
        :rtype: (int, str)
        """
        req = ExecuteProgramRequest()
        req.name = name
        req.language.used = language
        result = self.__call_service('/niryo_robot_programs_manager/execute_program', ExecuteProgram, req)
        return self.__classic_return_w_check(result)

    def stop_program(self):
        """
        Stop the currently running program

        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_programs_manager/stop_program', Trigger)
        return self.__classic_return_w_check(result)

    def delete_program(self, name, language):
        """
        Delete a program

        :param name: the program's name
        :type name: str
        :param language: the program's language
        :type language: ProgramLanguage

        :return: status, message
        :rtype: (int, str)
        """
        req = ManageProgramRequest()
        req.cmd = -1
        req.name = name
        req.language.used = language

        result = self.__call_service('/niryo_robot_programs_manager/manage_program', ManageProgram, req)

        return self.__classic_return_w_check(result)

    # - Autorun
    def start_autorun(self):
        """
        Start the program set as autorun

        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_programs_manager/execute_program_autorun', Trigger)
        return self.__classic_return_w_check(result)

    def get_autorun(self):
        """
        Get the autorun infos

        :return: language, name, mode
        :rtype: (int, str, int)
        """
        result = self.__call_service('/niryo_robot_programs_manager/get_program_autorun_infos', GetProgramAutorunInfos)
        self.__check_result_status(result)
        return result.language.used, result.name, result.mode
