import rospy

from robot_status_enums import *
from robot_status_observer import RobotStatusObserver
from robot_nodes_observer import RobotNodesObserver
from robot_logs_observer import RobotLogsObserver

# - Messages
from niryo_robot_status.msg import RobotStatus
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_arm_commander.msg import PausePlanExecution

# - Services
from niryo_robot_msgs.srv import AdvertiseShutdown, AdvertiseShutdownRequest


class RobotStatusHandler(object):

    def __init__(self):
        self.__robot_status = RobotStatus.BOOTING
        self.__robot_message = ""
        self.__booting = True
        self.__autonomous_mode = False
        self.__robot_status_pub = None

        # - Observers
        self.__robot_nodes_observer = RobotNodesObserver(self)
        self.__robot_status_observer = RobotStatusObserver(self)
        self.__robot_logs_observer = RobotLogsObserver(self)

        # - Publisher
        self.__robot_status_pub = rospy.Publisher('~robot_status', RobotStatus, latch=True, queue_size=10)

        # - Services
        self.__advertise_shutdown_service = rospy.Service('~advertise_shutdown', AdvertiseShutdown,
                                                          self.__callback_advertise_shutdown)

        # - Handle shutdown
        rospy.on_shutdown(self.__shutdown_advertise)

        # Init Status
        self.__autonomous_mode = self.get_autonomous_status()
        self.__build_robot_status()
        self.__waiting_for_booting()
        self.__robot_status_observer.read_joint_limits()

    @property
    def robot_status(self):
        return self.__robot_status

    def __callback_advertise_shutdown(self, req):
        if req.value == AdvertiseShutdownRequest.SHUTDOWN:
            self.__shutdown_advertise(RobotStatus.SHUTDOWN)
            return CommandStatus.SUCCESS, "Success"
        elif req.value == AdvertiseShutdownRequest.REBOOT:
            self.__shutdown_advertise(RobotStatus.REBOOT)
            return CommandStatus.SUCCESS, "Success"
        elif req.value == AdvertiseShutdownRequest.UPDATE:
            self.__shutdown_advertise(RobotStatus.UPDATE)
            return CommandStatus.SUCCESS, "Success"

        return CommandStatus.FAILURE, "Bad arguments"

    def __waiting_for_booting(self):
        self.__booting = True
        self.__build_robot_status()

        booting_message = ""
        while self.__booting:
            if self.__robot_status_observer.hardware_status is not None \
                    and self.__robot_status_observer.hardware_status.connection_up:

                self.__booting = not self.__robot_nodes_observer.check_nodes_initialization()

                if self.__booting:
                    if self.__robot_nodes_observer.missing_vital_nodes:
                        booting_message = "Missing nodes: {}".format(self.__robot_nodes_observer.missing_vital_nodes)
                    else:
                        booting_message = "Not initialized nodes: {}".format(
                            self.__robot_nodes_observer.not_initialized_nodes)
                    if booting_message != self.__robot_message:
                        self.__robot_message = booting_message
                        self.__publish()

            rospy.sleep(0.1)

        self.__robot_message = ""
        self.__build_robot_status()
        self.__robot_nodes_observer.start_nodes_check_loop()  # Start nodes checker observer

    def get_autonomous_status(self):
        return self.__robot_nodes_observer.check_user_node or \
               self.__robot_status_observer.program_is_running or \
               self.__robot_status_observer.is_tcp_client_connected or \
               self.__robot_nodes_observer.pyniryo_connected

    def advertise_new_state(self):
        if self.__robot_status_pub is not None:
            self.__build_robot_status()

    def advertise_new_logs(self):
        if self.__robot_status_pub is not None and self.__robot_status > RobotStatus.SHUTDOWN:
            self.__publish()

    def advertise_warning(self):
        if self.__robot_status_pub is not None:
            self.__publish()

    def __shutdown_advertise(self, status=RobotStatus.SHUTDOWN):
        self.__robot_status = status
        self.__robot_message = ""
        self.__booting = False
        self.__log_status = RobotStatus.NONE
        self.__log_msg = ""
        self.__publish()

    def __build_robot_status(self):
        # - Hardware status
        if self.__robot_status <= RobotStatus.SHUTDOWN:
            return
        elif self.__booting:
            new_robot_status, new_robot_message = RobotStatus.BOOTING, "Robot is booting"
        elif not self.__robot_nodes_observer.are_vital_nodes_alive:
            new_robot_status = RobotStatus.FATAL_ERROR
            new_robot_message = "{} nodes are missing".format(self.__robot_nodes_observer.missing_vital_nodes)
        else:
            new_robot_status, new_robot_message = self.__check_hardware_error()

        # - Program or motion errors
        if self.__robot_status_observer.collision_detected:
            new_robot_status, new_robot_message = RobotStatus.COLLISION, "Robot collision detected"
            self.__robot_status_observer.collision_detected = False
        elif self.__robot_status == RobotStatus.COLLISION and \
                (self.__robot_status_observer.program_error or self.__robot_status_observer.learning_mode_state):
            # Stay at collision status
            return
        elif self.__robot_status_observer.program_error:
            new_robot_status = RobotStatus.USER_PROGRAM_ERROR
            new_robot_message = self.__robot_status_observer.program_error_message

        # - Robot status - No errors
        if new_robot_status == RobotStatus.UNKNOWN:
            self.__autonomous_mode = self.get_autonomous_status()
            if self.__robot_status_observer.hardware_status.hardware_state:
                new_robot_status, new_robot_message = RobotStatus.REBOOT_MOTOR, "Rebooting one or more motors"
            elif self.__robot_status_observer.hardware_status.calibration_in_progress:
                new_robot_status, new_robot_message = RobotStatus.CALIBRATION_IN_PROGRESS, "Calibration in progress"
            elif self.__robot_status_observer.hardware_status.calibration_needed:
                new_robot_status, new_robot_message = RobotStatus.CALIBRATION_NEEDED, "Calibration needed"
            elif self.__robot_status_observer.learning_mode_state and not self.__autonomous_mode:
                new_robot_status, new_robot_message = RobotStatus.LEARNING_MODE, "Learning mode activated"
            elif self.__robot_status_observer.pause_state == PausePlanExecution.PAUSE:
                new_robot_status, new_robot_message = RobotStatus.PAUSE, "Program paused"
            elif self.__robot_status_observer.is_debug_motor_active:
                new_robot_status, new_robot_message = RobotStatus.RUNNING_DEBUG, "Debug program is running"
            elif self.__autonomous_mode:
                if self.__robot_status_observer.learning_mode_state:
                    new_robot_status = RobotStatus.LEARNING_MODE_AUTONOMOUS
                    new_robot_message = "Program is running and learning_mode active"
                else:
                    new_robot_status, new_robot_message = RobotStatus.RUNNING_AUTONOMOUS, "Program is running"
            elif self.__robot_status_observer.motion_goal_is_active or self.__robot_status_observer.jog_is_enabled:
                new_robot_status, new_robot_message = RobotStatus.MOVING, "Robot is moving"
            else:
                new_robot_status, new_robot_message = RobotStatus.STANDBY, "Standby, nothing else to say"

        if self.__robot_status != new_robot_status or self.__robot_message != new_robot_message:
            self.__robot_status = new_robot_status
            self.__robot_message = new_robot_message
            self.__publish()

    def __publish(self):
        robot_status_msg = RobotStatus()

        robot_status_msg.robot_status = self.__robot_status
        robot_status_msg.robot_status_str = ROBOT_STATUS_TO_STR[robot_status_msg.robot_status]
        robot_status_msg.robot_message = self.__robot_message

        robot_status_msg.logs_status = self.__robot_logs_observer.log_status
        robot_status_msg.logs_status_str = LOG_STATUS_TO_STR[robot_status_msg.logs_status]
        robot_status_msg.logs_message = self.__robot_logs_observer.log_message

        robot_status_msg.out_of_bounds = self.__robot_status_observer.out_of_bounds
        robot_status_msg.rpi_overheating = self.__robot_status_observer.rpi_overheating

        self.__robot_status_pub.publish(robot_status_msg)

        if RobotStatus.SHUTDOWN < robot_status_msg.robot_status < RobotStatus.NONE:
            rospy.logwarn(
                "[Robot Status] - {} - {}".format(robot_status_msg.robot_status_str, robot_status_msg.robot_message))

    def __check_hardware_error(self):
        new_robot_status = RobotStatus.UNKNOWN
        new_robot_message = ""

        if self.__robot_status_observer.hardware_status.error_message:
            new_robot_status = RobotStatus.MOTOR_ERROR
            new_robot_message = self.__robot_status_observer.hardware_status.error_message
        else:
            for motor_index, motor_error in enumerate(
                    self.__robot_status_observer.hardware_status.hardware_errors_message):
                if motor_error and not (motor_index > 5 and motor_error == "Overload"):
                    new_robot_status = RobotStatus.MOTOR_ERROR
                    self.__robot_message += "{}: {}\n".format(
                        self.__robot_status_observer.hardware_status.motor_names[motor_index],
                        motor_error)

        return new_robot_status, new_robot_message
