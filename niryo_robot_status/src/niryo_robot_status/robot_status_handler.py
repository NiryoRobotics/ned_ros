import rospy

from .robot_status_enums import *
from .robot_status_observer import RobotStatusObserver
from .robot_nodes_observer import RobotNodesObserver
from .robot_logs_observer import RobotLogsObserver

# - Messages
from std_msgs.msg import Empty
from niryo_robot_status.msg import RobotStatus
from niryo_robot_msgs.msg import HardwareStatus
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_arm_commander.msg import PausePlanExecution

# - Services
from niryo_robot_msgs.srv import AdvertiseShutdown, AdvertiseShutdownRequest


class NoChangesException(Exception):
    pass


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
        self.__robot_heartbeat_pub = rospy.Publisher('~heartbeat', Empty, queue_size=1)
        rospy.Timer(rospy.Duration(1), lambda _: self.__robot_heartbeat_pub.publish())

        # - Services
        self.__advertise_shutdown_service = rospy.Service('~advertise_shutdown',
                                                          AdvertiseShutdown,
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

    def __waiting_for_booting(self, timeout=120):
        self.__booting = True
        self.__build_robot_status()

        rate = rospy.Rate(2)
        start_time = rospy.Time.now()
        while self.__booting and not rospy.is_shutdown():
            if self.__robot_status_observer.hardware_status is not None \
                    and self.__robot_status_observer.hardware_status.connection_up:

                self.__booting = not self.__robot_nodes_observer.check_nodes_initialization()

                if self.__booting:
                    if (rospy.Time.now() - start_time).to_sec() > timeout:
                        self.__booting = False
                        break
                    elif self.__robot_nodes_observer.missing_vital_nodes:
                        booting_message = "Missing nodes: {}".format(self.__robot_nodes_observer.missing_vital_nodes)
                    else:
                        booting_message = "Not initialized nodes: {}".format(
                            self.__robot_nodes_observer.not_initialized_nodes)
                    if booting_message != self.__robot_message:
                        self.__robot_message = booting_message
                        self.__publish()
            rate.sleep()

        rospy.loginfo('\033[5;34;1m\\(^-^)/ \033[3mRobot ready to receive commands! \033[0;5;34;1m\\(^-^)/\033[0m')

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
        self.__robot_message = ROBOT_STATUS_TO_MESSAGE[status]
        self.__booting = False
        self.__log_status = RobotStatus.NONE
        self.__log_msg = ""
        self.__publish()

    def __build_robot_status(self):
        try:

            new_robot_status, new_robot_message = self.__check_estop_state()

            if new_robot_status == RobotStatus.UNKNOWN:
                # - Ros status -> Booting, Node crash, etc..
                new_robot_status, new_robot_message = self.__check_ros_state()

            if new_robot_status == RobotStatus.UNKNOWN:
                # Motor Error
                new_robot_status, new_robot_message = self.__check_hardware_error()

            if new_robot_status == RobotStatus.UNKNOWN:
                # - Program or motion errors
                new_robot_status, new_robot_message = self.__check_motion_errors()

            if new_robot_status == RobotStatus.UNKNOWN:
                # - Robot status - No errors
                new_robot_status, new_robot_message = self.__check_regular_robot_status()
        except NoChangesException:
            return

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
            rospy.logwarn("[Robot Status] - {} - {}".format(robot_status_msg.robot_status_str,
                                                            robot_status_msg.robot_message))

    def __check_estop_state(self):
        new_robot_status = RobotStatus.UNKNOWN
        new_robot_message = ""

        if self.__robot_status_observer.estop_detected:
            new_robot_status = RobotStatus.ESTOP
            new_robot_message = "Emergency stop triggered"

        return new_robot_status, new_robot_message

    def __check_hardware_error(self):
        new_robot_status = RobotStatus.UNKNOWN
        new_robot_message = ""

        if HardwareStatus.REBOOT == self.__robot_status_observer.hardware_status.hardware_state:
            new_robot_status, new_robot_message = RobotStatus.REBOOT_MOTOR, "Rebooting one or more motors"
        elif self.__robot_status_observer.hardware_status.error_message or \
                HardwareStatus.ERROR == self.__robot_status_observer.hardware_status.hardware_state:
            new_robot_status = RobotStatus.MOTOR_ERROR
            new_robot_message = self.__robot_status_observer.hardware_status.error_message
        else:
            for motor_index, motor_error in enumerate(
                    self.__robot_status_observer.hardware_status.hardware_errors_message):
                if motor_error and not (motor_index > 5 and motor_error == "Overload"):
                    new_robot_status = RobotStatus.MOTOR_ERROR
                    new_robot_message += "{}: {}\n".format(
                        self.__robot_status_observer.hardware_status.motor_names[motor_index], motor_error)

        return new_robot_status, new_robot_message

    def __check_regular_robot_status(self):
        self.__autonomous_mode = self.get_autonomous_status()
        if self.__robot_status_observer.hardware_status.calibration_in_progress:
            robot_status = RobotStatus.CALIBRATION_IN_PROGRESS
        elif self.__robot_status_observer.hardware_status.calibration_needed:
            robot_status = RobotStatus.CALIBRATION_NEEDED
        elif self.__robot_status_observer.learning_trajectory:
            robot_status = RobotStatus.LEARNING_TRAJECTORY
        elif self.__robot_status_observer.learning_mode_state and not self.__autonomous_mode:
            robot_status = RobotStatus.LEARNING_MODE
        elif self.__robot_status_observer.pause_state == PausePlanExecution.PAUSE:
            robot_status = RobotStatus.PAUSE
        elif self.__robot_status_observer.is_debug_motor_active:
            robot_status = RobotStatus.RUNNING_DEBUG
        elif self.__autonomous_mode:
            if self.__robot_status_observer.learning_mode_state:
                robot_status = RobotStatus.LEARNING_MODE_AUTONOMOUS
            else:
                robot_status = RobotStatus.RUNNING_AUTONOMOUS
        elif self.__robot_status_observer.motion_goal_is_active or self.__robot_status_observer.jog_is_enabled:
            robot_status = RobotStatus.MOVING
        # waiting for user intervention to clear these errors
        elif self.__robot_status not in [RobotStatus.COLLISION, RobotStatus.USER_PROGRAM_ERROR]:
            robot_status = RobotStatus.STANDBY
        else:
            raise NoChangesException

        return robot_status, ROBOT_STATUS_TO_MESSAGE[robot_status]

    def __check_motion_errors(self):
        # - Program or motion errors
        if self.__robot_status_observer.collision_detected:
            robot_status, robot_message = RobotStatus.COLLISION, ROBOT_STATUS_TO_MESSAGE[RobotStatus.COLLISION]
            self.__robot_status_observer.collision_detected = False
        elif self.__robot_status == RobotStatus.COLLISION and self.__robot_status_observer.program_error:
            # Stay at collision status
            raise NoChangesException
        elif self.__robot_status_observer.program_error:
            robot_status = RobotStatus.USER_PROGRAM_ERROR
            robot_message = self.__robot_status_observer.program_error_message
        else:
            robot_status, robot_message = RobotStatus.UNKNOWN, ""

        return robot_status, robot_message

    def __check_ros_state(self):
        if self.__robot_status <= RobotStatus.SHUTDOWN:
            raise NoChangesException
        elif self.__booting:
            robot_status, robot_message = RobotStatus.BOOTING, ROBOT_STATUS_TO_MESSAGE[RobotStatus.BOOTING]
        elif not self.__robot_nodes_observer.are_vital_nodes_alive:
            robot_status = RobotStatus.FATAL_ERROR
            robot_message = "{} nodes are missing".format(self.__robot_nodes_observer.missing_vital_nodes)
        else:
            robot_status, robot_message = RobotStatus.UNKNOWN, ""

        return robot_status, robot_message
