# Lib
import rospy
from threading import Lock

# Messages
from std_msgs.msg import Empty
from niryo_robot_status.msg import RobotStatus

# Services
from niryo_robot_sound.srv import PlaySound

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from .sound_database import SoundDatabase
from .sound_player import SoundPlayer
from .text_to_speech import NiryoTextToSpeech


class SoundManager:
    """
    Object which configure the sound management on the robot
    """

    def __init__(self):
        # - Init
        self.__lock = Lock()
        self.__shutdown = False
        self.__sound_player = SoundPlayer()
        self.__sound_database = SoundDatabase()
        self.__text_to_speech = NiryoTextToSpeech(self, self.__sound_database)

        self.play_sound(self.__sound_database.wake_up_sound)

        self.__rpi_overheating = False
        self.__overheat_timer = None
        self.__error_sound_delay = rospy.get_param("~error_sound_delay")

        # - Subscribers
        self.__robot_status = RobotStatus.BOOTING
        self.__logs_status = RobotStatus.NONE

        self.__robot_status_sub = rospy.Subscriber('/niryo_robot_status/robot_status',
                                                   RobotStatus,
                                                   self.__callback_sub_robot_status)
        self.__niryo_studio_connection_sub = rospy.Subscriber('/niryo_studio_connection',
                                                              Empty,
                                                              self.__callback_niryo_studio)

        # - Services
        rospy.Service('/niryo_robot_sound/play', PlaySound, self.__callback_play_sound_user)
        rospy.Service('/niryo_robot_sound/overlay', PlaySound, self.__callback_overlay_sound)

        # Set a bool to mention this node is initialized
        rospy.set_param('~initialized', True)
        rospy.loginfo("Sound Interface - Started")

        rospy.on_shutdown(self.play_shutdown_sound)

    # - Callbacks

    def __callback_niryo_studio(self, _):
        sound = self.__sound_database.connection_sound
        self.__sound_player.overlay_sound(sound)

    def __callback_overlay_sound(self, msg):
        sound = self.__sound_database(msg.sound_name)
        if sound is None:
            return CommandStatus.SOUND_FILE_NOT_FOUND, "{} sound not found".format(msg.sound_name)
        play_process = self.__sound_player.overlay_sound(sound)

        if msg.wait_end:
            play_process.wait()
            return play_process.result, play_process.message

        return CommandStatus.SUCCESS, "{} sound play requested with success".format(msg.sound_name)

    def __callback_play_sound_user(self, msg):
        return self.play_user_sound(msg.sound_name, msg.start_time_sec, msg.end_time_sec, msg.wait_end)

    def __callback_sub_robot_status(self, msg):
        if self.check_shutdown(msg.robot_status):
            return

        self.check_overheating(msg.rpi_overheating)
        self.check_robot_status(msg.robot_status)
        self.check_log_status(msg.logs_status)

    def check_shutdown(self, new_robot_status):
        if self.__robot_status <= RobotStatus.SHUTDOWN or self.__shutdown:
            return True
        elif new_robot_status <= RobotStatus.SHUTDOWN:
            self.__robot_status = new_robot_status
            rospy.sleep(1.5)  # avoid ctrl+c
            self.play_shutdown_sound()
            return True
        return False

    def check_overheating(self, new_overheating_status):
        if new_overheating_status != self.__rpi_overheating:
            self.__rpi_overheating = new_overheating_status
            if self.__rpi_overheating:
                sound = self.__sound_database.error_sound
                self.play_sound(sound)
                self.__overheat_timer = rospy.Timer(rospy.Duration(self.__error_sound_delay),
                                                    self.__error_sound_callback)
                return True
            elif self.__overheat_timer is not None:
                self.__overheat_timer.shutdown()
                self.__overheat_timer = None
        return False

    def check_robot_status(self, new_robot_status):
        if self.__robot_status != new_robot_status:
            last_status = self.__robot_status
            self.__robot_status = new_robot_status

            if last_status in [RobotStatus.RUNNING_AUTONOMOUS, RobotStatus.LEARNING_MODE_AUTONOMOUS] \
                    and self.__robot_status not in [RobotStatus.RUNNING_AUTONOMOUS,
                                                    RobotStatus.LEARNING_MODE_AUTONOMOUS]:
                self.__sound_player.stop_all()

            if last_status == RobotStatus.BOOTING and self.__robot_status != RobotStatus.BOOTING:
                self.__sound_player.stop_current(fade_out=True)
                self.play_sound(self.__sound_database.robot_ready_sound)
                return True
            elif RobotStatus.SHUTDOWN < self.__robot_status <= RobotStatus.USER_PROGRAM_ERROR:
                self.play_sound(self.__sound_database.error_sound)
                return True
            elif new_robot_status == RobotStatus.CALIBRATION_IN_PROGRESS:
                self.play_sound(self.__sound_database.calibration_sound)
                return True
            elif last_status == RobotStatus.LEARNING_TRAJECTORY and \
                    new_robot_status != RobotStatus.LEARNING_TRAJECTORY:
                self.play_sound(self.__sound_database.learning_trajectory_sound)
                return True
            elif new_robot_status == RobotStatus.REBOOT_MOTOR:
                self.play_sound(self.__sound_database.reboot_sound)
                return True
        return False

    def check_log_status(self, new_logs_status):
        if self.__logs_status != new_logs_status:
            self.__logs_status = new_logs_status
            if self.__logs_status in [RobotStatus.ERROR, RobotStatus.FATAL_ERROR]:
                self.play_sound(self.__sound_database.error_sound)
                return True
        return False

    def __error_sound_callback(self, _):
        if self.__rpi_overheating:
            sound = self.__sound_database.error_sound
            self.play_sound(sound)
        elif self.__overheat_timer is not None:
            self.__overheat_timer.shutdown()
            self.__overheat_timer = None

    def play_sound(self, sound, start_time=0, end_time=0, wait=False):
        player_execution = self.__sound_player(sound, start_time, end_time)
        if wait:
            return player_execution.wait()

        return CommandStatus.SUCCESS, "{} sound played with success".format(player_execution.name)

    def play_user_sound(self, sound_name, start_time_sec=0, end_time_sec=0, wait_end=True):
        sound = self.__sound_database(sound_name)
        if sound is None:
            return CommandStatus.SOUND_FILE_NOT_FOUND, "{} sound not found".format(sound_name)

        return self.play_sound(sound, start_time_sec, end_time_sec, wait=wait_end)

    def play_shutdown_sound(self):
        with self.__lock:
            if not self.__shutdown:
                self.__shutdown = True

                if self.__overheat_timer is not None:
                    self.__overheat_timer.shutdown()
                    self.__overheat_timer = None

                self.__robot_status_sub.unregister()

                rospy.logdebug("Play shutdown sound")
                self.__sound_player.overlay_sound(self.__sound_database.sleep_sound).wait()
