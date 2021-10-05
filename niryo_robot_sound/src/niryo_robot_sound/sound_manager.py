# Lib
import rospy
from threading import Thread

# Messages
from niryo_robot_status.msg import RobotStatus

# Services
from niryo_robot_sound.srv import PlaySound

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_sound.sound_database import SoundDatabase
from niryo_robot_sound.sound_player import SoundPlayer


class SoundManager:
    """
    Object which configure the sound management on the robot
    """

    def __init__(self):
        # - Init
        self.__sound_player = SoundPlayer()
        self.__sound_database = SoundDatabase()

        self.__sound_thread = Thread()

        self.__rpi_overheating = False
        self.__overheat_timer = None
        self.__error_sound_delay = rospy.get_param("~error_sound_delay")

        # - Subscribers
        self.__robot_status = RobotStatus.BOOTING
        self.__logs_status = RobotStatus.NONE
        rospy.Subscriber('/niryo_robot_status/robot_status', RobotStatus,
                         self.__callback_sub_robot_status)

        # - Services
        rospy.Service('/niryo_robot_sound/play_sound', PlaySound, self.__callback_play_sound_user)

        # Set a bool to mention this node is initialized
        rospy.set_param('~initialized', True)
        rospy.loginfo("Sound Interface - Started")

    # - Callbacks
    def __callback_sub_robot_status(self, msg):
        if msg.rpi_overheating != self.__rpi_overheating:
            self.__rpi_overheating = msg.rpi_overheating
            if self.__rpi_overheating:
                sound = self.__sound_database.error_sound
                self.play_sound(sound)
                self.__overheat_timer = rospy.Timer(rospy.Duration(self.__error_sound_delay),
                                                    self.__error_sound_callback)
            elif self.__overheat_timer is not None:
                self.__overheat_timer.shutdown()
                self.__overheat_timer = None

        if self.__robot_status != msg.robot_status:
            last_status = self.__robot_status
            self.__robot_status = msg.robot_status

            if last_status == RobotStatus.BOOTING and self.__robot_status != RobotStatus.BOOTING:
                sound = self.__sound_database.wake_up_sound
                self.play_sound(sound)
            elif self.__robot_status in [RobotStatus.FATAL_ERROR, RobotStatus.MOTOR_ERROR]:
                sound = self.__sound_database.error_sound
                self.play_sound(sound)

        if self.__logs_status != msg.logs_status:
            self.__logs_status = msg.logs_status
            if self.__logs_status in [RobotStatus.ERROR, RobotStatus.FATAL_ERROR]:
                self.play_sound(self.__sound_database.error_sound)

    def __callback_play_sound_user(self, msg):
        sound_name = msg.sound_name
        sound = self.__sound_database(sound_name)
        if sound is None:
            return CommandStatus.SOUND_FILE_NOT_FOUND, "{} sound not found".format(sound_name)

        self.play_sound(sound, msg.start_time_sec, msg.end_time_sec, wait=msg.wait_end)

        if sound.preempted:
            return CommandStatus.SUCCESS, "{} sound preempted".format(sound_name)
        return CommandStatus.SUCCESS, "{} sound played with success".format(sound_name)

    def __error_sound_callback(self, _):
        if self.__rpi_overheating:
            sound = self.__sound_database.error_sound
            self.play_sound(sound)
        elif self.__overheat_timer is not None:
            self.__overheat_timer.shutdown()
            self.__overheat_timer = None

    def play_sound(self, sound, start_time=0, end_time=0, wait=False):
        if self.__sound_thread.is_alive():
            self.__sound_player.stop()
            self.__sound_thread.join()

        self.__sound_thread = Thread(target=self.__sound_player.play_sound, args=(sound, start_time, end_time))
        self.__sound_thread.start()

        if wait:
            while not rospy.is_shutdown() and self.__sound_thread.is_alive():
                self.__sound_thread.join(timeout=0.1)

    def play_shutdown_sound(self):
        if self.__overheat_timer is not None:
            self.__overheat_timer.shutdown()
            self.__overheat_timer = None

        if self.__sound_thread.is_alive():
            self.__sound_player.stop()
            self.__sound_thread.join()

        sound = self.__sound_database.sleep_sound
        self.__sound_player.play_sound(sound)
