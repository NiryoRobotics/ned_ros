
# Lib
import rospy

# Messages
from std_msgs.msg import String

# Services
from niryo_robot_msgs.srv import Trigger

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from sound_volume import VolumeManager


class SoundPlayer:
    """
    Object which configure the sound management on the robot
    """

    def __init__(self):
        # - Init
        self.__actual_sound = None
        self.__sound_database = []
        self.__volume_manager = VolumeManager()

        # - Publisher
        self.__sound_publisher = rospy.Publisher('/niryo_robot_sound/sound', String, latch=True, queue_size=10)
        self.__sound_publisher.publish("")

        # - Services
        rospy.Service('/niryo_robot_sound/stop', Trigger, self.__callback_stop)

    # - Callbacks
    def __callback_stop(self, _msg):
        if self.__actual_sound is not None and self.__actual_sound.is_playing():
            sound_name = self.__actual_sound.name
            self.stop()
            return CommandStatus.SUCCESS, "{} sound stopped".format(sound_name)
        else:
            return CommandStatus.SUCCESS, "No sound to be stopped"

    # - Main class usage
    def stop(self):
        if self.__actual_sound is not None:
            self.__actual_sound.stop()

    def stop_w_fade_out(self):
        old_volume = self.__volume_manager.raw_volume
        self.__volume_manager.fade_out()
        self.stop()
        self.__volume_manager.set_volume(old_volume)

    def play_sound(self, sound, start_time_sec=0, end_time_sec=0):
        self.__actual_sound = sound

        duration = 0 if (end_time_sec == 0 or
                         end_time_sec > sound.duration or
                         start_time_sec > end_time_sec) else end_time_sec - start_time_sec
        self.__sound_publisher.publish(sound.name)
        self.__actual_sound.play(volume=self.__volume_manager.volume, start_sec=start_time_sec, duration=duration)
        self.__actual_sound.wait_end()
        self.__actual_sound = None
        self.__sound_publisher.publish("")
        return CommandStatus.SUCCESS, "{} sound played".format(sound)

    def is_busy(self):
        return self.__actual_sound is not None

