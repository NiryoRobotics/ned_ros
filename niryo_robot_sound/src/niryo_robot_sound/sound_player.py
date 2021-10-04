
# Lib
import rospy

# Messages
from std_msgs.msg import Int8, String

# Services
from niryo_robot_msgs.srv import SetBool, SetFloat, SetInt, Trigger

# Command Status
from niryo_robot_msgs.msg import CommandStatus


class SoundPlayer:
    """
    Object which configure the sound management on the robot
    """

    def __init__(self):
        # - Init
        self.__actual_sound = None
        self.__sound_database = []

        # - Publisher
        self.__volume_state_publisher = rospy.Publisher('/niryo_robot_sound/volume', Int8, latch=True,
                                                        queue_size=10)

        self.__sound_publisher = rospy.Publisher('/niryo_robot_sound/sound', String, latch=True, queue_size=10)
        self.__sound_publisher.publish("")

        self.__volume_percentage = None
        self.__max_volume = rospy.get_param("~max_volume")
        self.__min_volume = rospy.get_param("~min_volume")

        self.set_volume(rospy.get_param("~default_volume"))

        # - Services
        rospy.Service('/niryo_robot_sound/set_volume', SetInt, self.__callback_set_volume)
        rospy.Service('/niryo_robot_sound/stop', Trigger, self.__callback_stop)

    # - Callbacks
    def __callback_set_volume(self, msg):
        self.set_volume(msg.value)
        return CommandStatus.SUCCESS, "Volume set at {}%".format(self.__volume_percentage)

    def __callback_stop(self, _msg):
        if self.__actual_sound is not None and self.__actual_sound.is_playing():
            sound_name = self.__actual_sound.name
            self.stop()
            return CommandStatus.SUCCESS, "{} sound stopped".format(sound_name)
        else:
            return CommandStatus.SUCCESS, "No sound to be stopped"

    # - Main class usage
    def set_volume(self, percentage):
        if percentage != self.__volume_percentage:
            self.__volume_percentage = self.__check_before_set_volume(percentage)
            self.__volume_state_publisher.publish(self.__volume_percentage)

    def stop(self):
        if self.__actual_sound is not None:
            self.__actual_sound.stop()

    def play_sound(self, sound, start_time_sec=0, end_time_sec=0):
        self.__actual_sound = sound

        duration = 0 if (end_time_sec == 0 or
                         end_time_sec > sound.duration or
                         start_time_sec > end_time_sec) else end_time_sec - start_time_sec
        self.__sound_publisher.publish(sound.name)
        self.__actual_sound.play(volume=self.__volume_percentage, start_sec=start_time_sec, duration=duration)
        self.__actual_sound.wait_end()
        self.__actual_sound = None
        self.__sound_publisher.publish("")
        return CommandStatus.SUCCESS, "{} sound played".format(sound)

    # - Check sound
    def __check_before_set_volume(self, volume):
        """
        Check if the volume set by the user is correct.
        Volume should be between 0 and 100.

        :return: A float corresponding to the volume
        :rtype: float
        """
        return max(self.__min_volume, min(self.__max_volume, volume))
