# Lib
import rospy
import rospkg
import os
import base64

# Messages
from niryo_robot_sound.msg import SoundList

# Services
from niryo_robot_sound.srv import ManageSound

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_sound.sound_object import Sound, SoundFormatException, SoundFileException


class SoundDatabase:
    """
    Object which configure the sound management on the robot
    """

    def __init__(self):
        self.__simulation_mode = rospy.get_param("~simulation_mode")

        self.__user_sound_directory_path = os.path.expanduser(
            rospy.get_param("~path_user_sound_simulation" if self.__simulation_mode else "~path_user_sound"))
        if not os.path.isdir(self.__user_sound_directory_path):
            os.makedirs(self.__user_sound_directory_path)

        self.__robot_sound_directory_path = os.path.join(rospkg.RosPack().get_path('niryo_robot_sound'),
                                                         rospy.get_param("~path_robot_sound"))

        self.__system_sounds = [sound for sound in list(rospy.get_param("~robot_sounds").values())]
        self.__error_sound_name = rospy.get_param("~robot_sounds/error_sound")
        self.__turn_on_sound_name = rospy.get_param("~robot_sounds/turn_on_sound")
        self.__turn_off_sound_name = rospy.get_param("~robot_sounds/turn_off_sound")
        self.__connection_sound = rospy.get_param("~robot_sounds/connection_sound")
        self.__robot_ready_sound = rospy.get_param("~robot_sounds/robot_ready_sound")
        self.__calibration_sound = rospy.get_param("~robot_sounds/calibration_sound")
        self.__warning_sound = rospy.get_param("~robot_sounds/warn_sound")
        self.__reboot_sound = rospy.get_param("~robot_sounds/reboot_sound")
        self.__learning_trajectory_sound = rospy.get_param("~robot_sounds/learning_trajectory_sound")
        self.__robot_sounds = {}
        self.__user_sounds = {}

        self.__load_user_sounds()
        self.__load_robot_sounds()

        # - Publishers
        self.__sound_database_publisher = rospy.Publisher('/niryo_robot_sound/sound_database',
                                                          SoundList,
                                                          latch=True,
                                                          queue_size=10)
        self.__publish_sounds()

        # - Services
        rospy.Service('/niryo_robot_sound/manage', ManageSound, self.__callback_manage_sound)

    def __call__(self, *args, **kwargs):
        if len(args) >= 1:
            return self.get_sound(args[0])
        return None

    # - Properties and getters
    @property
    def user_sounds(self):
        return self.__user_sounds

    @property
    def robot_sounds(self):
        return self.__robot_sounds

    @property
    def error_sound(self):
        return self.__robot_sounds[self.__error_sound_name]

    @property
    def wake_up_sound(self):
        return self.__robot_sounds[self.__turn_on_sound_name]

    @property
    def sleep_sound(self):
        return self.__robot_sounds[self.__turn_off_sound_name]

    @property
    def connection_sound(self):
        return self.__robot_sounds[self.__connection_sound]

    @property
    def robot_ready_sound(self):
        return self.__robot_sounds[self.__robot_ready_sound]

    @property
    def calibration_sound(self):
        return self.__robot_sounds[self.__calibration_sound]

    @property
    def reboot_sound(self):
        return self.__robot_sounds[self.__reboot_sound]

    @property
    def warning_sound(self):
        return self.__robot_sounds[self.__warning_sound]

    @property
    def learning_trajectory_sound(self):
        return self.__robot_sounds[self.__learning_trajectory_sound]

    @property
    def state_sound_directory_path(self):
        return self.__robot_sound_directory_path

    @property
    def user_sound_directory_path(self):
        return self.__user_sound_directory_path

    def get_sound(self, sound_name):
        if sound_name in self.__user_sounds:
            return self.__user_sounds[sound_name]
        elif sound_name in self.__robot_sounds:
            return self.__robot_sounds[sound_name]
        return None

    # - Callbacks
    def __callback_manage_sound(self, req):
        if req.action == req.ADD:
            return self.add_sound(req.sound_name, req.sound_data)
        elif req.action == req.DELETE:
            return self.delete_sound(req.sound_name)
        else:
            return CommandStatus.FAILURE, "Incorrect action (ADD: 1, DELETE: 2)"

    def delete_sound(self, sound_name):
        if sound_name in self.__robot_sounds:
            return CommandStatus.PROTECTED_SOUND_NAME, "Failure to delete {} sound, this sound is protected".format(
                sound_name)
        elif sound_name not in self.__user_sounds:
            return CommandStatus.SOUND_FILE_NOT_FOUND, "Failure to delete {} sound, file doesn't exist".format(
                sound_name)

        try:
            sound_file_path = self.__user_sounds[sound_name].path
            self.__user_sounds.pop(sound_name)
            os.remove(sound_file_path)
            self.__publish_sounds()
        except OSError:
            return CommandStatus.FAILURE, "Failure to delete the {} sound , file doesn't exist".format(sound_name)

        return CommandStatus.SUCCESS, "{} Sound successfully deleted".format(sound_name)

    def add_sound(self, sound_name, raw_sound_data):
        if not (sound_name.endswith('.mp3') or sound_name.endswith('.wav')):
            return CommandStatus.INVALID_SOUND_FORMAT, "The {}file must be in .mp3 or .wav format".format(sound_name)

        if sound_name in self.__user_sounds:
            return CommandStatus.FAILURE, "Failure to write the {} sound, this name already exists".format(sound_name)
        elif sound_name in self.__robot_sounds:
            return CommandStatus.FAILURE, "Failure to write the {} sound, this name is protected".format(sound_name)

        # Split the sound_data to keep just the encoded data that we need
        try:
            decoded_base64_sound_data = raw_sound_data.split("base64,")[1]
        except IndexError:
            decoded_base64_sound_data = raw_sound_data

        encoded_base64_sound_data = decoded_base64_sound_data.encode('utf-8')

        # delete the padding to be able to decode the sound_data properly
        missing_padding = len(encoded_base64_sound_data) % 4
        if missing_padding != 0:
            encoded_base64_sound_data += b'=' * (4 - missing_padding)
        # decode the sound_data
        sound_data = base64.decodebytes(encoded_base64_sound_data)

        # Create the new soundFile and put the decoded sound_data on it
        sound_file_path = os.path.join(self.__user_sound_directory_path, sound_name)
        try:
            with open(sound_file_path, 'wb') as sound_file:
                sound_file.write(sound_data)
        except IOError:
            return CommandStatus.FAILURE, "Failure to write the {} sound".format(sound_name)
        try:
            self.__user_sounds[sound_name] = Sound(sound_name, sound_file_path)
            self.__publish_sounds()
        except SoundFormatException:
            os.remove(sound_file_path)
            return CommandStatus.INVALID_SOUND_FORMAT, "The {} file does not appear to be an audio file".format(
                sound_name)

        return CommandStatus.SUCCESS, "{} sound successfully deleted".format(sound_name)

    def refresh_user_sounds(self):
        self.__load_user_sounds()

    # # - Publishers

    def __load_user_sounds(self):
        # Get a list with the names of the user's sounds
        sound_names = os.listdir(self.__user_sound_directory_path)
        for sound_name in sound_names:
            if (sound_name.endswith('.mp3') or sound_name.endswith('.wav')) and sound_name not in self.__user_sounds:
                try:
                    self.__user_sounds[sound_name] = Sound(sound_name,
                                                           os.path.join(self.__user_sound_directory_path, sound_name))
                except (SoundFormatException, SoundFileException) as e:
                    rospy.logwarn(f"File {sound_name} doesn't appear to be an audio file: {e}")

    def __load_robot_sounds(self):
        self.__robot_sounds = {
            sound_name: Sound(sound_name, os.path.join(self.__robot_sound_directory_path, sound_name))
            for sound_name in self.__system_sounds
        }

    def __publish_sounds(self):
        msg = SoundList()
        msg.sounds = [
            sound.to_msg() for sound in list(self.__user_sounds.values()) + list(self.__robot_sounds.values())
        ]

        try:
            self.__sound_database_publisher.publish(msg)
        except rospy.ROSException:
            return
