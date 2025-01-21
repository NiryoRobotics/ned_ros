import rospy

# - Messages
from niryo_robot_sound.msg import SoundList

# - Services
from niryo_robot_sound.srv import PlaySound, PlaySoundRequest
from niryo_robot_msgs.srv import SetInt, Trigger
from niryo_robot_sound.srv import ManageSound, ManageSoundRequest
from niryo_robot_sound.srv import TextToSpeech

# Command Status
from niryo_robot_msgs.msg import CommandStatus
from std_msgs.msg import String


class SoundRosWrapperException(Exception):
    pass


def check_ned2_version(func):
    """
    Decorator that check the robot version
    """

    def wrap(*args, **kwargs):
        robot_instance = args[0]
        if robot_instance.hardware_version not in ['ned2', 'ned3pro']:
            raise SoundRosWrapperException(
                "Error Code : {}\nMessage : Wrong robot hardware version, feature only available on Ned2".format(
                    CommandStatus.BAD_HARDWARE_VERSION))

        return func(*args, **kwargs)

    return wrap


class SoundRosWrapper(object):

    def __init__(self, hardware_version='ned2', service_timeout=1):
        self.__service_timeout = service_timeout
        self.__hardware_version = hardware_version

        self.__sounds = []
        self.__sound_duration = {}
        self.__current_sound = ''
        if hardware_version in ['ned2', 'ned3pro']:
            rospy.Subscriber('/niryo_robot_sound/sound_database', SoundList, self.__sound_database_callback)
            rospy.Subscriber('/niryo_robot_sound/sound', String, self.__sound_callback)

    @property
    def hardware_version(self):
        return self.__hardware_version

    def __check_ned_2_version(self):
        if self.__hardware_version not in ['ned2', 'ned3pro']:
            raise SoundRosWrapperException(
                "Error Code : {}\nMessage : Wrong robot hardware version, feature only available on Ned2".format(
                    CommandStatus.BAD_HARDWARE_VERSION))

    @property
    def sounds(self):
        """
        Get sound name list

        :return: list of the sounds of the robot
        :rtype: list[string]
        """
        return self.__sounds

    @property
    def current_sound(self):
        """
        Get the current sound being played

        :return: current sound name
        :rtype: Optional[str]
        """
        return self.__current_sound or None

    def __sound_database_callback(self, msg):
        self.__sound_duration = {sound.name: sound.duration for sound in msg.sounds}
        self.__sounds = list(self.__sound_duration.keys())

    def __sound_callback(self, msg):
        self.__current_sound = msg.data

    #    @check_ned2_version
    def play(self, sound_name, wait_end=True, start_time_sec=0, end_time_sec=0):
        """
        Play a sound from the robot
        If failed, raise NiryoRosWrapperException

        :param sound_name: Name of the sound to play
        :type sound_name: str
        :param start_time_sec: start the sound from this value in seconds
        :type start_time_sec: float
        :param end_time_sec: end the sound at this value in seconds
        :type end_time_sec: float
        :param wait_end: wait for the end of the sound before exiting the function
        :type wait_end: bool
        :return: status, message
        :rtype: (int, str)
        """
        self.__check_ned_2_version()
        result = self.__call_service(
            '/niryo_robot_sound/play',
            PlaySound,
            PlaySoundRequest(sound_name=sound_name,
                             start_time_sec=start_time_sec,
                             end_time_sec=end_time_sec,
                             wait_end=wait_end))
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    #    @check_ned2_version
    def set_volume(self, sound_volume):
        """
        Set the volume percentage of the robot.
        If failed, raise NiryoRosWrapperException

        :param sound_volume: volume percentage of the sound (0: no sound, 100: max sound)
        :type sound_volume: int
        :return: status, message
        :rtype: (int, str)
        """
        self.__check_ned_2_version()
        result = self.__call_service('/niryo_robot_sound/set_volume', SetInt, sound_volume)
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    #    @check_ned2_version
    def stop(self):
        """
        Stop a sound being played.
        If failed, raise NiryoRosWrapperException

        :return: status, message
        :rtype: (int, str)
        """
        self.__check_ned_2_version()
        result = self.__call_service('/niryo_robot_sound/stop', Trigger)
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    #    @check_ned2_version
    def delete_sound(self, sound_name):
        """
        Delete a sound on the RaspberryPi of the robot.
        If failed, raise NiryoRosWrapperException

        :param sound_name: name of the sound which needs to be deleted
        :type sound_name: str
        :return: status, message
        :rtype: (int, str)
        """
        self.__check_ned_2_version()
        req = ManageSoundRequest(sound_name=sound_name, action=ManageSoundRequest.DELETE)
        result = self.__call_service('/niryo_robot_sound/manage', ManageSound, req)
        return self.__classic_return_w_check(result)

    #    @check_ned2_version
    def import_sound(self, sound_name, sound_data):
        """
        Delete a sound on the RaspberryPi of the robot.
        If failed, raise NiryoRosWrapperException

        :param sound_name: name of the sound which needs to be deleted
        :type sound_name: str
        :param sound_data: String containing the encoded data of the sound file (wav or mp3)
        :type sound_data: str
        :return: status, message
        :rtype: (int, str)
        """
        self.__check_ned_2_version()

        import base64

        sound_data_b64 = base64.b64encode(sound_data)
        req = ManageSoundRequest(sound_name=sound_name, action=ManageSoundRequest.DELETE, sound_data=sound_data_b64)
        result = self.__call_service('/niryo_robot_sound/send_sound', ManageSound, req)
        return self.__classic_return_w_check(result)

    #    @check_ned2_version
    def get_sound_duration(self, sound_name):
        """
        Returns the duration in seconds of a sound stored in the robot database
        raise SoundRosWrapperException if the sound doesn't exists

        :param sound_name: name of sound
        :type sound_name: str
        :return: sound duration in seconds
        :rtype: float
        """
        self.__check_ned_2_version()
        if sound_name not in self.__sound_duration:
            raise SoundRosWrapperException("Sound name: {} not found".format(sound_name))

        return self.__sound_duration[sound_name]

    #    @check_ned2_version
    def say(self, text, language=0):
        """
        Use gtts (Google Text To Speech) to interpret a string as sound
        Languages available are:
        - English: 0
        - French: 1
        - Spanish: 2
        - Mandarin: 3
        - Portuguese: 4


        :param text: text to speek < 100 char
        :type text: string
        :param language: language of the text
        :type language: int
        :return: status, message
        :rtype: (int, str)
        """
        self.__check_ned_2_version()
        result = self.__call_service('/niryo_robot_sound/text_to_speech', TextToSpeech, text, language)

        if not result.success:
            raise SoundRosWrapperException("Message : {}".format(result.message))

        return result.success, result.message

    # --- Functions interface
    def __call_service(self, service_name, service_msg_type, *args):
        """
        Wait for the service called service_name
        Then call the service with args

        :param service_name:
        :param service_msg_type:
        :param args: Tuple of arguments
        :raises NiryoRosWrapperException: Timeout during waiting of services
        :return: Response
        """
        # Connect to service
        try:
            rospy.wait_for_service(service_name, self.__service_timeout)
        except rospy.ROSException as e:
            raise SoundRosWrapperException(e)

        # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException as e:
            raise SoundRosWrapperException(e)

    def __classic_return_w_check(self, result):
        self.__check_result_status(result)
        return result.status, result.message

    @staticmethod
    def __check_result_status(result):
        if result.status < 0:
            raise SoundRosWrapperException("Error Code : {}\nMessage : {}".format(result.status, result.message))
