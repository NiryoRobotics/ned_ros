import rospy

# - Messages
from niryo_robot_sound.msg import SoundList

# - Services
from niryo_robot_sound.srv import PlaySound
from niryo_robot_msgs.srv import SetInt, Trigger
from niryo_robot_sound.srv import ManageSound, ManageSoundRequest
from niryo_robot_sound.srv import TextToSpeech

# Command Status
from niryo_robot_msgs.msg import CommandStatus


class SoundRosWrapperException(Exception):
    pass


def check_ned2_version(func):
    """
    Decorator that check the robot version
    """

    def wrap(*args, **kwargs):
        robot_instance = args[0]
        if robot_instance.hardware_version != 'ned2':
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
        if hardware_version == 'ned2':
            rospy.Subscriber('/niryo_robot_sound/sound_database', SoundList, self.__sound_database_callback)

    @property
    def hardware_version(self):
        return self.__hardware_version

    @property
    def sounds(self):
        return self.__sounds

    def __sound_database_callback(self, msg):
        self.__sound_duration = {sound.name: sound.duration for sound in msg.sounds}
        self.__sounds = list(self.__sound_duration.keys())

    @check_ned2_version
    def play(self, sound_name, wait_end=True, start_time_sec=0, end_time_sec=0):
        """
        Call service to play_sound according to SoundStateCommand.
        If failed, raise NiryoRosWrapperException

        :param sound_name: Name of the sound to play
        :type sound_name: string
        :param start_time_sec: start the sound from this value in seconds
        :type start_time_sec: float
        :param end_time_sec: end the sound at this value in seconds
        :type end_time_sec: float
        :param wait_end: wait for the end of the sound before exiting the function
        :type wait_end: bool
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_sound/play',
                                     PlaySound, sound_name, start_time_sec, end_time_sec, wait_end)
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    @check_ned2_version
    def set_volume(self, sound_volume):
        """
        Call service to set_volume to set the volume of Ned'sound accrding to sound_volume.
        If failed, raise NiryoRosWrapperException

        :param sound_volume: volume of the sound
        :type int: int (0: no sound, 100: max sound)
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_sound/set_volume',
                                     SetInt, sound_volume)
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    @check_ned2_version
    def stop(self):
        """
        Call service stop_sound to stop a sound being played.
        If failed, raise NiryoRosWrapperException

        :param None: take the sound being played
        :type None: None
        :return: status, message
        :rtype: (int, str)
        """
        result = self.__call_service('/niryo_robot_sound/stop', Trigger)
        rospy.sleep(0.1)
        return self.__classic_return_w_check(result)

    def delete_sound(self, sound_name):
        """
        Call service delete_sound to delete a sound on the RaspberryPi of the robot.
        If failed, raise NiryoRosWrapperException

        :param sound_name: name of the sound which needs to be deleted
        :type string: String
        :return: status, message
        :rtype: (int, str)
        """
        req = ManageSoundRequest(sound_name=sound_name, action=ManageSoundRequest.DELETE)
        result = self.__call_service('/niryo_robot_sound/manage', ManageSound, req)
        return self.__classic_return_w_check(result)

    def import_sound(self, sound_name, sound_data):
        """
        Call service import_sound to delete a sound on the RaspberryPi of the robot.
        If failed, raise NiryoRosWrapperException

        :param sound_name, sound_data: name of the sound which needs to be deleted,
               encoded data from the sound (wav or mp3), encoded data from the sound file (wav or mp3)
        :type string, string: String, String containing the encoded data of the sound file
        :return: status, message
        :rtype: (int, str)
        """
        print(sound_data)
        req = ManageSoundRequest(sound_name=sound_name, action=ManageSoundRequest.DELETE, sound_data=sound_data)
        result = self.__call_service('/niryo_robot_sound/send_sound', ManageSound, req)
        return self.__classic_return_w_check(result)

    @check_ned2_version
    def get_sound_duration(self, sound_name):
        if sound_name not in self.__sound_duration:
            raise SoundRosWrapperException("Sound name: {} not found".format(sound_name))

        return self.__sound_duration[sound_name]

    @check_ned2_version
    def say(self, text, language=0):
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
