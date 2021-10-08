# Lib
import os
import subprocess
import rospy

from niryo_robot_sound.msg import SoundObject


class Sound(object):

    def __init__(self, name, path):
        self.__name = name
        self.__path = path

        rospy.logdebug(self.__name, self.__path)
        if not self.exists():
            raise IOError("File {} doesn't exists".format(self.__path))

        self.__duration = self.__get_sound_duration()
        self.__preempted = False

        self.__playing_instance = None

    def __str__(self):
        return self.__name

    def __repr__(self):
        return self.__str__()

    def __call__(self, *args, **kwargs):
        return self.play()

    @property
    def name(self):
        return self.__name

    @property
    def path(self):
        return self.__path

    @property
    def duration(self):
        return self.__duration

    @property
    def preempted(self):
        return self.__preempted

    def to_msg(self):
        return SoundObject(self.__name, self.__duration)

    def exists(self):
        return os.path.exists(self.__path)

    def play(self, volume=100, start_sec=0, duration=0):
        if duration > 0:
            args = (
                "ffplay", "-nodisp", "-loglevel", "quiet", "-autoexit", "-volume", str(volume), "-ss", str(start_sec),
                "-t", str(duration), self.__path)
        else:
            args = (
                "ffplay", "-nodisp", "-loglevel", "quiet", "-autoexit", "-volume", str(volume), "-ss", str(start_sec),
                self.__path)
        # os.system('ffplay -nodisp -autoexit -volume {} -ss {} {}'.format(volume, start_sec, self.__path))

        self.__preempted = False
        self.__playing_instance = subprocess.Popen(args, stdout=subprocess.PIPE)

    def stop(self):
        if self.__playing_instance is not None:
            self.__preempted = True
            self.__playing_instance.kill()  # .terminate()

    def is_playing(self):
        return self.__playing_instance is not None and self.__playing_instance.poll() is None

    def wait_end(self):
        while (not rospy.is_shutdown() and
               self.__playing_instance is not None and self.__playing_instance.poll() is None):
            rospy.sleep(0.1)

    def __get_sound_duration(self):
        args = ("ffprobe", "-loglevel", "quiet", "-show_entries", "format=duration", "-i", self.__path)
        popen = subprocess.Popen(args, stdout=subprocess.PIPE)
        popen.wait()
        output = popen.stdout.read()
        return float(output.replace("[FORMAT]\nduration=", "").replace("\n[/FORMAT]\n", ""))
