# Lib
import rospy
import subprocess
from threading import Event, Thread, Lock
from queue import Queue, Empty

# Messages
from std_msgs.msg import String

# Services
from niryo_robot_msgs.srv import Trigger

# Command Status
from niryo_robot_msgs.msg import CommandStatus

from sound_volume import VolumeManager


class SoundExecution:

    def __init__(self, sound, start_time_sec=0, end_time_sec=0):
        self.sound = sound
        self.name = sound.name

        self.start_time_sec = start_time_sec
        self.duration = 0 if (end_time_sec == 0 or
                              end_time_sec > sound.duration or
                              start_time_sec > end_time_sec) else end_time_sec - start_time_sec

        self.__process = None

        self.__lock = Lock()
        self.event = Event()
        self.event.clear()

        self.result = CommandStatus.SUCCESS
        self.message = "Sound {} not played".format(self.name)

    def wait(self):
        play_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self.event.wait(0.25):
            # self.__process.poll() is None ==> finished
            if self.__process.poll() is None:
                break
            elif (rospy.Time.now() - play_time).to_sec() > self.duration * 1.5:
                self.message = "Sound {} timeout".format(self.name)
                self.result = CommandStatus.SOUND_TIMEOUT
                return self.result, self.message

        if self.result != CommandStatus.PREEMPTED:
            self.message = "Sound {} played".format(self.name)
            self.result = CommandStatus.SUCCESS

        return self.result, self.message

    # - Main class usage
    def play(self, volume):
        with self.__lock:
            if self.event.is_set():
                return self

            try:
                sound_path = self.sound.path
            except AttributeError:
                self.result = CommandStatus.FAILURE
                self.message = "Path attribute error on sound {}".format(self.name)
                self.event.set()
                return self

            if self.duration > 0:
                args = (
                    "ffplay", "-nodisp", "-loglevel", "quiet", "-autoexit", "-volume", str(volume), "-ss",
                    str(self.start_time_sec), "-t", str(self.duration), sound_path)
            else:
                args = (
                    "ffplay", "-nodisp", "-loglevel", "quiet", "-autoexit", "-volume", str(volume), "-ss",
                    str(self.start_time_sec), sound_path)
            # os.system('ffplay -nodisp -autoexit -volume {} -ss {} {}'.format(volume, start_sec, self.__path))
            self.__process = subprocess.Popen(args, stdout=subprocess.PIPE)
        return self

    def stop(self):
        with self.__lock:
            self.result = CommandStatus.PREEMPTED
            self.message = "Sound {} stopped".format(self.name)

            if self.__process is not None:
                try:
                    self.__process.kill()  # .terminate()
                except OSError:
                    pass

            self.event.set()

    def is_busy(self):
        if self.__process is None:
            return False
        else:
            return self.__process.poll() is None


class SoundPlayer:
    """
    Object which configure the sound management on the robot
    """

    def __init__(self):
        # - Init
        self.__volume_manager = VolumeManager()

        self.__playing_instance = None
        self.__execution_queue = Queue()
        self.__lock = Lock()

        self.loop_state = False
        self.__loop = Thread(target=self.playing_loop)

        # - Publisher
        self.__sound_publisher = rospy.Publisher('/niryo_robot_sound/sound', String, latch=True, queue_size=10)
        self.__sound_publisher.publish("")

        # - Services
        rospy.Service('/niryo_robot_sound/stop', Trigger, self.__callback_stop)

        rospy.on_shutdown(self.stop_loop)
        self.__loop.start()
        rospy.loginfo("Sound loop started")

    def __call__(self, *args, **kwargs):
        if len(args) < 3:
            return

        sound, start_time_sec, end_time_sec = args[:3]
        sound_execution = SoundExecution(sound, start_time_sec, end_time_sec)

        self.__execution_queue.put(sound_execution)
        self.stop_current()

        return sound_execution

    def __del__(self):
        self.stop_loop()

    def stop_loop(self):
        self.loop_state = False
        self.stop_all()

    def playing_loop(self):
        self.loop_state = True
        while not rospy.is_shutdown() and self.loop_state:
            try:
                sound_execution = self.__execution_queue.get(timeout=1)
            except Empty:
                continue

            self.__sound_publisher.publish(sound_execution.name)
            self.__playing_instance = sound_execution.play(volume=self.__volume_manager.volume)

            while not rospy.is_shutdown() and self.__playing_instance.is_busy():
                rospy.sleep(0.1)
                if not self.__execution_queue.empty():
                    sound_execution.stop()

            self.__sound_publisher.publish("")

            sound_execution.event.set()
            self.__execution_queue.task_done()

    def overlay_sound(self, sound):
        return SoundExecution(sound).play(volume=self.__volume_manager.volume)

    # - Callbacks
    def __callback_stop(self, _msg):
        if self.__playing_instance is None or not self.__playing_instance.is_busy():
            return CommandStatus.SUCCESS, "No sound to be stopped"

        sound_name = self.stop_all()
        return CommandStatus.SUCCESS, "{} sound stopped".format(sound_name)

    def stop_current(self, fade_out=False):

        if self.__playing_instance:
            if fade_out:
                old_volume = self.__volume_manager.raw_volume
                self.__volume_manager.fade_out()
                self.__playing_instance.stop()
                self.__volume_manager.set_volume(old_volume)
            else:
                self.__playing_instance.stop()

    def stop_all(self):
        last_sound_name = self.__playing_instance.name if self.__playing_instance else ""
        while not self.__execution_queue.empty():
            sound_execution = self.__execution_queue.get()
            sound_execution.stop()
            self.__execution_queue.task_done()
            last_sound_name = sound_execution.name
        self.stop_current()
        return last_sound_name
