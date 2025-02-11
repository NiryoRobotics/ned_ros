import math
import re
from queue import Queue, Empty
from threading import Thread
from typing import Dict, Generator

import rospy
from niryo_robot_led_ring.msg import LedRingAnimation
from niryo_robot_python_ros_wrapper import NiryoRosWrapper

use_sound = True

# Dictionary which replace some words by their phonetic equivalent to improve the TTS pronunciation
# The key is a word (can be a regex) to replace and the value is the replacement
_accents_dictionary = {r'free-? ?motion': 'freemoshion'}

_ned2_offsets = (
    (0.14, 0.14),
    (0.26, 0.07),
    (0.05, 0.07),
    (0.07, 0.07),
    (0.09, 0.09),
    (0.02, 0.02),
)


def say(robot: NiryoRosWrapper, text: str):
    for key, value in _accents_dictionary.items():
        text = re.sub(key, value, text)

    if not use_sound:
        rospy.loginfo(text)
        return
    robot.sound.say(text, 1)


def parse_duration(value: str) -> rospy.Duration:
    """
    Parse a string into a rospy.Duration object.
    Format: {number}{unit}, e.g., '10s', '5m', '2h', '1d'.
    Supported units:
      - s: seconds
      - m: minutes
      - h: hours
      - d: days
    """
    try:
        units = {'s': 1, 'm': 60, 'h': 3600, 'd': 86400}  # Seconds per unit
        return rospy.Duration(sum(units[unit] * int(num) for num, unit in re.findall(r'(\d+)([smhd])', value)))
    except (ValueError, IndexError):
        raise TypeError(
            f"Invalid duration format: '{value}'. Expected format: '{{number}}{{unit}}', e.g., '10s', '5m', '2h', '1d'."
        )


def get_joints_limits(robot: NiryoRosWrapper) -> Dict[int, Dict[str, float]]:
    """
    Get the joints limits of the robot. The limits are reduced to avoid hitting the mechanical limits
    :return: The limits with the following format: {joint_number: {'min': min_value, 'max': max_value}}
    """
    if robot.get_hardware_version() == 'ned2':
        offsets = _ned2_offsets
    else:
        offsets = ((0, 0), ) * len(robot.get_joints())

    return {
        int(joint_name[-1]): {
            'min': joints_limits['min'] + offset_min, 'max': joints_limits['max'] - offset_max
        }
        for (joint_name,
             joints_limits), (offset_min,
                              offset_max) in zip(robot.get_axis_limits()[1]['joint_limits'].items(), offsets)
    }


def __iterate_with_led_ring(robot: NiryoRosWrapper, calculate_progress):
    """
    Generalized function to update the LED ring according to an iterator.
    :param robot: The robot instance
    :param calculate_progress: A function that takes the current iteration index and returns the progress value.
           End the iteration by returning None.
    """
    n_leds = 30
    blank = [0, 0, 0]
    if robot.led_ring.animation_mode == LedRingAnimation.CUSTOM:
        colors = [color for color in robot.led_ring.get_led_colors() if color != blank]
        color = colors[0] if colors else [255, 255, 255]
    else:
        color = robot.led_ring.color
    ix = 0

    def update_led_ring(queue: Queue):
        """
        Update the led ring according to the latest element pushed in the queue.
        Run until None is pushed in the queue.
        :param queue: The queue containing the number of leds to light up
        :return:
        """
        n_leds_to_light = None
        while True:
            try:
                n_leds_to_light = queue.get(timeout=1)
            except Empty:
                pass

            if n_leds_to_light is None:
                break

            if robot.led_ring.is_autonomous:
                robot.led_ring.custom([color] * n_leds_to_light + [blank] * (n_leds - n_leds_to_light))

    leds_queue = Queue()
    led_ring_thread = Thread(target=update_led_ring, args=(leds_queue, ), daemon=True)
    led_ring_thread.start()

    while not rospy.is_shutdown():
        progress = calculate_progress(ix)
        if progress is None:  # End of iteration
            break

        leds_queue.put(math.ceil(n_leds * progress))

        yield ix
        ix += 1

    leds_queue.put(None)
    led_ring_thread.join()


def led_ring_range(robot: NiryoRosWrapper, *args, **kwargs) -> Generator[int, None, None]:
    """
    Act like a range but light up the led ring with a color that changes from start to stop
    :param robot: The robot instance
    :param args: The same arguments as the built-in range function
    :param kwargs: The same arguments as the built-in range function
    :return: A generator that yields the current iteration index
    """
    n_iter = len(range(*args, **kwargs))

    def calculate_progress(ix):
        if ix < n_iter:
            return (ix + 1) / n_iter

    return __iterate_with_led_ring(robot, calculate_progress)


def led_ring_duration(robot: NiryoRosWrapper, duration: rospy.Duration):
    """
    Light up the LED ring gradually over the given duration.
    :param robot: The robot instance
    :param duration: The duration over which the LED ring will be lit up
    :return: A generator that yields the current iteration index
    """
    start_time = rospy.Time.now()

    def calculate_progress(_ix):
        if (elapsed_time := rospy.Time.now() - start_time) < duration:
            return elapsed_time.to_sec() / duration.to_sec()

    return __iterate_with_led_ring(robot, calculate_progress)
