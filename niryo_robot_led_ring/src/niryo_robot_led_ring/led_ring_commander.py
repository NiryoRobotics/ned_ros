# Lib
import threading
import time

import rosnode
import rospy
from niryo_robot_led_ring.msg import LedRingStatus, LedRingAnimation
# Services
from niryo_robot_led_ring.srv import LedUser, LedUserRequest, SetLedColor, GetLedColors, GetLedColorsResponse
# Command Status
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_rpi.msg import HotspotButtonStatus
# Message
from niryo_robot_user_interface.msg import ConnectionState
from niryo_robot_status.msg import RobotStatus
from std_msgs.msg import Int32

from niryo_robot_led_ring.led_ring_animations import LedRingAnimations, get_rgba_color_from_list
from niryo_robot_led_ring.led_ring_enums import *


class LedRingCommander(object):
    """
    This class is in charge of the Led Ring Node
    Its main goal is to interpret commands, and use methods from the LedRingCommander class according to this command
    """

    def __init__(self):
        rospy.loginfo("Led Ring Commander - Initialisation")
        self.__is_shutdown = False
        self.__is_simulation = rospy.get_param("~simulation_mode")

        self.led_ring_anim = LedRingAnimations()
        self.robot_status = RobotStatus.BOOTING
        self.robot_status_str = ""
        self.logs_status = RobotStatus.NONE
        self.robot_out_of_bounds = False
        self.rpi_overheating = False

        self.user_animation_lock = threading.Lock()
        self.led_ring_animation_thread = threading.Thread()

        self.error_animation_lock = threading.Lock()
        self.led_ring_state = LedRingStatus()

        self.user_mode = False  # True if controlled by user, False if displaying robot state (priority)
        self.running_status_command = None

        self.dict_led_ring_methods = {
            LedRingAnimation.NONE: self.none_animation,
            LedRingAnimation.SOLID: self.solid_animation,
            LedRingAnimation.ALTERNATE: self.alternate_animation,
            LedRingAnimation.CHASE: self.chase_animation,
            LedRingAnimation.FLASHING: self.flashing_animation,
            LedRingAnimation.COLOR_WIPE: self.wipe_animation,
            LedRingAnimation.RAINBOW: self.rainbow_animation,
            LedRingAnimation.RAINBOW_CYLE: self.rainbow_cycle_animation,
            LedRingAnimation.RAINBOW_CHASE: self.rainbow_chase_animation,
            LedRingAnimation.GO_UP: self.go_up_animation,
            LedRingAnimation.GO_UP_AND_DOWN: self.go_up_and_down_animation,
            LedRingAnimation.BREATH: self.breath_animation,
            LedRingAnimation.SNAKE: self.snake_animation,
            LedRingAnimation.CUSTOM: self.custom_animation,
        }

        # - Publishers
        self.__led_ring_status_pub = rospy.Publisher('~led_ring_status', LedRingStatus, latch=True, queue_size=10)
        rospy.sleep(1)
        self._publish_led_ring_status()  # publish the status at the beginning

        rospy.on_shutdown(self.shutdown)
        self.display_user_mode()  # Turn on the leds with the right animation

        # Define this object as an observer of led_ring_anim, so when an observable
        # variable change, this node is informed.
        # This is usefull for the current animation and the current color, which are
        #  managed by the led ring commander class.
        self.led_ring_anim.register_observer(self)

        # - Services
        self.__set_ring_led_user = rospy.Service('~set_user_animation', LedUser, self.__callback_set_led_ring_user)
        self.__set_led_color_service = rospy.Service('~set_led_color', SetLedColor, self.__callback_set_led_color)
        rospy.Service('~get_led_colors', GetLedColors, self.__callback_get_led_colors)

        # - Subscribers
        self.robot_status_subscriber = rospy.Subscriber('/niryo_robot_status/robot_status',
                                                        RobotStatus,
                                                        self.__callback_robot_status,
                                                        queue_size=1)
        rospy.Subscriber("/niryo_robot/blockly/save_current_point", Int32, self.__callback_save_current_point)
        rospy.Subscriber('/niryo_robot_user_interface/niryo_studio_connection',
                         ConnectionState,
                         self.__callback_niryo_studio)
        rospy.Subscriber('/niryo_robot/hotspot_button_state', HotspotButtonStatus, self.__callback_hotspot_button_state)

        self.__check_shutdown_timer = rospy.Timer(rospy.Duration(2), self.shutdown_check)

        rospy.loginfo("Led Ring Commander - Started")

    def shutdown(self, color=None):
        if not self.__is_shutdown:
            self.__is_shutdown = True
            self.robot_status_subscriber.unregister()
            self.stop_led_ring_thread()
            self.user_animation_lock.acquire()

            command = LedUserRequest()
            command.colors = [WHITE]
            command.iterations = 1
            command.period = 2
            self.breath_animation(command)

            self.blink(WHITE, 8, 0.5)
            self.error_animation_lock.acquire()

            if color is None:
                self.none_animation(None)
            else:
                self.led_ring_anim.solid(color)
                time.sleep(0.5)
            # self.__is_shutdown = True
            rospy.signal_shutdown("shutdown")

    def shutdown_check(self, *_):
        try:
            if not rospy.is_shutdown() and not self.__is_shutdown:
                rosnode.get_node_names()
        except rosnode.ROSNodeIOException:
            self.shutdown()

    @property
    def is_shutdown(self):
        return self.__is_shutdown

    # - Callbacks
    def __callback_robot_status(self, msg):
        """
        Callback method to set led ring state according to current robot status.
        This callback takes precedence over the user's led ring control method, if the
        robot status is not RUNNING_AUTONOMOUS and logs status is not NONE.
        It is triggered only when the robot status changed.
        """
        if msg.robot_status == RobotStatus.SHUTDOWN:
            self.shutdown()
        elif msg.robot_status in [RobotStatus.REBOOT, RobotStatus.UPDATE]:
            self.shutdown(WHITE)
        elif self.robot_status == RobotStatus.BOOTING != msg.robot_status:
            rospy.sleep(3.5)  # because no fade
            if not self.__is_simulation:
                from niryo_robot_led_ring.led_ring_utils import enable_led_ring
                enable_led_ring(rospy.get_param('~enable_led_ring_bcm_pin'))
                self.led_ring_anim.init_led_ring()
            #    self.led_ring_anim.fade(BLUE)

        new_robot_status = self.robot_status != msg.robot_status
        new_robot_out_of_bound = self.robot_out_of_bounds != msg.out_of_bounds
        new_rpi_overheating = self.rpi_overheating != msg.rpi_overheating

        if new_robot_status or new_robot_out_of_bound or new_rpi_overheating:

            if new_robot_status and msg.robot_status in [RobotStatus.CALIBRATION_IN_PROGRESS, RobotStatus.REBOOT_MOTOR]:
                rospy.sleep(0.5)  # for synchro
                self.blink(YELLOW, 3, 1)

            self.robot_status = msg.robot_status
            self.robot_status_str = msg.robot_status_str
            self.logs_status = msg.logs_status
            self.robot_out_of_bounds = msg.out_of_bounds
            self.rpi_overheating = msg.rpi_overheating

            self.set_user_mode(
                self.robot_status in [RobotStatus.RUNNING_AUTONOMOUS, RobotStatus.LEARNING_MODE_AUTONOMOUS])
            self.display_user_mode()

    def __callback_set_led_ring_user(self, req):
        """
        Service to allow the user to control the Led Ring. A new request will interrupt the previous one, if still
        playing.
        """
        if not self.user_mode:
            # if the status of the robot is not RUNNING_AUTONOMOUS
            return CommandStatus.ABORTED, "Led Ring not in autonomous mode, user can't control it"

        animation = req.animation_mode.animation

        rospy.logdebug('Led ring - controled by user, animation n. {}'.format(animation))
        if animation in self.dict_led_ring_methods:
            self.start_user_thread(req)

            animation_name = ROBOT_ANIM_TO_STRING[animation]
            if req.wait_end and (req.iterations != 0 or animation == LedRingAnimation.COLOR_WIPE):
                interrupted = self.wait_until_finished()
                return self._return_after_waiting(animation_name, req.iterations, interrupted)
            else:
                return CommandStatus.SUCCESS, "Led Ring set by user: {} mode launched".format(animation_name)
        else:
            return CommandStatus.FAILURE, "Led Ring set by user: command request doesn't exist"

    def __callback_set_led_color(self, req):
        if not self.user_mode:
            # if the status of the robot is not RUNNING_AUTONOMOUS
            return CommandStatus.ABORTED, "Led Ring not in autonomous mode, user can't control it"

        if not (0 <= req.led_id < self.led_ring_anim.led_count):
            return CommandStatus.FAILURE, "Wrong led id, must be between 0 and {}".format(
                self.led_ring_anim.led_count - 1)

        self.led_ring_anim.set_led_color(req.led_id, req.color)

        return CommandStatus.SUCCESS, "Successfully set led {} at color {}".format(
            req.led_id, [req.color.r, req.color.g, req.color.b])

    def __callback_get_led_colors(self, _):
        led_ring_colors = map(get_rgba_color_from_list, self.led_ring_anim.get_state_list_from_pixel_strip())
        return GetLedColorsResponse(status=CommandStatus.SUCCESS,
                                    message='Successfully retrieved the leds colors',
                                    led_ring_colors=list(led_ring_colors))

    def __callback_save_current_point(self, _msg):
        if not self.user_mode:
            self.blink_over_status(WHITE, 1, 0.5)

    def __callback_niryo_studio(self, msg):
        if msg.state in [ConnectionState.connection, ConnectionState.close] and not self.user_mode:
            self.blink_over_status(PURPLE, 2, 0.5)

    def __callback_hotspot_button_state(self, msg):
        if msg.state == HotspotButtonStatus.RELEASED and msg.mode != HotspotButtonStatus.IGNORE_PRESS:
            self.blink(BLUE, 2, 0)
            self.display_user_mode()
        elif msg.mode in [HotspotButtonStatus.LONG_PRESS, HotspotButtonStatus.VERY_LONG_PRESS]:
            command = LedUserRequest()
            command.animation_mode.animation = LedRingAnimation.FLASHING
            command.colors = [PURPLE] if msg.mode == HotspotButtonStatus.VERY_LONG_PRESS else [WHITE]
            self.start_led_ring_thread(command)
        else:
            self.display_user_mode()

    def notify_current_anim_and_color(self, _observer):
        """
        called by the led_ring_anim when the current animation played or the color on the Led Ring is updated,
        used to publish Led STATUS
        """
        self._publish_led_ring_status()

    # - Threads
    def start_user_thread(self, command):
        self.stop_led_ring_thread()
        self.led_ring_animation_thread = threading.Thread(target=self.user_animation, args=[command])
        self.led_ring_animation_thread.start()

    def start_led_ring_thread(self, command):
        """
        Start a thread to allow the user to control the led.
        We use a thread to avoid the blocking effect of the Led ring
        control methods.
        """
        # print 'get lock'
        # with self.error_animation_lock:
        #    pass
        # print 'retrieve lock'
        self.stop_led_ring_thread()
        # print 'stopped thread'
        self.led_ring_animation_thread = threading.Thread(
            target=self.dict_led_ring_methods[command.animation_mode.animation], args=[command])
        self.running_status_command = command
        self.led_ring_animation_thread.start()
        # print 'thread started'

    def stop_led_ring_thread(self):
        try:
            if self.led_ring_anim.is_animation_running():
                self.led_ring_anim.stop_animation()

            while self.led_ring_animation_thread.join(timeout=0.1) is None and not rospy.is_shutdown():
                self.led_ring_anim.stop_animation()
                if not self.led_ring_animation_thread.is_alive():
                    return

        except (AttributeError, RuntimeError):
            pass

    def error_animation(self):
        self.blink_over_status(RED, 5, 2)

    def blink_over_status(self, color, iterations, period):
        self.blink(color, iterations, period)
        command = self.get_robot_status_led_ring_cmd()
        self.start_led_ring_thread(command)

    # - Publisher

    def _publish_led_ring_status(self):
        """
        Publish the current status of the Led ring, as a LedRingStatus message :
        led_mode: either controlled by user or displaying robot status
        animation_mode: which method is used to animate pixels (solid, flashing,...), as a LedRingAnimation message
        animation_color: current animation color, except for rainbow methods

        Function called only when one parameter changed.
        """
        # check if led mode or led animations changed

        led_ring_state = LedRingStatus()
        led_ring_state.led_mode = LedRingStatus.USER if self.user_mode else LedRingStatus.ROBOT_STATUS
        led_ring_state.animation_mode = LedRingAnimation()
        led_ring_state.animation_mode.animation = self.led_ring_anim.current_animation
        led_ring_state.animation_color = self.led_ring_anim.current_animation_color

        if led_ring_state != self.led_ring_state:
            self.led_ring_state = led_ring_state
            try:
                self.__led_ring_status_pub.publish(led_ring_state)
            except rospy.ROSException:
                return

    # - Usefull methods
    def wait_until_finished(self):
        """
        Wait until all iterations of the Led Ring method are done. After that, the led ring is turned off.
        If function was interrupted, return True.
        """
        if self.led_ring_animation_thread.is_alive():
            self.led_ring_animation_thread.join()
        return self.led_ring_anim.was_function_interrupted()

    @staticmethod
    def _return_after_waiting(anim_name, iterations, interrupted):
        """
        Return a CommandStatus and a message, after the requested animation was completed
        """
        if not interrupted:
            if iterations is not None:
                return CommandStatus.SUCCESS, "Led Ring set by user: {} iterations done - ".format(
                    iterations) + anim_name + " mode"
            else:
                # iteration is 0 for color wipe and solid mode.
                return CommandStatus.SUCCESS, "Led Ring set by user: done - " + anim_name + " mode"
        else:
            return CommandStatus.STOPPED, "Led Ring set by user: interrupted - " + anim_name + " mode"

    def set_user_mode(self, boolean):
        if boolean != self.user_mode:
            self.user_mode = boolean
            self._publish_led_ring_status()

    def display_user_mode(self):
        if RobotStatus.BOOTING == self.robot_status:
            return

        command = self.get_robot_status_led_ring_cmd()
        if command == self.running_status_command or self.__is_shutdown:
            return
        elif self.user_animation_lock.locked() and self.robot_status in [
                RobotStatus.RUNNING_AUTONOMOUS, RobotStatus.LEARNING_MODE_AUTONOMOUS
        ]:
            return

        self.start_led_ring_thread(command)

    def get_robot_status_led_ring_cmd(self):
        # Prioritize Emergency Stop animation
        if self.robot_status in ROBOT_STATUS_TO_ANIM and self.robot_status in ROBOT_CRITICAL_STATUS:
            animation, colors = ROBOT_STATUS_TO_ANIM[self.robot_status]
        elif self.robot_out_of_bounds:
            animation, colors = ROBOT_STATUS_TO_ANIM["out_of_bound"]
        elif self.rpi_overheating:
            animation, colors = ROBOT_STATUS_TO_ANIM["overheating"]
        elif self.robot_status in ROBOT_STATUS_TO_ANIM:
            animation, colors = ROBOT_STATUS_TO_ANIM[self.robot_status]
        else:
            # If no method defined (no entry for this key in dictionnay), turn off led rings
            rospy.logdebug('Led ring - No Led control method defined for {} status and {} log'.format(
                self.robot_status, self.robot_status_str))
            animation, colors = ROBOT_STATUS_TO_ANIM[RobotStatus.UNKNOWN]

        command = LedUserRequest()
        command.animation_mode.animation = animation
        command.colors = colors if isinstance(colors, list) else [colors]
        return command

    # - Animations functions
    def user_animation(self, cmd):
        animation_id = cmd.animation_mode.animation
        with self.user_animation_lock:
            self.running_status_command = None
            self.dict_led_ring_methods[animation_id](cmd)

            # Pour certaines animation, on laisse la couleur,
            # pour les autre on affiche l'etat du robot a la fin de l'animation
            if not self.led_ring_anim.was_function_interrupted() and animation_id not in [
                    LedRingAnimation.NONE,
                    LedRingAnimation.COLOR_WIPE,
                    LedRingAnimation.SOLID,
                    LedRingAnimation.CUSTOM,
            ]:
                robot_status_command = self.get_robot_status_led_ring_cmd()
                self.running_status_command = robot_status_command
                self.dict_led_ring_methods[robot_status_command.animation_mode.animation](robot_status_command)

    def none_animation(self, _cmd):
        try:
            self.led_ring_anim.none()
        except AttributeError:
            pass

    def solid_animation(self, cmd):
        self.led_ring_anim.solid(cmd.colors[0])

    def alternate_animation(self, cmd):
        self.led_ring_anim.alternate(cmd.colors, cmd.period, cmd.iterations)

    def chase_animation(self, cmd):
        self.led_ring_anim.chase(cmd.colors[0], cmd.period, cmd.iterations)

    def flashing_animation(self, cmd):
        self.led_ring_anim.flashing(cmd.colors[0], cmd.period, cmd.iterations)

    def wipe_animation(self, cmd):
        self.led_ring_anim.color_wipe(cmd.colors[0], cmd.period)

    def rainbow_animation(self, cmd):
        self.led_ring_anim.rainbow(cmd.period, cmd.iterations)

    def rainbow_cycle_animation(self, cmd):
        self.led_ring_anim.rainbow_cycle(cmd.period, cmd.iterations)

    def rainbow_chase_animation(self, cmd):
        self.led_ring_anim.rainbow_chase(cmd.period, cmd.iterations)

    def go_up_animation(self, cmd):
        self.led_ring_anim.go_up(cmd.colors[0], cmd.period, cmd.iterations)

    def go_up_and_down_animation(self, cmd):
        self.led_ring_anim.go_up_and_down(cmd.colors[0], cmd.period, cmd.iterations)

    def breath_animation(self, cmd):
        self.led_ring_anim.breath(cmd.colors[0], cmd.period, cmd.iterations)

    def snake_animation(self, cmd):
        self.led_ring_anim.snake(cmd.colors[0], cmd.period, cmd.iterations)

    def custom_animation(self, cmd):
        self.led_ring_anim.custom(cmd.colors)

    def blink(self, color, iterations, period):
        with self.error_animation_lock:

            command = LedUserRequest()
            command.animation_mode.animation = LedRingAnimation.FLASHING
            command.colors = [color]
            command.iterations = iterations
            command.period = period
            self.start_led_ring_thread(command)
            try:
                if self.led_ring_animation_thread.is_alive():
                    self.led_ring_animation_thread.join()
            except RuntimeError:
                pass
