import rospy
import math
from threading import Lock

import rpi_ws281x
from rpi_ws281x import Color, PixelStrip, ws
from niryo_robot_led_ring.msg import LedRingAnimation
from std_msgs.msg import ColorRGBA

from niryo_robot_led_ring.led_ring_simulation import LedRingSimulation

GREY = ColorRGBA(51, 51, 51, 0)  # used as led ring turned off in rviz
BLACK = ColorRGBA(0, 0, 0, 0)


def version_to_tab(version_str):
    return list(map(int, version_str.split('.')))


class LedRingAnimations:
    """
    Object which implements control method for the Led ring
    """
    def __init__(self):

        # - Parameters
        if version_to_tab(rpi_ws281x.__version__) < version_to_tab('4.3.4'):
            rospy.logerr(
                'Please install a more recent version of the rpi_ws281x (min 4.3.4) library, '
                'otherwise your led ring may not work.')

        self.__is_simulation = rospy.get_param("~simulation_mode")

        # LED self.strip configuration:
        self.LED_COUNT = rospy.get_param('~led_count')  # Number of LED pixels.
        self.LED_PIN = rospy.get_param(
            '~led_pin')  # GPIO pin connected to the pixels (must support PWM!)
        self.LED_FREQ_HZ = rospy.get_param(
            '~led_freq_hs')  # LED signal frequency in hertz (usually 800khz)
        self.LED_DMA = rospy.get_param(
            '~led_dma')  # DMA channel to use for generating signal (try 10)
        # Set to 0 for darkest and 255 for brightest
        self.LED_BRIGHTNESS = rospy.get_param('~led_brightness')
        # True to invert the signal (when using NPN transistor level shift)
        self.LED_INVERT = rospy.get_param('~led_invert')
        self.LED_CHANNEL = rospy.get_param('~led_channel')
        self.__led_offset = rospy.get_param("~led_offset")

        if not self.__is_simulation:
            self.LED_STRIP = ws.WS2811_STRIP_GRB

        # default param for led ring control methods
        self.default_flashing_period = rospy.get_param(
            '~default_flashing_period')
        self.default_alternate_period = rospy.get_param(
            '~default_alternate_period')
        self.default_chase_period = rospy.get_param('~default_chase_period')
        self.default_colorwipe_period = rospy.get_param(
            '~default_colorwipe_period')
        self.default_rainbow_period = rospy.get_param(
            '~default_rainbow_period')
        self.default_rainbowcycle_period = rospy.get_param(
            '~default_rainbowcycle_period')
        self.default_rainbowchase_period = rospy.get_param(
            '~default_rainbowchase_period')
        self.default_goup_period = rospy.get_param('~default_goup_period')
        self.default_goupanddown_period = rospy.get_param(
            '~default_goupanddown_period')
        self.default_breath_period = rospy.get_param('~default_breath_period')
        self.default_snake_period = rospy.get_param('~default_snake_period')

        self.__stop_func = False
        self.__animation_lock = Lock()

        self._observers = []

        self.current_animation_color = BLACK
        self.current_animation = LedRingAnimation.NONE
        self.set_current_anim_and_color(
            self.current_animation,
            self.current_animation_color,
        )

        self.strip = None
        self.led_count = self.LED_COUNT
        self.off_color = GREY if self.__is_simulation else BLACK

        # for real mode
        # list used to store the current state of the real led ring, as a list of ColorRGBA objects
        self.current_real_led_ring_state = []

        self.led_ring_makers = LedRingSimulation(self.led_count)

        self.blackout()

    # def __del__(self):
    #     pass
    #     #self.set_and_show_leds(self.off_color)
    #     if self.strip is not None:
    #         del self.strip

    def init_led_ring(self):
        if not self.__is_simulation:
            self.strip = PixelStrip(self.LED_COUNT,
                                    self.LED_PIN,
                                    self.LED_FREQ_HZ,
                                    self.LED_DMA,
                                    self.LED_INVERT,
                                    self.LED_BRIGHTNESS,
                                    self.LED_CHANNEL,
                                    self.LED_STRIP)
            self.strip.begin()
            self.led_count = self.strip.numPixels()

    def stop_animation(self):
        """
        Stop iteration (and endless iteration) for functions that perform continous action like alternate_color, and
        wait until the previous function is finished.
        Used when launched in robot status display mode. Indeed, when used by the user,
        the previous function is stopped before, in the start thread function
        """
        if self.is_animation_running():
            self.__stop_func = True
            with self.__animation_lock:  # wait the end of the running animation
                self.blackout()

    def init_animation(self):
        self.__stop_func = False

    def is_animation_running(self):
        return self.__animation_lock.locked()

    def was_function_interrupted(self):
        return self.__stop_func  # if true, function was interrupted

    def __play_cycle_animation(self,
                               color_cycle,
                               period,
                               iterations,
                               animation_function):
        # start playing animation :
        loop_period = period * 1.0 / len(color_cycle)
        next_loop_time = rospy.Time.now()

        count = 1 if not iterations else iterations
        while count > 0:
            for cycle_index in range(len(color_cycle)):
                if self.__stop_func:
                    break

                animation_function(color_cycle, cycle_index)
                next_loop_time += rospy.Duration(loop_period)
                self.__sleep_animation(next_loop_time)

            if self.__stop_func:
                break

            if iterations:
                count -= 1

        self.blackout()

    def none(self):
        """
        Turn off leds, in simu and in real, with the "solid" method
        """
        self.solid(self.off_color)

    def solid(self, color_rgba):
        """
        Sets all Leds to a color at once
        """
        self.init_animation()
        with self.__animation_lock:
            self.set_and_show_leds(color_rgba)

            mode = LedRingAnimation.NONE if color_rgba == self.off_color else LedRingAnimation.SOLID
            self.set_current_anim_and_color(mode, color_rgba)

    def custom(self, color_rgba):
        """
        Sets all Leds to a color at once
        """
        self.init_animation()
        with self.__animation_lock:
            colors = color_rgba[:self.led_count] if len(
                color_rgba) > self.led_count else color_rgba + (
                    len(color_rgba) - self.led_count) * [BLACK]

            for led_id, led_color in enumerate(colors):
                # set color led by led
                self.set_led(led_id, led_color)
            self.show_leds()  # display all leds

            self.set_current_anim_and_color(LedRingAnimation.CUSTOM)

    def set_led_color(self, led_id, color_rgba):
        self.init_animation()
        with self.__animation_lock:
            self.set_led(led_id, color_rgba)
        self.show_leds()
        self.set_current_anim_and_color(LedRingAnimation.CUSTOM)

    def flashing(self, color_rgba, period=None, iterations=0):
        """
        Flash a color according to a frequency
        """
        def animation_function(anim_color_cycle, anim_cycle_index):
            self.set_and_show_leds(anim_color_cycle[anim_cycle_index])

        self.init_animation()

        # set default frequency
        if not period:
            period = self.default_flashing_period

        # configure the animation
        color_cycle = [color_rgba, self.off_color]

        # start the animation
        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.FLASHING,
                                            color_rgba)
            self.__play_cycle_animation(color_cycle,
                                        period,
                                        iterations,
                                        animation_function)

    def alternate(self, color_list_rgba, period=None, iterations=0):
        """
        The different colors are alternated one after the other.
        If iterations is 0, do it indefinitely
        """
        def animation_function(anim_color_cycle, anim_cycle_index):
            self.set_current_anim_and_color(LedRingAnimation.ALTERNATE,
                                            anim_color_cycle[anim_cycle_index])
            self.set_and_show_leds(anim_color_cycle[anim_cycle_index])

        self.init_animation()

        # configure the animation
        if not period:
            period = self.default_alternate_period
        color_cycle = color_list_rgba[:]

        # start the animation
        with self.__animation_lock:
            self.__play_cycle_animation(color_cycle,
                                        period,
                                        iterations,
                                        animation_function)

    def chase(self, color_rgba, period=None, iterations=0):
        """
        Movie theater light style chaser animation.
        If iterations is 0, do it indefinitely
        """
        def animation_function(anim_color_cycle, anim_cycle_index):
            for led_id in range(self.led_count):
                # set color led by led
                self.set_led(
                    led_id,
                    anim_color_cycle[led_id % len(anim_color_cycle) -
                                     anim_cycle_index])
            self.show_leds()  # display all leds

        self.init_animation()
        # configure the animation
        if not period:
            period = self.default_chase_period
        color_cycle = [self.off_color, self.off_color, color_rgba]

        # start the animation
        with self.__animation_lock:
            self.set_and_show_leds(self.off_color)
            self.set_current_anim_and_color(LedRingAnimation.CHASE, color_rgba)
            self.__play_cycle_animation(color_cycle,
                                        period,
                                        iterations,
                                        animation_function)

    def color_wipe(self, color_rgba, duration=None):
        """
        Wipe color across, light a Led at a time.
        Similar to goUp, but Leds are not turned off at the end.
        """
        self.init_animation()

        if not duration:
            duration = self.default_colorwipe_period

        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.COLOR_WIPE,
                                            color_rgba)
            self.__wipe_animation(color_rgba, duration)

            if self.__stop_func:
                self.blackout()  # turn off leds

    def __wipe_animation(self, color_rgba, duration):
        next_loop = rospy.Time.now()
        period = rospy.Duration(duration * 1.0 / self.led_count)

        for led_id in range(self.led_count):
            if self.__stop_func:
                break
            self.set_led(led_id, color_rgba)
            self.show_leds()

            next_loop += period
            self.__sleep_animation(next_loop)

    def breath_animation(self, color_rgba, duration):
        next_loop = rospy.Time.now()

        gamma = 0.2  # affects the width of peak (more or less darkness)
        beta = 0.5  # shifts the gaussian to be symmetric

        nb_steps = 255
        anim_period = rospy.Duration(duration * 1.0 / nb_steps)

        self.current_animation_color = color_rgba
        for counter in range(nb_steps):
            if self.__stop_func:
                break

            # value = 255 * math.sin(2 * math.pi * (counter * 1.0 / smoothness_pts))
            factor = math.exp(-(pow(
                ((counter * 1.0 / nb_steps) - beta) / gamma, 2.0)) / 2.0)

            # self.set_brightness(value)
            self.set_and_show_leds(
                ColorRGBA(self.current_animation_color.r * factor,
                          self.current_animation_color.g * factor,
                          self.current_animation_color.b * factor,
                          0))

            next_loop += anim_period
            rospy.sleep(next_loop - rospy.Time.now())

    def breath(self, color_rgba, period=None, iterations=0):
        """
        Leds turn on like a loading circle, and are all turned off at the same time
        If iterations is 0, do it indefinitely
        """
        self.init_animation()
        if period == 0 or period is None:
            period = self.default_breath_period

        # start playing animation :
        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.BREATH,
                                            color_rgba)
            if not iterations:
                while not self.__stop_func:
                    self.breath_animation(color_rgba, period)
            else:
                for _i in range(iterations):
                    if self.__stop_func:
                        break
                    self.breath_animation(color_rgba, period)

            self.blackout()

    def snake(self, color_rgba, period=None, iterations=0):
        def animation_function(anim_color_cycle, anim_cycle_index):
            for led_id in range(self.led_count):
                # set color led by led
                self.set_led(
                    led_id,
                    anim_color_cycle[(led_id + anim_cycle_index) %
                                     len(anim_color_cycle)])
            self.show_leds()  # display all leds

        self.init_animation()
        # configure the animation
        if not period:
            period = self.default_snake_period

        attenuated_color = ColorRGBA(color_rgba.r * 0.5,
                                     color_rgba.g * 0.5,
                                     color_rgba.b * 0.5,
                                     0)
        snake_length = 10
        snake_pattern = [
            attenuated_color
        ] + (snake_length - 2) * [color_rgba] + [
            attenuated_color
        ] + (self.LED_COUNT - snake_length) * [self.off_color]

        # start the animation
        with self.__animation_lock:
            self.set_and_show_leds(self.off_color)
            self.set_current_anim_and_color(LedRingAnimation.SNAKE, color_rgba)
            self.__play_cycle_animation(snake_pattern,
                                        period,
                                        iterations,
                                        animation_function)

    def go_up(self, color_rgba, period=None, iterations=0):
        """
        Leds turn on like a loading circle, and are all turned off at the same time
        If iterations is 0, do it indefinitely
        """
        def animation(duration):
            end_time = rospy.Time.now() + rospy.Duration(duration)

            self.__wipe_animation(
                color_rgba, duration * self.led_count / (self.led_count + 1))
            self.set_and_show_leds(self.off_color)
            rospy.sleep(end_time - rospy.Time.now())

        self.init_animation()
        if period == 0 or period is None:
            period = self.default_goup_period

        # start playing animation :
        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.GO_UP, color_rgba)
            if not iterations:
                while not self.__stop_func:
                    animation(period)
            else:
                for _ in range(iterations):
                    if self.__stop_func:
                        break
                    animation(period)

            self.blackout()

    def go_up_and_down(self, color_rgba, period=None, iterations=0):
        """
        Leds turn on like a loading circle, and turn off the same way
        If iterations is 0, do it indefinitely
        """
        def animation(duration):
            self.__wipe_animation(color_rgba, duration / 2.0)
            self.__wipe_animation(self.off_color, duration / 2.0)

        self.init_animation()
        if period == 0 or period is None:
            period = self.default_goupanddown_period

        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.GO_UP_AND_DOWN,
                                            color_rgba)

            # start playing animation :
            if not iterations:
                while not self.__stop_func:
                    animation(period)
            else:
                for _ in range(iterations):
                    if self.__stop_func:
                        break
                    animation(period)

            self.blackout()

    def __rainbow_animation(self, duration, animation_function):
        next_loop = rospy.Time.now()

        anim_period = rospy.Duration(duration / 256.0)
        for color_counter in range(256):
            if self.__stop_func:
                break

            animation_function(color_counter)

            next_loop += anim_period
            rospy.sleep(next_loop - rospy.Time.now())

    def rainbow(self, period=None, iterations=0):
        """
        Draw rainbow that fades across all Leds at once
        If iterations is 0, do it indefinitely
        """
        def animation(color_counter):
            # for led_id in range(self.led_count):
            #   self.set_led(led_id, wheel_rgba((led_id + color_counter) & 255))
            # self.show_leds()
            self.set_and_show_leds(wheel_rgba(color_counter & 255))

        self.init_animation()
        # configure the animation
        if not period:
            period = self.default_rainbow_period

        with self.__animation_lock:
            # no color info in led_ring_status topic, so we just notify that the animation changed
            self.set_current_anim_and_color(LedRingAnimation.RAINBOW)

            # start playing animation :

            if not iterations:
                while not self.__stop_func:
                    self.__rainbow_animation(period, animation)
            else:
                for _ in range(iterations):
                    if self.__stop_func:
                        break
                    self.__rainbow_animation(period, animation)

            self.blackout()  # turn off leds after all iterations

    def rainbow_cycle(self, period=None, iterations=0):
        """
        Draw rainbow that uniformly distributes itself across all Leds
        If iterations is 0, do it indefinitely
        """
        def animation(color_counter):
            for led_id in range(self.led_count):
                pos = int(
                    (led_id * 256.0 // self.led_count + color_counter)) & 255
                self.set_led(
                    led_id,
                    wheel_rgba(pos),
                )
            self.show_leds()

        self.init_animation()
        if not period:
            period = self.default_rainbowcycle_period

        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.RAINBOW_CYLE)

            # start playing animation :
            if not iterations:
                while not self.__stop_func:
                    self.__rainbow_animation(period, animation)
            else:
                for _ in range(iterations):
                    if self.__stop_func:
                        break
                    self.__rainbow_animation(period, animation)

            self.blackout()  # turn off leds after all iterations

    def rainbow_chase(self, period=None, iterations=0):
        """
        Rainbow chase animation
        If iterations is 0, do it indefinitely
        """
        def animation(color_counter):
            for led_id in range(self.led_count):
                offset = color_counter % 3

                if (led_id + offset) % 3 == 0:
                    self.set_led(led_id,
                                 wheel_rgba((led_id + color_counter) & 255))
                else:
                    self.set_led(led_id, self.off_color)  # don't show
            self.show_leds()

        self.init_animation()
        if not period:
            period = self.default_rainbowchase_period

        with self.__animation_lock:
            self.set_current_anim_and_color(LedRingAnimation.RAINBOW_CHASE)
            self.set_and_show_leds(self.off_color)

            # start playing animation :
            if not iterations:
                while not self.__stop_func:
                    self.__rainbow_animation(period, animation)
            else:
                for _ in range(iterations):
                    if self.__stop_func:
                        break
                    self.__rainbow_animation(period, animation)

            self.blackout()  # turn off leds after all iterations

    def blackout(self):
        """
        Black out every led, without stopping previous function
        """
        self.set_and_show_leds(self.off_color)
        self.set_current_anim_and_color(LedRingAnimation.NONE, BLACK)

    def fade(self, color_rgba, duration=3.5, steps=100):
        current_color = self.current_animation_color
        step_r = (color_rgba.r - current_color.r) / float(steps)
        step_g = (color_rgba.g - current_color.g) / float(steps)
        step_b = (color_rgba.b - current_color.b) / float(steps)

        sleep_duration = duration / float(steps)
        for _ in range(steps):
            self.current_animation_color = ColorRGBA(
                self.current_animation_color.r + step_r,
                self.current_animation_color.g + step_g,
                self.current_animation_color.b + step_b,
                0)
            rospy.sleep(sleep_duration)

    # - Real Led ring related method

    def get_state_list_from_pixel_strip(self):
        """
        Return a list of size self.led_count, containing the current rgb color [r, g, b] of each Led in the
        real led ring
        """
        if self.strip is None:
            return []

        real_leds_state = []
        for i in range(self.led_count):
            # read back the state from the library memory buffer
            # (can't read directly Led state, they are Write-only)
            color_i = self.strip.getPixelColorRGB(i)
            real_leds_state.append([color_i.r, color_i.g, color_i.b])
        return real_leds_state

    # - Generic methods usable for simulation and real robot

    def set_led(self, index, color_rgba):
        """
        Set the color of a pixel, in simu or in real
        """
        led_index = (index + self.__led_offset) % self.led_count
        if self.strip is not None:
            led_color = get_24bits_color_from_msg(color_rgba)
            self.strip.setPixelColor(led_index, led_color)
        self.led_ring_makers.set_one_led_marker(led_index, color_rgba)

    def show_leds(self):
        """
        Display all Led's values previously set, in simu or in real
        """
        if self.strip is not None:
            self.strip.show()

            # update value of current led state
            current_rgb_array_led_state = self.get_state_list_from_pixel_strip(
            )
            current_color_rgba_led_state = []
            # transform a list like [[r, g, b], [r, g, b], ...] to a list like [ColorRGBA, ColorRGBA, ...]
            for elem in current_rgb_array_led_state:
                current_color_rgba_led_state.append(
                    get_rgba_color_from_list(elem))
            self.set_current_real_leds_state(current_color_rgba_led_state)
        self.led_ring_makers.show_led_ring_markers()

    def set_and_show_leds(self, led_color, range_=None, index_delta=0):
        """
        Iterate over all leds, set them to the chosen color and show them all at once.
        "range_" must be filled only if we want to iterate over a part of the led ring.
        "index_delta" is used by "chase" methods only.
        """

        if range_ is None:
            range_ = self.led_count
            for i in range(range_):
                self.set_led(i + index_delta, led_color)
            self.show_leds()

        elif isinstance(range_, list) and len(range_) == 3:
            for i in range(range_[0], range_[1], range_[2]):
                self.set_led(i + index_delta, led_color)
            self.show_leds()

    def set_brightness(self, brightness):
        if self.strip is not None:
            self.strip.setBrightness(brightness)

    def __sleep_animation(self, until_time):
        while (until_time - rospy.Time.now()).to_sec() > 0.1:
            rospy.sleep(0.1)
            if self.__stop_func:
                break
        else:
            rospy.sleep(until_time - rospy.Time.now())

    # Observable related methods
    def set_current_anim_and_color(self, animation, anim_color=None):
        self.current_animation_color = anim_color if anim_color is not None else BLACK
        self.current_animation = animation
        self.notify_observers()

    def set_current_real_leds_state(self, rgb_list):
        self.current_real_led_ring_state = rgb_list

    def notify_observers(self):
        """
        trigger a function in the observer (led ring node) class
        """
        for obs in self._observers:
            obs.notify_current_anim_and_color(self)

    def register_observer(self, observer):
        """
        Used to add the led ring node as an observer of this class
        """
        self._observers.append(observer)


def get_rgba_color_from_list(color_list):
    return ColorRGBA(color_list[0], color_list[1], color_list[2], 0)


def get_24bits_color_from_msg(color_request):
    r = int(color_request.r)
    g = int(color_request.g)
    b = int(color_request.b)
    return Color(r, g, b)


def color(r, g, b):
    """
    return a 24 bit color value in the form of a Color object
    """
    return Color(r, g, b)


# Rainbow led related usefully methods
def wheel_rgba(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return ColorRGBA(pos * 3, 255 - pos * 3, 0, 0)
    elif pos < 170:
        pos -= 85
        return ColorRGBA(255 - pos * 3, 0, pos * 3, 0)
    else:
        pos -= 170
        return ColorRGBA(0, pos * 3, 255 - pos * 3, 0)
