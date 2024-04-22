# !/usr/bin/env python

# Test FQC V1.4

import json
import subprocess
from datetime import datetime

import numpy as np
import psutil
import requests
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, NiryoRosWrapperException
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ButtonAction

import rospy
from niryo_robot_database.srv import SetSettings
from niryo_robot_rpi.srv import LedBlinker, LedBlinkerRequest
from niryo_robot_rpi.srv import ScanI2CBus
from std_msgs.msg import String, Int32

# This is the variable to modify for the demo program. FULL is here to define the default test (here demo)
LOOPS = 1
CALIBRATION_LOOPS = 1
SPIRAL_LOOPS = 1
HIGH_SPEED_LOOP = 10
NB_HOURS = 5
FULL = 0

SPEED = 100  # %
ACCELERATION = 100  # %

USE_BUTTON = False
VOLUME = 100
USE_VOCAL = False

WHITE = [255, 255, 255]
GREEN = [50, 255, 0]
BLACK = [0, 0, 0]
BLUE = [15, 50, 255]
PURPLE = [153, 51, 153]
PINK = [255, 0, 255]
RED = [255, 0, 0]
YELLOW = [255, 255, 0]

j_limit_1m, j_limit_1M, j_limit_2m, j_limit_2M = np.deg2rad(-164), np.deg2rad(164), np.deg2rad(-90), np.deg2rad(31)
j_limit_3m, j_limit_3M, j_limit_4m, j_limit_4M = np.deg2rad(-74), np.deg2rad(86), np.deg2rad(-116), np.deg2rad(116)
j_limit_5m, j_limit_5M, j_limit_6m, j_limit_6M = np.deg2rad(-105), np.deg2rad(105), np.deg2rad(-144), np.deg2rad(144)


def almost_equal_array(a, b, decimal=1):
    try:
        np.testing.assert_almost_equal(a, b, decimal)
        return True
    except AssertionError:
        return False


class TestStatus(object):
    FAILURE = -1
    NONE = 0
    SUCCESS = 1


class TestFailure(Exception):
    pass


class TestReport(object):  # We define the report to send to RFM

    def __init__(self, header):
        self._header = header
        self._report = ""
        self._report_dict = {}

    def __str__(self):
        return self._report

    def append(self, message):
        new_line = "[{}] - {} - {}.".format(self._header, datetime.now(), message)
        rospy.loginfo('\033[96;24;23m ' + new_line + ' \033[0m')
        self._report += new_line

    def execute(self, function, prefix, args=None, ret=False):  # Execute a function and create a report of the latter
        try:
            status, message = function() if args is None else function(*args)
        except Exception as e:
            self.append("{}{} - {}".format(prefix, " failed" if prefix else "Failed", str(e)))
            raise TestFailure(e)
        else:
            if status >= 0:
                success_message = " succeed" if prefix else "Succeed"
                self.append("{}{} - {} - {}".format(prefix, success_message, status, message))
                if ret:
                    return status
            else:
                success_message = " failed" if prefix else "Failed"
                self.append("{}{} - {} - {}".format(prefix, success_message, status, message))
                raise TestFailure(message)

    @property
    def report(self):
        return self._report


class TestStep(object):  # This is the part who execute each program

    def __init__(self, function, name, critical=False):
        self.__function = function
        self.__name = name
        self.__report = TestReport(name)
        self.__critical = critical

        self.__status = TestStatus.NONE

    def run(self, *args, **kwargs):  # run each function
        self.__status = TestStatus.NONE

        try:
            self.__function(self.__report)
        except TestFailure as e:
            error_message = "[Error] [{}] - {}".format(self.__name, str(e))
            self.__report.append(error_message)
        except Exception as e:
            error_message = "[Error] [{}] - {}".format(self.__name, str(e))
            self.__report.append(error_message)
        else:
            self.__status = TestStatus.SUCCESS
            return True

        self.__status = TestStatus.FAILURE
        if self.__critical:
            raise TestFailure(error_message)
        return False

    def get_report(self):
        return {"name": self.__name, "status": self.__status, "report": str(self.__report)}


class TestProduction:  # Here we create the program who contain each function that we want

    def __init__(self):
        self.__functions = TestFunctions()
        self.__success = False

        if FULL == 1:  # For the short test (freemotion and button)
            self.__sub_tests = [
                TestStep(self.__functions.name, "Program name", critical=True),
                TestStep(self.__functions.test_cloud_connection, "Test robot connection", critical=True),
                TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
                TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
                TestStep(self.__functions.test_sound, "Test haut parleurs", critical=True),
                TestStep(self.__functions.test_led_ring, "Test led ring", critical=True),
                TestStep(self.__functions.test_joint_limits, "Test joint limits", critical=True),
                TestStep(self.__functions.test_spiral, "Test spiral", critical=True),
                TestStep(self.__functions.test_fun_poses, "Test poses", critical=True),
                TestStep(self.__functions.test_pick_and_place, "Test pick and place", critical=True),
                TestStep(self.__functions.test_high_speed, "Test high speed", critical=True),
                TestStep(self.__functions.end_test, "Test final check", critical=True)
            ]
        if FULL == 2:  # For the long test (custom and button)
            self.__sub_tests = [
                TestStep(self.__functions.name, "Program name", critical=True),
                TestStep(self.__functions.test_cloud_connection, "Test robot connection", critical=True),
                TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
                TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
                TestStep(self.__functions.test_spiral, "Test spiral", critical=True),
                TestStep(self.__functions.test_long, "Long test", critical=True),
                TestStep(self.__functions.end_test, "Test final check", critical=True)
            ]
        if FULL == 0:  # For the demo (only button) and also default mode
            self.__sub_tests = [
                TestStep(self.__functions.name, "Program name", critical=True),
                TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
                TestStep(self.__functions.test_spiral, "Test spiral", critical=True),
                TestStep(self.__functions.test_joint_limits, "Test joint limits", critical=True),
                TestStep(self.__functions.test_fun_poses, "Test poses", critical=True),
                TestStep(self.__functions.test_pick_and_place, "Test pick and place", critical=True),
                TestStep(self.__functions.end_test, "Test final check", critical=True)
            ]
        if FULL == 3:  # Expedition test
            self.__sub_tests = [
                TestStep(self.__functions.name, "Program name", critical=True),
                TestStep(self.__functions.test_cloud_connection, "Test robot connection", critical=True),
                TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
                TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
                TestStep(self.__functions.test_sound, "Test haut parleurs", critical=True),
                TestStep(self.__functions.test_led_ring, "Test led ring", critical=True),
                TestStep(self.__functions.test_joint_limits, "Test joint limits", critical=True),
                TestStep(self.__functions.test_spiral, "Test spiral", critical=True),
                TestStep(self.__functions.test_fun_poses, "Test poses", critical=True),
                TestStep(self.__functions.test_pick_and_place, "Test pick and place", critical=True),
                TestStep(self.__functions.end_test, "Test final check", critical=True)
            ]

    def run(self):  # run the whole program
        rospy.sleep(1)
        self.__functions.led_stop()
        try:
            for test in self.__sub_tests:
                test.run()

        except TestFailure:  # What's append if the test fail
            self.__sub_tests.append(TestStep(self.__functions.test_robot_status, "Test robot status", critical=True))
            # self.__sub_tests[-1].run()
            self.__functions.reset()
            self.__success = False
            self.__functions.led_error()

        else:
            self.__success = True

        return self.__success

    def get_report(self):
        return {"details": [test.get_report() for test in self.__sub_tests], "success": self.__success}

    def print_report(self):
        print(json.dumps(self.get_report(), indent=4, sort_keys=True))

    def send_report(self):
        new_report_publisher = rospy.Publisher('/niryo_robot_reports/test_report', String, queue_size=10)
        rospy.sleep(0.5)

        # Wait for the publisher initialization
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and new_report_publisher.get_num_connections() == 0:
            if (rospy.Time.now() - start_time).to_sec() > 1:
                rospy.logerr('Unable to publish the new report')
                return False
            rospy.sleep(0.1)

        new_report_publisher.publish(json.dumps(self.get_report()))
        rospy.sleep(1)

        rospy.logdebug('test report published')

        return True


class TestFunctions(object):  # definition of each function (some are unused)

    def __init__(self):
        rospy.sleep(2)
        self.__robot = robot
        self.__hardware_version = self.__robot.get_hardware_version()

        self.__robot.set_arm_max_velocity(SPEED)
        self.__robot.set_arm_max_acceleration(ACCELERATION)
        self.__set_led_state_service = rospy.ServiceProxy('/niryo_robot_rpi/set_led_custom_blinker', LedBlinker)
        self.led_stop()

    def reset(self):
        self.__robot.set_arm_max_velocity(100)
        self.__robot.set_arm_max_acceleration(100)

    def led_error(self, duration=360):
        if self.__hardware_version in ['ned', 'one'] and not self.__robot.get_simulation_mode():
            self.__set_led_state_service(True, 5, LedBlinkerRequest.LED_WHITE, duration)

    def led_stop(self):
        if self.__hardware_version in ['ned', 'one'] and not self.__robot.get_simulation_mode():
            self.__set_led_state_service(False, 0, 0, 0)

    def say(self, text, prio=0):  # prio is here to speak even if USE_VOCAL = False
        if (USE_VOCAL or prio == 1) and self.__hardware_version in ['ned2', 'ned3']:
            try:
                self.__robot.sound.say(text, 1)
            except Exception:
                rospy.loginfo("No internet connection, robot can't speak")
        subprocess.run('rm -rf /home/niryo/niryo_robot_saved_files/niryo_robot_user_sounds/last_text_to_speech.mp3',
                       shell=True,
                       stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL)

    def name(self, report):
        if FULL != 0:
            report.execute(self.__robot.sound.set_volume, "Set volume", [VOLUME])

        name = ["Programme demo", "Debut du test court FQC", "Debut du test long FQC",
                "Test avant expedission"]  # expedition
        self.say(name[FULL], 1)

    def ip_say(self, report):
        self.__robot.led_ring.solid(PURPLE)
        ip_eth = "Adresse ip ethernet : " + psutil.net_if_addrs()['eth0'][0].address
        ip_wifi = "Adresse ip wifi : " + psutil.net_if_addrs()['wlan0'][0].address
        while True:  # repeat fonction
            try:
                value = report.execute(self.wait_custom_button_press, "Wait button press to validate", ret=True)

                if value == 2:  # short press
                    self.say(ip_wifi)
                    self.say(ip_eth)
                if value == 1:  # long press
                    break
            except Exception as e:
                report.append(e)
                raise TestFailure(e)
        report.append("Voice step successfully pass")

    def test_cloud_connection(self, report):

        rasp_id = self.__robot.database.get_setting('rasp_id')
        apiKey = self.__robot.database.get_setting('api_key')
        try:
            response = requests.get('https://api.niryo.com/test-reports/api/v1/test-reports/ping',
                                    headers={
                                        'accept': 'application/json', 'identifier': rasp_id, 'apiKey': apiKey
                                    })
            if response.status_code >= 400:
                report.append("error {}".format(response.text))
                self.say("Erreur connexion RFM", 1)
                raise TestFailure("Erreur connexion RFM")
            else:
                report.append("Service test_reports successfully reached")
                self.say("Connexion au serveur RFM rai hussi", 1)  # reussi
        except Exception as e:
            report.append(e)
            report.append("No internet connection")
            raise TestFailure(e)

    def test_robot_status(self, report):
        try:
            hardware_status = self.__robot.get_hardware_status()
        except rospy.exceptions.ROSException as e:
            report.append(str(e))
            raise TestFailure(e)

        if hardware_status.error_message:
            message = "Hardware status Error - {}".format(hardware_status.error_message)
            report.append(message)
            raise TestFailure(message)

        if any(hardware_status.hardware_errors):
            message = "Hardware status Motor Error - {}".format(hardware_status.hardware_errors_message)
            report.append(message)
            raise TestFailure(message)

        if hardware_status.rpi_temperature > 70:
            message = "Rpi overheating"
            report.append(message)
            raise TestFailure(message)
        report.append("Hardware status ok")

        try:
            robot_status = self.__robot.get_robot_status()
        except rospy.exceptions.ROSException as e:
            report.append(str(e))
            raise TestFailure(e)

        if robot_status.robot_status < 0:
            message = "Robot status - {} - {}".format(robot_status.robot_status_str, robot_status.robot_message)
            report.append(message)
            raise TestFailure(message)
        report.append("Robot status ok")

        def test_i2c():
            if self.__robot.get_simulation_mode():
                report.append("I2C Bus - Skipped in simulation mode")
                return

            try:
                rospy.wait_for_service("/niryo_robot_rpi/scan_i2c_bus", timeout=0.2)
                resp = rospy.ServiceProxy("/niryo_robot_rpi/scan_i2c_bus", ScanI2CBus).call()

                if not resp.is_ok:
                    message = "I2C Bus - Missing components: {}".format(resp.missing)
                    report.append(message)
                    raise TestFailure(message)

            except (rospy.ROSException, rospy.ServiceException) as e:
                report.append(str(e))
                raise TestFailure(e)

        if self.__hardware_version in ['ned2', 'ned3']:
            test_i2c()

    def test_calibration(self, report):
        for i in range(CALIBRATION_LOOPS):

            self.__robot.request_new_calibration()
            rospy.sleep(0.1)
            report.execute(self.__robot.calibrate_auto, "Calibration")
            self.__robot.set_learning_mode(False)
            rospy.sleep(0.1)
            report.execute(self.move_and_compare, "Move after calibration", args=[6 * [0], 1])

            if self.__hardware_version in ['ned2']:
                if i == 0:
                    self.__robot.led_ring.flashing(BLUE)
                else:
                    self.__robot.led_ring.flashing(PURPLE)

                self.say("Appuyez sur SAVE pour valider la position du elbo a 90 degrer")

                report.execute(self.wait_save_button_press, "Wait save button press to validate")

            self.__robot.move_to_sleep_pose()

    def test_led_ring(self, report):
        if self.__hardware_version not in ['ned2', 'ned3']:
            report.append("Led ring test - Skipped on {}".format(self.__hardware_version))
            return

        self.__robot.led_ring.solid(WHITE)

        # self.say("Premier test du ruban led")
        report.append("Led ring color set to WHITE")
        self.say("Appuyer sur custom pour valider le LED RING Blanc")
        report.execute(self.wait_custom_button_press, "Wait custom button press to continue")

        self.__robot.led_ring.rainbow_cycle()
        report.append("Led ring color set to RAINBOW")
        self.say("Appuyer sur CUSTOM pour valider le LED RING multicolor tournant")
        # self.say("Validez le test")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

    def test_sound(self, report):
        if self.__hardware_version not in ['ned2', 'ned3']:
            report.append("Sound test - Skipped on {}".format(self.__hardware_version))
            return

        self.__robot.led_ring.solid(PURPLE)

        self.say("Test des haut parleurs")

        def play_channel(channel):
            subprocess.run((f'ffplay -nodisp -autoexit -af "pan=stereo|c{channel}=c0+c1" '
                            f'/home/niryo/catkin_ws/src/niryo_robot_sound/niryo_robot_state_sounds/reboot.wav'),
                           shell=True,
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)

        report.execute(self.__robot.sound.set_volume, "Set volume", [int(VOLUME / 2)])
        report.append("Volume set to {}%".format(int(VOLUME / 2)))
        self.say("Volume haut parleur gauche 50 pour 100")

        play_channel(1)

        report.execute(self.__robot.sound.set_volume, "Set volume", [VOLUME])
        report.append("Volume set to {}%".format(VOLUME))
        self.say("Volume haut parleur gauche 100 pour 100")

        play_channel(1)

        report.execute(self.__robot.sound.set_volume, "Set volume", [int(VOLUME / 2)])
        report.append("Volume set to {}%".format(int(VOLUME / 2)))
        self.say("Volume haut parleur droit 50 pour 100")

        play_channel(0)

        report.execute(self.__robot.sound.set_volume, "Set volume", [VOLUME])
        report.append("Volume set to {}%".format(VOLUME))
        self.say("Volume haut parleur droit 100 pour 100")

        play_channel(0)

        self.say("Appuyez sur CUSTOM pour validez le test des haut parleurs")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

    def test_freedrive(self, report):
        if self.__hardware_version not in ['ned2', 'ned3']:
            report.append("Freemotion test - Skipped on {}".format(self.__hardware_version))
            return

        def wait_learning_mode(value):
            start_time = rospy.Time.now()
            while self.__robot.get_learning_mode() == value:
                if (rospy.Time.now() - start_time).to_sec() > 20:
                    return -1, "Timeout: no detected".format("learning mode" if value else "torque on")
                rospy.sleep(0.1)

            return 1, "Learning mode".format("enabled" if value else "disabled")

        # self.say("Test de free motion")

        joint_limit = self.__robot.get_axis_limits()[1]['joint_limits']

        self.__robot.led_ring.solid(GREEN)
        report.append("Led ring color set to GREEN")

        report.append("Wait learning mode")
        report.execute(wait_learning_mode, "Wait learning mode", [True])

        for i in range(0, 30, 6):
            for j in range(2):
                self.__robot.led_ring.set_led_color(i + j, BLACK)

        report.append("Wait joint1 minimum limit")
        start_time = rospy.Time.now()
        while not self.__robot.get_joints()[0] < joint_limit['joint_1']['min'] + 0.2:
            if (rospy.Time.now() - start_time).to_sec() > 20:
                report.append("Joint1 minimum limit not reached")
                break
        else:
            # self.say("Limite validee")
            report.append("Joint1 minimum limit reached")

        for i in range(2, 30, 6):
            for j in range(2):
                self.__robot.led_ring.set_led_color(i + j, BLACK)

        report.append("Wait joint1 minimum limit")
        start_time = rospy.Time.now()
        while not self.__robot.get_joints()[0] > joint_limit['joint_1']['max'] - 0.2:
            if (rospy.Time.now() - start_time).to_sec() > 20:
                report.append("Joint1 maximum limit not reached")
                break
        else:
            # self.say("Limite validee")
            report.append("Joint1 maximum limit reached")

        for i in range(4, 30, 6):
            for j in range(2):
                self.__robot.led_ring.set_led_color(i + j, BLACK)

        rospy.sleep(1)
        self.__robot.led_ring.flashing(GREEN)

        report.execute(wait_learning_mode, "Wait learning mode disabled", [False])
        rospy.sleep(1)

    def test_io(self, report):
        if self.__hardware_version not in ['ned2', 'ned3']:
            report.append("IO test - Skipped on {}".format(self.__hardware_version))
            return

        def test_digital_io_value(io_name, state):
            io_state = self.__robot.digital_read(io_name)
            if io_state != state:
                raise TestFailure("Non expected value on digital input {} - Actual {} - Target {}".format(
                    io_name, io_state, state))
            return 1, "Success"

        def test_analog_io_value(io_name, value, error=0.3):
            io_state = self.__robot.analog_read(io_name)
            if not (value - error <= io_state <= value + error):
                raise TestFailure("Non expected value on digital input {} - Actual {} - Target {}".format(
                    io_name, io_state, value))
            return 1, "Success"

        self.__robot.led_ring.solid(PINK)

        # Test digital ios
        dio = self.__robot.get_digital_io_state()

        for do in dio.digital_output:
            report.execute(self.__robot.digital_write, 'Set digital output {}'.format(do.name), [do.name, True])
        for di in dio.digital_input:
            report.execute(test_digital_io_value, 'Test digital input {} is High'.format(di.name), [di.name, True])

        # Test analog ios
        aio = self.__robot.get_analog_io_state()
        for ao in aio.analog_outputs:
            report.execute(self.__robot.digital_write, 'Set analog output {} to 5V'.format(ao.name), [ao.name, 5.0])
        for ai in aio.analog_inputs:
            report.execute(test_analog_io_value, 'Test analog input {} is 5V'.format(ai.name), [ai.name, 5.0])

        self.__robot.led_ring.flashing(PINK)
        self.wait_custom_button_press()

        for do in dio.digital_output:
            report.execute(self.__robot.digital_write, 'Unset digital output {}'.format(do.name), [do.name, False])
        for di in dio.digital_input:
            report.execute(test_digital_io_value, 'Test digital input {} is Low'.format(di.name), [di.name, False])
        for ao in aio.analog_outputs:
            report.execute(self.__robot.digital_write, 'Set analog output {} to 0V'.format(ao.name), [ao.name, 0.0])
        for ai in aio.analog_inputs:
            report.execute(test_analog_io_value, 'Test analog input {} is 0V'.format(ai.name), [ai.name, 0.0])

    def test_joint_limits(self, report):
        if self.__hardware_version in ['ned2', 'ned3']:
            self.__robot.led_ring.rainbow_cycle()
            # self.say("Test des limites des joints")

        self.__robot.set_learning_mode(False)
        rospy.sleep(1)

        default_joint_pose = 6 * [0.0]

        first_target, second_target, third_target, last_target = 6 * [0], 6 * [0], 6 * [0], 6 * [0]

        first_target[0], first_target[3], first_target[4], first_target[
            5] = j_limit_1m, j_limit_4m, j_limit_5m, j_limit_6m

        second_target[1], second_target[2] = j_limit_2M, j_limit_3m

        third_target[0], third_target[3], third_target[4], third_target[
            5] = j_limit_1M, j_limit_4M, j_limit_5M, j_limit_6M

        last_target[2], last_target[4] = j_limit_3M, j_limit_5m

        poses = [(default_joint_pose, 1, 3), (first_target, 1, 4), (default_joint_pose, 1, 4), (second_target, 1, 3),
                 (default_joint_pose, 1, 3), (third_target, 1, 4), (last_target, 1, 4)]

        for loop_index in range(LOOPS):
            for position_index, step in enumerate(poses):
                joint_position, precision, duration = step
                report.execute(self.move_and_compare_without_moveit,
                               "Move number {}.{}".format(loop_index, position_index),
                               args=[joint_position, precision, duration])

    def test_spiral(self, report):
        if self.__hardware_version in ['ned2', 'ned3']:
            self.__robot.led_ring.rainbow_cycle()
            # self.say("Test des spirales")

        for loop_index in range(SPIRAL_LOOPS):
            report.execute(self.__robot.move_pose,
                           "Loop {} - Move to spiral center".format(loop_index), [0.3, 0, 0.2, 0, 1.57, 0])
            report.execute(self.__robot.move_spiral, "Loop {} - Execute spiral".format(loop_index), [0.15, 5, 216, 3])

    def test_fun_poses(self, report):
        if self.__hardware_version in ['ned2', 'ned3']:
            self.__robot.led_ring.rainbow_cycle()
            # self.say("Test de divers movements")

        waypoints = [[0.16, 0.00, -0.75, -0.56, 0.60, -2.26], [2.25, -0.25, -0.90, 1.54, -1.70, 1.70],
                     [1.40, 0.35, -0.34, -1.24, -1.23, -0.10], [0.00, 0.60, 0.46, -1.55, -0.15,
                                                                2.50], [-1.0, 0.00, -1.00, -1.70, -1.35, -0.14]]

        for loop_index in range(LOOPS):
            for wayoint_index, wayoint in enumerate(waypoints):
                report.execute(self.move_and_compare_without_moveit,
                               "Loop {}.{} - Fun move".format(loop_index, wayoint_index),
                               args=[wayoint, 1, 4])

    def test_pick_and_place(self, report):
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        if self.__hardware_version in ['ned2', 'ned3']:
            self.say("Mettre le grippeur et appuyer sur CUSTOM")
            self.__robot.led_ring.flashing(YELLOW)
            self.wait_custom_button_press()
            self.__robot.led_ring.solid(YELLOW)
            # self.say("Test de pick and place")

        report.execute(self.__robot.update_tool, "Scan tool")
        report.append("Detected tool: {}".format(self.__robot.get_current_tool_id()))

        self.__robot.enable_tcp(True)

        z_offset = 0.15 if self.__robot.get_current_tool_id() <= 0 else 0.02
        sleep_pose = [0.3, 0, 0.3, 0, 1.57, 0]
        home_pose = [0.3, 0, 0.3, 0, 0, 0]
        pick_1 = [0, 0.2, z_offset, 0, 1.57, 0]
        pick_2 = [0, -0.2, z_offset, 0, 1.57, 0]
        place_1 = [0.15, 0, z_offset, 0, 1.57, 0]
        place_2 = [0.22, 0, z_offset, 0, 1.57, 0]

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 1st piece", pick_1)
        report.execute(self.__robot.place_from_pose, "Place 1st piece", place_1)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 2nd piece", pick_2)
        report.execute(self.__robot.place_from_pose, "Place 2nd piece", place_2)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 1st piece from center", place_1)
        pick_1[5] = 1.57
        report.execute(self.__robot.place_from_pose, "Replace 1st piece", pick_1)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 2nd piece from center", place_2)
        pick_2[5] = -1.57
        report.execute(self.__robot.place_from_pose, "Replace 2nd piece", pick_2)

        try:
            report.execute(self.__robot.grasp_with_tool, "Close gripper")
        except Exception:
            report.append("No tool connected")

        report.execute(self.__robot.move_pose, "Move to home pose", home_pose)

        self.__robot.enable_tcp(False)

    def test_high_speed(self, report):

        self.__robot.set_arm_max_velocity(170)
        self.__robot.set_arm_max_acceleration(100)

        default_joint_pose = 6 * [0.0]

        first_joint_min, first_joint_max, second_joint_min, second_joint_max = 6 * [0], 6 * [0], 6 * [0], 6 * [0]
        third_joint_min, third_joint_max, fourth_joint_min, fourth_joint_max = 6 * [0], 6 * [0], 6 * [0], 6 * [0]
        fifth_joint_min, fifth_joint_max, sixth_joint_min, sixth_joint_max = 6 * [0], 6 * [0], 6 * [0], 6 * [0]

        first_joint_min[0], first_joint_min[1], first_joint_min[2] = j_limit_1m, j_limit_2m, j_limit_3M
        first_joint_max[0], first_joint_max[1], first_joint_max[2] = j_limit_1M, j_limit_2m, j_limit_3M

        second_joint_min[1], second_joint_min[2] = j_limit_2m, j_limit_3M
        second_joint_max[1], second_joint_max[2] = j_limit_2M, j_limit_3M

        third_joint_min[1], third_joint_min[2] = j_limit_2M, j_limit_3m
        third_joint_max[1], third_joint_max[2] = j_limit_2M, j_limit_3M

        fourth_joint_min[3], fourth_joint_min[4] = j_limit_4m, np.deg2rad(-90)
        fourth_joint_max[3], fourth_joint_max[4] = j_limit_4M, np.deg2rad(-90)

        fifth_joint_min[4] = j_limit_5m
        fifth_joint_max[4] = j_limit_5M

        sixth_joint_min[5] = j_limit_6m
        sixth_joint_max[5] = j_limit_6M

        poses = [
            first_joint_min,
            first_joint_max,
            second_joint_min,
            second_joint_max,
            third_joint_min,
            third_joint_max,
            fourth_joint_min,
            fourth_joint_max,
            fifth_joint_min,
            fifth_joint_max,
            sixth_joint_min,
            sixth_joint_max
        ]

        c = [-1, 0, 0, 2, 1, 0]  # correctif for 10 loop and 1min per axis :  c = [-1, 0, 0, 2, 1, 0]

        self.say("Mettre la masse et appuyer sur CUSTOM")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

        for position_index in range(int(len(poses) / 2)):

            if position_index == 3:  # limit of the axe 4
                self.__robot.set_arm_max_velocity(150)  # to avoid problem
            if position_index == 4:  # 150% speed is enough
                self.__robot.set_arm_max_velocity(200)  #
            if position_index == 5:
                report.execute(self.move_and_compare, "Change payload", args=[default_joint_pose])
                self.say("Mettre la masse des porter et appuyer sur CUSTOM")  # masse deportee
                report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

            for loop_index in range(HIGH_SPEED_LOOP + c[position_index]):
                if position_index == 3 and loop_index == 6:  # change position of the Payload
                    fourth_joint_min[4] = np.deg2rad(90)  # at the half of the movements
                    fourth_joint_max[4] = np.deg2rad(90)  #

                joint_position_min = poses[2 * position_index]
                joint_position_max = poses[2 * position_index + 1]
                report.execute(self.move_and_compare,
                               "Move number {}.{} min ".format(position_index + 1, loop_index),
                               args=[joint_position_min])
                report.execute(self.move_and_compare,
                               "Move number {}.{} max".format(position_index + 1, loop_index),
                               args=[joint_position_max])

        report.execute(self.move_and_compare, "Move HOME", args=[default_joint_pose])
        self.say("Enlever la masse des porter et appuyer sur CUSTOM ")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")
        self.__robot.set_arm_max_velocity(80)
        self.__robot.set_arm_max_acceleration(50)

    def test_long(self, report):

        self.__robot.set_arm_max_velocity(100)
        self.__robot.set_arm_max_acceleration(100)
        self.__robot.led_ring.solid(GREEN)
        nb_collision, nb_loop = 0, 0
        report.append("Start of the long test")

        waypoints = [[j_limit_1m, j_limit_2M, np.deg2rad(-55), j_limit_4M, j_limit_5m,
                      j_limit_6M], [j_limit_1M, j_limit_2M, j_limit_3m, j_limit_4m, j_limit_5M, j_limit_6m],
                     [j_limit_1M, j_limit_2M, j_limit_3M, j_limit_4m, j_limit_5M,
                      j_limit_6m], [j_limit_1m, j_limit_2M, j_limit_3M, j_limit_4M, j_limit_5m, j_limit_6M],
                     [j_limit_1m, j_limit_2m, j_limit_3M, j_limit_4M, j_limit_5M,
                      j_limit_6M], [j_limit_1M, j_limit_2m, j_limit_3M, j_limit_4m, j_limit_5m,
                                    j_limit_6m], [0, 0.5, -1.25, 0, 0, 0]]

        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < NB_HOURS * 3600:
            nb_loop += 1
            for wayoint_index, wayoint in enumerate(waypoints):
                try:
                    self.__robot.move_joints(*wayoint)

                except NiryoRosWrapperException as e:
                    self.__robot.clear_collision_detected()
                    report.append("{}".format(str(e)))
                    nb_collision += 1
                    if nb_collision >= 5:
                        report.append("End of the test with {} loop and {} collision(s)".format(nb_loop, nb_collision))
                        raise TestFailure("Stop program : number of collision higher than 5")
                    self.__robot.wait(5)
                    if nb_collision >= 3:
                        self.__robot.led_ring.flashing(YELLOW)
        report.append("End of the test with {} loop and {} collision(s)".format(nb_loop, nb_collision))

        if nb_collision > 2:
            raise TestFailure("Number of collision higher than 2")

    def end_test(self, report):
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        if self.__hardware_version in ['ned2', 'ned3']:
            self.__robot.led_ring.flashing(BLUE)
            report.append("End")
            if FULL == 0:
                self.say("Fin de la demo", 1)
            if FULL == 1:
                self.say("Appuyez sur SAVE pour valider la position du elbo a 90 degrer")
                report.execute(self.wait_save_button_press, "Wait save button press to validate")
                self.say("Fin du test court FQC")
            if FULL == 2:
                self.say("Fin du test long FQC", 1)
            if FULL == 3:
                self.say("Appuyez sur SAVE pour valider la position du elbo a 90 degrer")
                report.execute(self.wait_save_button_press, "Wait save button press to validate")
                self.say("Fin du test avant expedission")
        self.__robot.led_ring.solid(BLUE)
        self.__robot.move_to_sleep_pose()
        self.__robot.set_arm_max_velocity(100)
        self.__robot.set_arm_max_acceleration(100)

    def wait_custom_button_press(self, timeout=600):
        if not USE_BUTTON:
            # raw_input('Enter to continue')
            return 1, "Press custom button step skipped"

        action = self.__robot.custom_button.wait_for_any_action(timeout=timeout)

        if action == ButtonAction.NO_ACTION:
            return -1, "Timeout: no press detected"

        return action, "Press detected"

    @staticmethod
    def wait_save_button_press(timeout=600):
        if not USE_BUTTON:
            # raw_input('Enter to continue')
            return 1, "Press save button step skipped"
        try:
            rospy.wait_for_message("/niryo_robot/blockly/save_current_point", Int32, timeout=timeout)
            return 1, "Press detected"
        except rospy.ROSException:

            return -1, "Timeout: no press detected"

    def move_and_compare(self, target, precision_decimal=1):
        status, message = self.__robot.move_joints(*target)
        if status >= 0:
            current_joints = self.__robot.get_joints()
            if not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
                raise TestFailure("Target not reached - Actual {} - Target {}".format(current_joints, target))
        return status, message

    def move_and_compare_without_moveit(self, target, precision_decimal=1, duration=4):
        _status, _message = self.__robot.move_without_moveit(target, duration)
        start_time = rospy.Time.now()
        while not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
            if (rospy.Time.now() - start_time).to_sec() > 5:
                raise TestFailure("Target not reached - Actual {} - Target {}".format(
                    self.__robot.get_joints(), target))
            rospy.sleep(0.1)
        return 1, "Success"


if __name__ == '__main__':
    rospy.init_node('niryo_test_FQC_ros_wrapper')
    robot = NiryoRosWrapper()

    if robot.get_learning_mode() or robot.custom_button.is_pressed():  # detect in which mode we are

        if robot.get_learning_mode() and robot.custom_button.is_pressed():
            LOOPS = 5
            CALIBRATION_LOOPS = 1
            SPIRAL_LOOPS = 5

            USE_VOCAL = True
            USE_BUTTON = True
            SPEED = 80  # %
            ACCELERATION = 50  # %
            FULL = 3

        elif robot.get_learning_mode():

            LOOPS = 5
            CALIBRATION_LOOPS = 1
            SPIRAL_LOOPS = 5

            USE_VOCAL = True
            USE_BUTTON = True
            SPEED = 80  # %
            ACCELERATION = 50  # %
            FULL = 1

        elif robot.custom_button.is_pressed():

            SPIRAL_LOOPS = 3
            SPEED = 80  # %
            ACCELERATION = 50  # %
            FULL = 2

        robot.sound.play('ready.wav')

        print("----- START -----")
        test = TestProduction()
        prog = test.run()
        print("----- END -----")

        test.print_report()

        try:
            set_setting = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
            set_setting('test_report_done', 'True', 'bool')
            try:
                test.send_report()
            except Exception as _e:
                rospy.logerr(f'Failed to send report: {_e}')
            rospy.sleep(3)
        except Exception as _e:
            rospy.logerr(_e)
        if not prog:
            raise TestFailure('Program failure : {}'.format(prog))
    else:  # Demo program
        test = TestProduction()
        if not test.run():
            raise TestFailure('Program failure')
