#!/usr/bin/env python

import rospy
import json
import numpy as np
from datetime import datetime

from std_msgs.msg import String, Int32
from niryo_robot_reports.msg import Service

from niryo_robot_database.srv import SetSettings
from niryo_robot_rpi.srv import ScanI2CBus
from niryo_robot_rpi.srv import LedBlinker, LedBlinkerRequest
from niryo_robot_reports.srv import CheckConnection

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, NiryoRosWrapperException
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ButtonAction

LOOPS = 1
CALIBRATION_LOOPS = 1
SPIRAL_LOOPS = 1

SPEED = 200  # %
ACCELERATION = 100  # %

WHITE = [255, 255, 255]
GREEN = [50, 255, 0]
BLACK = [0, 0, 0]
BLUE = [15, 50, 255]
PURPLE = [153, 51, 153]
PINK = [255, 0, 255]
RED = [255, 0, 0]
YELLOW = [255, 255, 0]

USE_BUTTON = False
VOLUME = 50
USE_VOCAL = False


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


class TestReport(object):
    def __init__(self, header):
        self._header = header
        self._report = ""
        self._report_dict = {}

    def __str__(self):
        return self._report

    def append(self, message):
        new_line = "[{}] - {} - {}.".format(self._header, datetime.now(), message)
        rospy.loginfo('\033[96;24;23m ' + new_line + ' \033[0m')
        self._report += new_line + "\\n"

    def execute(self, function, prefix="", args=None):
        try:
            status, message = function() if args is None else function(*args)
        except Exception as e:
            self.append("{}{} - {}".format(prefix, " failed" if prefix else "Failed", str(e)))
            raise TestFailure(e)
        else:
            if status >= 0:
                success_message = " succeed" if prefix else "Succeed"
                self.append("{}{} - {} - {}".format(prefix, success_message, status, message))
            else:
                success_message = " failed" if prefix else "Failed"
                self.append("{}{} - {} - {}".format(prefix, success_message, status, message))
                raise TestFailure(message)

    @property
    def report(self):
        return self._report


class TestStep(object):

    def __init__(self, function, name, critical=False):
        self.__function = function
        self.__name = name
        self.__report = TestReport(name)
        self.__critical = critical

        self.__status = TestStatus.NONE

    def run(self, *args, **kwargs):
        self.__status = TestStatus.NONE

        try:
            self.__function(self.__report)
        except TestFailure as e:
            error_message = "[Error] [{}] - {}".format(self.__name, str(e))
            print(error_message)
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


class TestProduction:

    def __init__(self, full=True):
        self.__functions = TestFunctions()
        self.__success = False

        if full:
            self.__sub_tests = [
                # TestStep(self.__functions.test_cloud_connection, "Test robot connection", critical=True),
                TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
                # TestStep(self.__functions.test_sound, "Test sound", critical=False),
                TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
                TestStep(self.__functions.test_led_ring, "Test led ring", critical=False),
                # TestStep(self.__functions.test_freedrive, "Test free motion", critical=False),
                TestStep(self.__functions.test_joint_limits, "Test move", critical=True),
                TestStep(self.__functions.test_spiral, "Test move", critical=True),
                TestStep(self.__functions.test_fun_poses, "Test move", critical=True),
                TestStep(self.__functions.test_pick_and_place, "Test pick and place", critical=True),
                TestStep(self.__functions.end_test, "Test final check", critical=True),
            ]
        else:
            self.__sub_tests = [
                TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
                TestStep(self.__functions.test_spiral, "Test move", critical=True),
                TestStep(self.__functions.test_joint_limits, "Test move", critical=True),
                TestStep(self.__functions.test_fun_poses, "Test move", critical=True),
                TestStep(self.__functions.test_pick_and_place, "Test pick and place", critical=True),
                TestStep(self.__functions.end_test, "Test final check", critical=True),
            ]

    def run(self):
        rospy.sleep(1)
        self.__functions.led_stop()
        try:
            for test in self.__sub_tests:
                test.run()

        except TestFailure:
            self.__sub_tests.append(TestStep(self.__functions.test_robot_status, "Test robot status", critical=True))
            self.__sub_tests[-1].run()
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
        new_report_publisher.publish('')
        rospy.logdebug('test report published')
        return True


class TestFunctions(object):

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

    def say(self, text):
        if USE_VOCAL and self.__hardware_version in ['ned2']:
            self.__robot.sound.say(text, 1)

    def test_cloud_connection(self, report):
        if self.__robot.get_simulation_mode():
            report.append("Test Cloud connection - Skipped in simulation mode")
            return

        check_connection_service = rospy.ServiceProxy('/niryo_robot_reports/check_connection', CheckConnection)
        result = check_connection_service(Service(Service.TEST_REPORTS))
        if result.status < 0:
            error_str = "Service test_report doesn't exists"
            report.append(error_str)
            raise TestFailure("Service test_report doesn't exists")
        if result.result is False:
            error_str = "test_reports api didnt respond"
            report.append(error_str)
            raise TestFailure(error_str)
        report.append('Service test_reports successfully reached')

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

        try:
            robot_status = self.__robot.get_robot_status()
        except rospy.exceptions.ROSException as e:
            report.append(str(e))
            raise TestFailure(e)

        if robot_status.robot_status < 0:
            message = "Robot status - {} - {}".format(robot_status.robot_status_str, robot_status.robot_message)
            report.append(message)
            raise TestFailure(message)

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

        if self.__hardware_version in ['ned2']:
            test_i2c()

    def test_calibration(self, report):
        for _ in range(CALIBRATION_LOOPS):
            self.__robot.request_new_calibration()
            rospy.sleep(0.1)

            report.execute(self.__robot.calibrate_auto, "Calibration")
            self.__robot.set_learning_mode(False)
            rospy.sleep(0.1)
            report.execute(self.move_and_compare, "Move after calibration", args=[6 * [0], 1])

            if self.__hardware_version in ['ned2']:
                self.__robot.led_ring.flashing(BLUE)

                self.say("Validez la position du robot")
                report.execute(self.wait_save_button_press, "Wait save button press to validate")

            self.__robot.move_to_sleep_pose()

    def test_led_ring(self, report):
        if self.__hardware_version not in ['ned2']:
            report.append("Led ring test - Skipped on {}".format(self.__hardware_version))
            return

        self.__robot.led_ring.solid(WHITE)

        self.say("Premier test du ruban led")
        report.append("Led ring color set to WHITE")
        self.say("Validez le test")
        report.execute(self.wait_custom_button_press, "Wait custom button press to continue", args=[60, ])

        self.__robot.led_ring.rainbow_cycle()
        report.append("Led ring color set to RAINBOW")
        self.say("Second test du ruban led")
        self.say("Validez le test")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate", args=[60, ])

    def test_sound(self, report):
        if self.__hardware_version not in ['ned2']:
            report.append("Sound test - Skipped on {}".format(self.__hardware_version))
            return

        self.__robot.led_ring.solid(PURPLE)
        report.append("Led ring color set to PURPLE")

        report.execute(self.__robot.sound.set_volume, "Set volume", [VOLUME])
        report.append("Volume set to {}%".format(VOLUME))

        self.say("Test de son")

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/calibration_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])
        rospy.sleep(0.5)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/connection_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])
        rospy.sleep(0.5)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/robot_ready_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])

        self.__robot.led_ring.flashing(PURPLE)
        self.say("Validez le test")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

    def test_freedrive(self, report):
        if self.__hardware_version not in ['ned2']:
            report.append("Freemotion test - Skipped on {}".format(self.__hardware_version))
            return

        def wait_learning_mode(value):
            start_time = rospy.Time.now()
            while self.__robot.get_learning_mode() == value:
                if (rospy.Time.now() - start_time).to_sec() > 20:
                    return -1, "Timeout: no detected".format("learning mode" if value else "torque on")
                rospy.sleep(0.1)

            return 1, "Learning mode".format("enabled" if value else "disabled")

        self.say("Test de free motion")

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
            self.say("Limite validee")
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
            self.say("Limite validee")
            report.append("Joint1 maximum limit reached")

        for i in range(4, 30, 6):
            for j in range(2):
                self.__robot.led_ring.set_led_color(i + j, BLACK)

        rospy.sleep(1)
        self.__robot.led_ring.flashing(GREEN)

        report.execute(wait_learning_mode, "Wait learning mode disabled", [False])
        rospy.sleep(1)

    def test_io(self, report):
        if self.__hardware_version not in ['ned2']:
            report.append("IO test - Skipped on {}".format(self.__hardware_version))
            return

        def test_digital_io_value(io_name, state):
            io_state = self.__robot.digital_read(io_name)
            if io_state != state:
                raise TestFailure(
                    "Non expected value on digital input {} - Actual {} - Target {}".format(io_name, io_state, state))
            return 1, "Success"

        def test_analog_io_value(io_name, value, error=0.3):
            io_state = self.__robot.analog_read(io_name)
            if not (value - error <= io_state <= value + error):
                raise TestFailure(
                    "Non expected value on digital input {} - Actual {} - Target {}".format(io_name, io_state, value))
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
        if self.__hardware_version in ['ned2']:
            self.__robot.led_ring.rainbow_cycle()
            self.say("Test des limites des joints")

        self.__robot.set_learning_mode(False)
        rospy.sleep(1)

        joint_limit = self.__robot.get_axis_limits()[1]['joint_limits']
        joint_names = sorted(joint_limit.keys())

        default_joint_pose = 6 * [0.0]

        first_target = [joint_limit[joint_name]['min'] + 0.1 for joint_name in joint_names]
        first_target[1] = first_target[2] = 0

        second_target = 6 * [0]
        second_target[1] = joint_limit[joint_names[1]]['max'] - 0.1
        second_target[2] = joint_limit[joint_names[2]]['min'] + 0.1

        third_target = [joint_limit[joint_name]['max'] - 0.1 for joint_name in joint_names]
        third_target[1] = third_target[2] = 0

        last_target = 6 * [0]
        last_target[2] = joint_limit[joint_names[2]]['max'] - 0.1
        last_target[4] = joint_limit[joint_names[4]]['min'] + 0.1

        poses = [(default_joint_pose, 1, 3),
                 (first_target, 1, 4),
                 (default_joint_pose, 1, 4),
                 (second_target, 1, 3),
                 (default_joint_pose, 1, 3),
                 (third_target, 1, 4),
                 (last_target, 1, 4)]

        for loop_index in range(LOOPS):
            for position_index, step in enumerate(poses):
                joint_position, precision, duration = step
                report.execute(self.move_and_compare_without_moveit,
                               "Move number {}.{}".format(loop_index, position_index),
                               args=[joint_position, precision, duration])

    def test_spiral(self, report):
        if self.__hardware_version in ['ned2']:
            self.__robot.led_ring.rainbow_cycle()
            self.say("Test des spirales")

        for loop_index in range(SPIRAL_LOOPS):
            report.execute(self.__robot.move_pose, "Loop {} - Move to spiral center".format(loop_index),
                           [0.3, 0, 0.2, 0, 1.57, 0])
            report.execute(self.__robot.move_spiral, "Loop {} - Execute spiral".format(loop_index),
                           [0.15, 5, 216, 3])

    def test_fun_poses(self, report):
        if self.__hardware_version in ['ned2']:
            self.__robot.led_ring.rainbow_cycle()
            self.say("Test de divers movements")

        waypoints = [[0.16, 0.00, -0.75, -0.56, 0.60, -2.26],
                     [2.25, -0.25, -0.90, 1.54, -1.70, 1.70],
                     [1.40, 0.35, -0.34, -1.24, -1.23, -0.10],
                     [0.00, 0.60, 0.46, -1.55, -0.15, 2.50],
                     [-1.0, 0.00, -1.00, -1.70, -1.35, -0.14]]

        for loop_index in range(LOOPS):
            for wayoint_index, wayoint in enumerate(waypoints):
                report.execute(self.move_and_compare_without_moveit,
                               "Loop {}.{} - Fun move".format(loop_index, wayoint_index),
                               args=[wayoint, 1, 4])

    def test_pick_and_place(self, report):
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        if self.__hardware_version in ['ned2']:
            self.say("Changez d'outil")
            self.__robot.led_ring.flashing(YELLOW)
            self.wait_custom_button_press(timeout=120)
            self.__robot.led_ring.solid(YELLOW)
            self.say("Test de pick and place")

        report.execute(self.__robot.update_tool, "Scan tool")
        report.append("Detected tool: {}".format(self.__robot.get_current_tool_id()))

        self.__robot.enable_tcp(True)

        z_offset = 0.15 if self.__robot.get_current_tool_id() <= 0 else 0.02
        sleep_pose = [0.25, 0, 0.3, 0, 1.57, 0]
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

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        self.__robot.enable_tcp(False)

    def end_test(self, report):
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        if self.__hardware_version in ['ned2']:
            self.__robot.led_ring.flashing(BLUE)
            report.append("End")
            self.say("Fin du test, validez la position 0")
            report.execute(self.wait_save_button_press, "Wait save button press to validate", args=[600, ])

        self.__robot.led_ring.solid(BLUE)
        self.__robot.move_to_sleep_pose()
        self.__robot.set_arm_max_velocity(100)
        self.__robot.set_arm_max_acceleration(100)

    def wait_custom_button_press(self, timeout=20):
        if not USE_BUTTON:
            # raw_input('Enter to continue')
            return 1, "Press custom button step skipped"

        action = self.__robot.custom_button.wait_for_any_action(timeout=timeout)

        if action == ButtonAction.NO_ACTION:
            return -1, "Timeout: no press detected"

        return 1, "Press detected"

    @staticmethod
    def wait_save_button_press(timeout=20):
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
                raise TestFailure(
                    "Target not reached - Actual {} - Target {}".format(self.__robot.get_joints(), target))
            rospy.sleep(0.1)
        return 1, "Success"


if __name__ == '__main__':
    rospy.init_node('niryo_test_production_ros_wrapper')
    robot = NiryoRosWrapper()
    if robot.get_learning_mode():
        LOOPS = 5
        CALIBRATION_LOOPS = 2
        SPIRAL_LOOPS = 5
        USE_BUTTON = True
        SPEED = 80  # %
        ACCELERATION = 50  # %

        robot.sound.play('ready.wav')

        print("----- START -----")
        test = TestProduction()
        test.run()
        print("----- END -----")
        test.print_report()

        if test.get_report()['success']:
            try:
                set_setting = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
                set_setting('sharing_allowed', 'True', 'bool')
                set_setting('test_report_done', 'True', 'bool')
                try:
                    robot.system_api_client.set_ethernet_auto()
                    rospy.sleep(7)

                    test.send_report()
                except Exception as _e:
                    pass
                rospy.sleep(3)
                set_setting('sharing_allowed', 'False', 'bool')
            except Exception as _e:
                pass

            robot.system_api_client.set_ethernet_static()
        else:
            raise TestFailure('Test failure')
    else:
        test = TestProduction(full=False)
        if not test.run():
            raise TestFailure('Program failure')
