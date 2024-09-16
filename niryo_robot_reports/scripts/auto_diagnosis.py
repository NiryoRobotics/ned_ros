#!/usr/bin/env python

import rospy
import json
import numpy as np
from datetime import datetime

from niryo_robot_reports.msg import Service

from niryo_robot_rpi.srv import LedBlinker, LedBlinkerRequest
from niryo_robot_reports.srv import CheckConnection
from niryo_robot_rpi.srv import ScanI2CBus

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, NiryoRosWrapperException

LOOPS = 1
CALIBRATION_LOOPS = 2
SPIRAL_LOOPS = 1
SPEED = 80  # %
ACCELERATION = 50  # %


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
        rospy.loginfo('[Test Report]' + new_line)
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

    def run(self, *_args, **_kwargs):
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

    def __init__(self):
        self.__functions = TestFunctions()
        self.__success = False

        self.__sub_tests = [
            TestStep(self.__functions.test_cloud_connection, "Test robot connection", critical=True),
            TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
            TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
            TestStep(self.__functions.test_joint_limits, "Test move", critical=True),
            TestStep(self.__functions.test_spiral, "Test move", critical=True),
            TestStep(self.__functions.end_test, "Test final check", critical=True),
        ]

    def run(self):
        rospy.sleep(1)
        self.__functions.led_stop()
        try:
            for sub_test in self.__sub_tests:
                sub_test.run()

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
        return {"script": [sub_test.get_report() for sub_test in self.__sub_tests], "success": self.__success}

    def print_report(self):
        print(json.dumps(self.get_report(), sort_keys=True))


class TestFunctions(object):

    def __init__(self):
        rospy.sleep(2)
        self.__robot = robot
        self.__robot_version = self.__robot.get_hardware_version()
        self.__robot.set_arm_max_velocity(SPEED)
        self.__robot.set_arm_max_acceleration(ACCELERATION)

        if self.__robot_version in ['ned2', 'ned3pro']:
            rospy.sleep(2)
            self.__robot.led_ring.rainbow()
            # self.__robot.sound.set_volume(100)
            # self.__robot.sound.say('Initialization of the auto-diagnosis')
            rospy.sleep(3)
        else:
            self.led_stop()

    def reset(self):
        self.__robot.set_arm_max_velocity(100)
        self.__robot.set_arm_max_acceleration(100)

    def led_error(self, duration=360):
        if self.__robot_version in ['one', 'ned'] and not self.__robot.get_simulation_mode():
            try:
                led_serv = rospy.ServiceProxy('/niryo_robot_rpi/set_led_custom_blinker', LedBlinker)
                led_serv(True, 5, LedBlinkerRequest.LED_WHITE, duration)
            except rospy.ROSException:
                pass

    def led_stop(self):
        if self.__robot_version in ['one', 'ned'] and not self.__robot.get_simulation_mode():
            try:
                led_serv = rospy.ServiceProxy('/niryo_robot_rpi/set_led_custom_blinker', LedBlinker)
                led_serv(False, 0, 0, 0)
            except rospy.ROSException:
                pass

    def test_cloud_connection(self, report):
        if self.__robot.get_simulation_mode():
            report.append("Test Cloud connection - Skipped in simulation mode")
            return

        check_connection_service = rospy.ServiceProxy('/niryo_robot_reports/check_connection', CheckConnection)
        result = check_connection_service(Service(Service.AUTO_DIAGNOSIS_REPORTS))
        if result.status < 0:
            error_str = "Service auto_diagnosis_reports doesn't exists"
            report.append(error_str)
            raise TestFailure("Service auto_diagnosis_reports doesn't exists")
        if result.result is False:
            error_str = "auto_diagnosis_reports api didnt respond"
            report.append(error_str)
            raise TestFailure(error_str)
        report.append('Service auto_diagnosis_reports successfully reached')

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

        if self.__robot_version in ['ned2', 'ned3pro']:
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

    def test_calibration(self, report):
        for _ in range(CALIBRATION_LOOPS):
            self.__robot.request_new_calibration()
            rospy.sleep(0.1)

            report.execute(self.__robot.calibrate_auto, "Calibration")
            self.__robot.set_learning_mode(False)
            rospy.sleep(0.1)
            report.execute(self.move_and_compare, "Move after calibration", args=[6 * [0], 1])
            self.__robot.move_to_sleep_pose()

    def test_joint_limits(self, report):
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

        poses = [default_joint_pose, first_target,
                 default_joint_pose, second_target,
                 default_joint_pose, third_target, last_target]

        for loop_index in range(LOOPS):
            for position_index, joint_position in enumerate(poses):
                report.execute(self.move_and_compare_without_moveit,
                               "Move number {}.{}".format(loop_index, position_index),
                               args=[joint_position, 1, 4])

    def test_spiral(self, report):
        for loop_index in range(SPIRAL_LOOPS):
            report.execute(self.__robot.move_pose, "Loop {} - Move to spiral center".format(loop_index),
                           [0.3, 0, 0.2, 0, 1.57, 0])
            report.execute(self.__robot.move_spiral, "Loop {} - Execute spiral".format(loop_index),
                           [0.15, 5, 216, 3])

    def end_test(self, report):
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])
        self.__robot.move_to_sleep_pose()
        self.__robot.set_learning_mode(self.__robot_version in ['ned', 'one'])
        self.__robot.set_arm_max_velocity(100)
        self.__robot.set_arm_max_acceleration(100)

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
    test = TestProduction()
    test.run()
    test.print_report()
