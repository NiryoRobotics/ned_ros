import rospy
import json
import argparse
import numpy as np
from datetime import datetime

from std_msgs.msg import String
from niryo_robot_database.srv import SetSettings

from niryo_robot_rpi.srv import LedBlinker, LedBlinkerRequest
from niryo_robot_reports.srv import CheckConnection
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, NiryoRosWrapperException

LOOPS = 5
CALIBRATION_LOOPS = 2
SPIRAL_LOOPS = 5

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
        print(new_line)
        self._report += new_line + "\n"

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

    def __init__(self):
        self.__functions = TestFunctions()
        self.__success = False

        self.__sub_tests = [
            TestStep(self.__functions.test_cloud_connection, "Test robot connection", critical=True),
            TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
            TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
            TestStep(self.__functions.test_joint_limits, "Test move", critical=True),
            TestStep(self.__functions.test_spiral, "Test move", critical=True),
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

        # Wait for the publisher initialization
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and new_report_publisher.get_num_connections() == 0:
            if (rospy.Time.now() - start_time).to_sec() > 1:
                rospy.logerr('Unable to publish the new report')
                return
            rospy.sleep(0.1)

        new_report_publisher.publish(json.dumps(self.get_report()))
        rospy.logdebug('test report published')


class TestFunctions(object):

    def __init__(self):
        rospy.sleep(2)
        self.__robot = NiryoRosWrapper()
        self.__robot.set_arm_max_velocity(SPEED)
        self.__robot.set_arm_max_acceleration(ACCELERATION)
        self.__set_led_state_service = rospy.ServiceProxy('/niryo_robot_rpi/set_led_custom_blinker', LedBlinker)
        self.led_stop()

    def led_error(self, duration=360):
        self.__set_led_state_service(True, 5, LedBlinkerRequest.LED_WHITE, duration)

    def led_stop(self):
        self.__set_led_state_service(False, 0, 0, 0)

    def test_cloud_connection(self, report):
        check_connection_service = rospy.ServiceProxy('/niryo_robot_reports/check_connection', CheckConnection)
        result = check_connection_service('test_reports')
        if result.status < 0:
            error_str = "Service test_report doesn't exists"
            report.append(error_str)
            raise TestFailure("Service test_report doesn't exists")
        if result.success is False:
            error_str = "Service test_reports didnt respond"
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
            report.append("Hardware status Error - {}".format(hardware_status.error_message))
            raise TestFailure

        if any(hardware_status.hardware_errors):
            report.append("Hardware status Motor Error - {}".format(hardware_status.hardware_errors_message))
            raise TestFailure

        if hardware_status.rpi_temperature > 70:
            report.append("Rpi overheating")
            raise TestFailure

    def test_calibration(self, report):
        for loop_index in range(CALIBRATION_LOOPS):
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

    def test_fun_poses(self, report):
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
        self.__robot.move_to_sleep_pose()
        self.__robot.set_learning_mode(True)
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
    print("----- START -----")
    test = TestProduction()
    test.run()
    print("----- END -----")
    test.print_report()
    test.send_report()

    # This is to avoid deactivating the sharing each time we run this script
    parser = argparse.ArgumentParser()
    parser.add_argument('--set_sharing_allowed', action='store_true')
    args = parser.parse_args()
    if args.set_sharing_allowed:
        set_setting = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
        set_setting('sharing_allowed', 'False', 'bool')
