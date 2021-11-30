import rospy
import json
import numpy as np
from datetime import datetime

from std_msgs.msg import Int32

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, NiryoRosWrapperException
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ButtonAction

WHITE = [255, 255, 255]
GREEN = [50, 255, 0]
BLACK = [0, 0, 0]
BLUE = [15, 50, 255]
PURPLE = [153, 51, 153]
PINK = [255, 0, 255]
RED = [255, 0, 0]

USE_BUTTON = False
VOLUME = 10
LOOPS = 2


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
            TestStep(self.__functions.test_robot_status, "Test robot status", critical=True),
            TestStep(self.__functions.test_sound, "Test sound", critical=False),
            TestStep(self.__functions.test_calibration, "Test calibration", critical=True),
            TestStep(self.__functions.test_led_ring, "Test led ring", critical=False),
            TestStep(self.__functions.test_freedrive, "Test free motion", critical=False),
            TestStep(self.__functions.test_joint_limits, "Test move", critical=True),
            TestStep(self.__functions.test_spiral, "Test move", critical=True),
            TestStep(self.__functions.test_fun_poses, "Test move", critical=True),
            TestStep(self.__functions.test_pick_and_place, "Test pick and place", critical=True),
            TestStep(self.__functions.end_test, "Test final check", critical=True),
        ]

    def run(self):
        rospy.sleep(1)
        try:
            for test in self.__sub_tests:
                test.run()

        except TestFailure:
            self.__sub_tests.append(TestStep(self.__functions.test_robot_status, "Test robot status", critical=True))
            self.__sub_tests[-1].run()
            self.__success = False
        else:
            self.__success = True

        return self.__success

    def get_report(self):
        return {"details": [test.get_report() for test in self.__sub_tests], "success": self.__success}

    def print_report(self):
        print(json.dumps(self.get_report(), indent=4, sort_keys=True))


class TestFunctions(object):

    def __init__(self):
        rospy.sleep(2)
        self.__robot = NiryoRosWrapper()

    def test_robot_status(self, report):
        try:
            robot_status = self.__robot.get_robot_status()
        except rospy.exceptions.ROSException as e:
            report.append(str(e))
            raise TestFailure(e)

        if robot_status.robot_status < 0:
            report.append("Robot status - {} - {}".format(robot_status.robot_status_str, robot_status.robot_message))
            raise TestFailure

        if robot_status.rpi_overheating:
            report.append("Rpi overheating")
            raise TestFailure

    def test_calibration(self, report):
        self.__robot.sound.say("Test de calibration", 1)

        for loop_index in range(2):
            self.__robot.request_new_calibration()
            rospy.sleep(0.1)
            report.execute(self.__robot.calibrate_auto, "Calibration")
            report.execute(self.move_and_compare, "Move after calibration", args=[6 * [0], 1])
            self.__robot.led_ring.flashing(BLUE)

            self.__robot.sound.say("Validez la position du robot", 1)
            report.execute(self.wait_save_button_press, "Wait save button press to validate")

    def test_led_ring(self, report):
        self.__robot.led_ring.solid(WHITE)

        self.__robot.sound.say("Premier test du ruban led", 1)
        report.append("Led ring color set to WHITE")
        self.__robot.sound.say("Validez le test", 1)
        report.execute(self.wait_custom_button_press, "Wait custom button press to continue", args=[60, ])

        self.__robot.led_ring.rainbow_cycle()
        report.append("Led ring color set to RAINBOW")
        self.__robot.sound.say("Second test du ruban led", 1)
        self.__robot.sound.say("Validez le test", 1)
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate", args=[60, ])

    def test_sound(self, report):

        self.__robot.led_ring.solid(PURPLE)
        report.append("Led ring color set to PURPLE")

        report.execute(self.__robot.sound.set_volume, "Set volume", [VOLUME])
        report.append("Volume set to {}%".format(VOLUME))

        self.__robot.sound.say("Test de son", 1)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/calibration_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])
        rospy.sleep(0.5)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/connection_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])
        rospy.sleep(0.5)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/robot_ready_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])

        self.__robot.led_ring.flashing(PURPLE)
        self.__robot.sound.say("Validez le test", 1)
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

    def test_freedrive(self, report):
        self.__robot.sound.say("Test de free motion", 1)

        joint_limit = self.__robot.get_axis_limits()[1]['joint_limits']

        self.__robot.led_ring.solid(GREEN)
        report.append("Led ring color set to GREEN")

        report.append("Wait learning mode")
        report.execute(self.wait_learning_mode, "Wait learning mode")

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
            self.__robot.sound.say("Limite validee", 1)
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
            self.__robot.sound.say("Limite validee", 1)
            report.append("Joint1 maximum limit reached")

        for i in range(4, 30, 6):
            for j in range(2):
                self.__robot.led_ring.set_led_color(i + j, BLACK)

        rospy.sleep(1)
        self.__robot.led_ring.flashing(GREEN)

        report.execute(self.wait_torque_on, "Wait learning mode disabled")
        rospy.sleep(1)

    def test_io(self, report):
        self.__robot.led_ring.solid(PINK)

        # Test digital ios
        dio = self.__robot.get_digital_io_state()

        # for input in dio.digital_inputs:
        #     self.__robot.digital_write(input.name, True)

        for output in dio.digital_output:
            assert self.__robot.digital_read(output.name)

        # Test analog ios
        aio = self.__robot.get_analog_io_state()
        for input in aio.analog_inputs:
            self.__robot.analog_write(input.name, 5)

        # for output in aio.analog_output:
        #     assert 4.8 <= self.__robot.analog_read(output.name) <= 5.2

        self.__robot.led_ring.flashing(PINK)
        self.wait_custom_button_press()

    def test_joint_limits(self, report):
        self.__robot.led_ring.rainbow_cycle()
        self.__robot.sound.say("Test des limites des joints", 1)

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

        poses = [default_joint_pose, first_target, second_target, third_target, last_target]

        for loop_index in range(LOOPS):
            for position_index, joint_position in enumerate(poses):
                report.execute(self.move_and_compare_without_moveit,
                               "Move number {}.{}".format(loop_index, position_index),
                               args=[joint_position, 1, 2])

    def test_spiral(self, report):
        self.__robot.led_ring.rainbow_cycle()
        self.__robot.sound.say("Test des spirales", 1)

        for loop_index in range(LOOPS):
            report.execute(self.__robot.move_pose, "Loop {} - Move to spiral center".format(loop_index),
                           [0.3, 0, 0.2, 0, 1.57, 0])
            report.execute(self.__robot.move_spiral, "Loop {} - Execute spiral".format(loop_index),
                           [0.1, 5, 216, 3])

    def test_fun_poses(self, report):
        self.__robot.led_ring.rainbow_cycle()
        self.__robot.sound.say("Test de divers movements", 1)

        waypoints = [[0.16, 0.00, -0.75, -0.56, 0.60, -2.26],
                     [2.25, -0.25, -0.90, 1.54, -1.70, 1.70],
                     [1.40, 0.35, -0.34, -1.24, -1.23, -0.10],
                     [0.00, 0.60, 0.46, -1.55, -0.15, 2.50],
                     [-1.0, 0.00, -1.00, -1.70, -1.35, -0.14]]

        for loop_index in range(LOOPS):
            for wayoint_index, wayoint in enumerate(waypoints):
                report.execute(self.move_and_compare_without_moveit,
                               "Loop {}.{} - Fun move".format(loop_index, wayoint_index),
                               args=[wayoint, 1, 2])

    def test_pick_and_place(self, report):
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        self.__robot.sound.say("Changez d'outil", 1)
        self.wait_custom_button_press()

        self.__robot.sound.say("Test de pick and place", 1)

        report.execute(self.__robot.update_tool, "Scan tool")
        report.append("Detected tool: {}".format(self.__robot.get_current_tool_id()))

        self.__robot.enable_tcp(True)

        z_offset = 0.02
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

        self.__robot.led_ring.flashing(BLUE)
        report.append("End")
        self.__robot.sound.say("Fin du test, validez la position 0", 1)
        report.execute(self.wait_save_button_press, "Wait save button press to validate")

    def wait_custom_button_press(self, timeout=20):
        if not USE_BUTTON:
            raw_input('Enter to continue')
            return 1, "Press custom button step skipped"

        action = self.__robot.__custom_button.wait_for_any_action(timeout=timeout)

        if action == ButtonAction.NO_ACTION:
            return -1, "Timeout: no press detected"

        return 1, "Press detected"

    @staticmethod
    def wait_save_button_press():
        if not USE_BUTTON:
            raw_input('Enter to continue')
            return 1, "Press save button step skipped"

        try:
            rospy.wait_for_message("/niryo_robot/blockly/save_current_point", Int32, timeout=20)
            return 1, "Press detected"
        except rospy.ROSException:

            return -1, "Timeout: no press detected"

    def wait_learning_mode(self):
        start_time = rospy.Time.now()
        while not self.__robot.get_learning_mode():
            if (rospy.Time.now() - start_time).to_sec() > 20:
                return -1, "Timeout: no learning mode detected"
            rospy.sleep(0.1)

        return 1, "Learning mode enabled"

    def wait_torque_on(self):
        start_time = rospy.Time.now()
        while self.__robot.get_learning_mode():
            if (rospy.Time.now() - start_time).to_sec() > 20:
                return -1, "Timeout: no torque on detected"
            rospy.sleep(0.1)

        return 1, "Learning mode disabled"

    def move_and_compare(self, target, precision_decimal=1):
        status, message = self.__robot.move_joints(*target)
        if status >= 0:
            current_joints = self.__robot.get_joints()
            if not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
                raise TestFailure("Target not reached - Actual {} - Target {}".format(current_joints, target))
        return status, message

    def move_and_compare_without_moveit(self, target, precision_decimal=1, duration=2):
        _status, _message = self.__robot.move_without_moveit(target, duration)

        start_time = rospy.Time.now()
        while not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
            if (rospy.Time.now() - start_time).to_sec() > 5:
                raise TestFailure(
                    "Target not reached - Actual {} - Target {}".format(self.__robot.get_joints(), target))
            rospy.sleep(0.1)
        return 1, "Success"

    def play_sound(self, sound_name):
        start_time = rospy.Time.now()
        self.__robot.sound.play(sound_name, wait_end=True)

        sound_duration = self.__robot.sound.get_sound_duration(sound_name)
        if sound_duration - 0.5 < (rospy.Time.now() - start_time).to_sec() < sound_duration + 1:
            return -1, "Sound {} runtime error".format(sound_name)

        return 1, "Success"


if __name__ == '__main__':
    rospy.init_node('niryo_test_production_ros_wrapper')
    print("----- START -----")
    test = TestProduction()
    test.run()
    print("----- END -----")
    test.print_report()
