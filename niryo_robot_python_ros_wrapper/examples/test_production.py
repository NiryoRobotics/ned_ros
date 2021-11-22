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


def almost_equal_array(a, b, decimal=1):
    try:
        np.testing.assert_almost_equal(a, b, decimal)
        return True
    except AssertionError:
        return False


def print_dict(dict_to_print):
    print(json.dumps(dict_to_print, indent=4, sort_keys=True))


class TestReportException(Exception):
    pass


class TestFailure(Exception):
    pass


class TestReport(object):
    def __init__(self, header):
        self._header = header
        self._report = ""
        self._report_dict = {}
        self._state = "IN PROGRESS"

        self.append("Start test report")

    def __str__(self):
        return self._report

    def __add__(self, other):
        self._report += str(other)
        self._report_dict.update(other.summary(display=False))
        return self

    @property
    def state(self):
        return self._state

    @property
    def has_failed(self):
        return self._state == "FAILURE"

    def append(self, message):
        new_line = "[{}] - {} - {}.".format(self._header, datetime.now(), message)
        print(new_line)
        self._report += new_line + "\n"

    def execute(self, function, prefix="", args=None):
        try:
            status, message = function() if args is None else function(*args)
        except (
                NiryoRosWrapperException, TestReportException, TestFailure,
                rospy.exceptions.ROSInterruptException) as e:
            self.append("{}{} - {}".format(prefix, " failed" if prefix else "Failed", str(e)))
            self._state = "FAILURE"
            raise TestFailure(e)
        else:
            if status >= 0:
                success_message = " succeed" if prefix else "Succeed"
                self.append("{}{} - {} - {}".format(prefix, success_message, status, message))
            else:
                success_message = " failed" if prefix else "Failed"
                self.append("{}{} - {} - {}".format(prefix, success_message, status, message))
                self._state = "FAILURE"
                raise TestFailure(message)

    def summary(self, display=False):
        summary = {'[ ' + self._header + ' ]': self._report_dict if self._report_dict else self._state}
        if display:
            print_dict(summary)
        return summary

    @property
    def report(self):
        return self._report

    def end(self):
        self._state = "SUCCESS"

    def abort(self):
        self._state = "FAILURE"


class TestStep(object):

    def __init__(self, function, name, critical=False):
        self.__function = function
        self.__name = name
        self.__report = TestReport(name)

    def run (self, *args, **kwargs):
        self.__function(self.__report)


class TestProduction:

    def __init__(self):
        self.__functions = TestFunctions()


class TestFunctions(object):

    def __init__(self):
        rospy.sleep(2)
        self.__robot = NiryoRosWrapper()

    def _run_test_step(self, report, function, args=None):
        new_report = function(*args) if args else function()

        report += new_report
        if new_report.has_failed:
            raise TestFailure

    def run(self):
        report = TestReport("Test Report")
        report.append("##### START ######")

        try:
            self._run_test_step(report, self.test_robot_status)
            self._run_test_step(report, self.test_sound)
            self._run_test_step(report, self.test_calibration)
            self._run_test_step(report, self.test_led_ring)
            self._run_test_step(report, self.test_freedrive)
            self._run_test_step(report, self.test_move)
            self._run_test_step(report, self.test_pick_and_place)
            self._run_test_step(report, self.end_test)

        except TestFailure:
            report.append("Test failed")
            self._run_test_step(report, self.test_robot_status)
            report.abort()
            try:
                self.__robot.led_ring.chase(RED)
            except Exception:
                pass
            report.execute(self.wait_custom_button_press, "Wait custom button press to end", args=[3600, ])
        else:
            report.end()

        report.append("##### END ######")
        report.summary(display=True)
        print (report.report)

        return report

    def test_move(self, loops=2):
        self.__robot.led_ring.rainbow_cycle()
        test_move_report = TestReport("Test moves")

        try:
            self._run_test_step(test_move_report, self.test_joint_limits, [loops])
            self._run_test_step(test_move_report, self.test_spiral, [loops])
            self._run_test_step(test_move_report, self.test_fun_poses, [loops])
        except TestFailure:
            test_move_report.abort()
        else:
            test_move_report.end()

        return test_move_report

    def test_robot_status(self):
        report = TestReport("Check robot status Test")
        try:
            robot_status = self.__robot.get_robot_status()
        except rospy.exceptions.ROSException as e:
            report.append(str(e))
            report.abort()
            return report

        if robot_status.robot_status < 0:
            report.append("Robot status - {} - {}".format(robot_status.robot_status_str, robot_status.robot_message))
            report.abort()

        if robot_status.rpi_overheating:
            report.append("Rpi overheating")
            report.abort()

        if not report.has_failed:
            report.end()

        return report

    def test_calibration(self, loops=2):
        report = TestReport("Calibration Test")
        self.__robot.sound.say("Test de calibration", 1)

        try:
            for loop_index in range(loops):
                self.__robot.request_new_calibration()
                rospy.sleep(0.1)
                report.execute(self.__robot.calibrate_auto, "Calibration")
                report.execute(self.move_and_compare, "Move after calibration", args=[6 * [0], 1])
                self.__robot.led_ring.flashing(BLUE)

                self.__robot.sound.say("Validez la position du robot", 1)
                report.execute(self.wait_save_button_press, "Wait save button press to validate")
                report.end()

        except TestFailure:
            pass

        return report

    def test_led_ring(self):
        report = TestReport("Led Ring Test")
        self.__robot.led_ring.solid(WHITE)

        try:
            self.__robot.sound.say("Premier test du ruban led", 1)
            report.append("Led ring color set to WHITE")
            self.__robot.sound.say("Validez le test", 1)
            report.execute(self.wait_custom_button_press, "Wait custom button press to continue", args=[60, ])

            self.__robot.led_ring.rainbow_cycle()
            report.append("Led ring color set to RAINBOW")
            self.__robot.sound.say("Second test du ruban led", 1)
            self.__robot.sound.say("Validez le test", 1)
            report.execute(self.wait_custom_button_press, "Wait custom button press to validate", args=[60, ])
            report.end()

        except TestFailure:
            pass

        return report

    def test_sound(self):
        report = TestReport("Sound Test")

        self.__robot.led_ring.solid(PURPLE)
        report.append("Led ring color set to PURPLE")

        try:
            self.__robot.sound.set_volume(VOLUME)
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

            report.end()

        except TestFailure:
            pass

        return report

    def test_freedrive(self):
        self.__robot.sound.say("Test de free motion", 1)

        report = TestReport("Freedrive Test")
        joint_limit = self.__robot.get_axis_limits()[1]['joint_limits']

        try:
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

            report.end()

        except TestFailure:
            pass

        return report

    def test_io(self):
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

    def test_joint_limits(self, loops=2):
        report = TestReport("Move to joints limits")
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

        try:
            for loop_index in range(loops):
                for position_index, joint_position in enumerate(poses):
                    report.execute(self.move_and_compare_without_moveit,
                                   "Move number {}.{}".format(loop_index, position_index),
                                   args=[joint_position, 1, 2])

            report.end()

        except TestFailure:
            pass

        return report

    def test_spiral(self, loops):
        report = TestReport("Test spiral move")
        self.__robot.sound.say("Test des spirales", 1)

        try:
            for loop_index in range(loops):
                report.execute(self.__robot.move_pose, "Loop {} - Move to spiral center".format(loop_index),
                               [0.3, 0, 0.2, 0, 1.57, 0])
                report.execute(self.__robot.move_spiral, "Loop {} - Execute spiral".format(loop_index),
                               [0.1, 5, 216, 3])

            report.end()

        except TestFailure:
            pass

        return report

    def test_fun_poses(self, loops):
        report = TestReport("Test random moves")
        self.__robot.sound.say("Test de divers movements", 1)

        waypoints = [[0.16, 0.00, -0.75, -0.56, 0.60, -2.26],
                     [2.25, -0.25, -0.90, 1.54, -1.70, 1.70],
                     [1.40, 0.35, -0.34, -1.24, -1.23, -0.10],
                     [0.00, 0.60, 0.46, -1.55, -0.15, 2.50],
                     [-1.0, 0.00, -1.00, -1.70, -1.35, -0.14]]

        try:
            for loop_index in range(loops):
                for wayoint_index, wayoint in enumerate(waypoints):
                    report.execute(self.move_and_compare_without_moveit,
                                   "Loop {}.{} - Fun move".format(loop_index, wayoint_index),
                                   args=[wayoint, 1, 2])

            report.end()

        except TestFailure:
            pass

        return report

    def test_pick_and_place(self):
        report = TestReport("Test avec l'outil")
        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        self.__robot.sound.say("Changez d'outil", 1)
        self.wait_custom_button_press()

        self.__robot.sound.say("Test de pick and place", 1)

        try:
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

            report.end()

        except TestFailure:
            pass

        return report

    def end_test(self):
        report = TestReport("Last Test")

        try:
            report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

            self.__robot.led_ring.flashing(BLUE)
            report.append("End")
            self.__robot.sound.say("Fin du test, validez la position 0", 1)
            report.execute(self.wait_save_button_press, "Wait save button press to validate")
            report.end()
        except TestFailure:
            pass

        return report

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
                raise TestReportException("Target not reached - Actual {} - Target {}".format(current_joints, target))
        return status, message

    def move_and_compare_without_moveit(self, target, precision_decimal=1, duration=2):
        _status, _message = self.__robot.move_without_moveit(target, duration)

        start_time = rospy.Time.now()
        while not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
            if (rospy.Time.now() - start_time).to_sec() > 5:
                raise TestReportException(
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
    print "--- Start"
    test = TestProduction()
    test.run()
