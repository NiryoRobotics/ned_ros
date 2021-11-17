from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper, NiryoRosWrapperException
import rospy
from datetime import datetime
from end_effector_interface.msg import EEButtonStatus
from std_msgs.msg import Int32
import numpy as np
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ButtonAction

rospy.init_node('niryo_test_production_ros_wrapper')

print "--- Start"

WHITE = [255, 255, 255]
GREEN = [0, 255, 0]
BLACK = [0, 0, 0]
BLUE = [0, 0, 255]
PURPLE = [100, 0, 255]
PINK = [255, 0, 255]
RED = [255, 0, 0]
USE_BUTTON = False


def almost_equal_array(a, b, decimal=1):
    try:
        np.testing.assert_almost_equal(a, b, decimal)
        return True
    except AssertionError:
        return False


class TestReportException(Exception):
    pass


class TestFailure(Exception):
    pass


class TestReport(object):
    def __init__(self, header):
        self._header = header
        self._report = ""

        self.append("Start test report")

    def __str__(self):
        return self._report

    def __add__(self, other):
        return self._report + str(other)

    def __radd__(self, other):
        self._report += str(other)
        return self

    def append(self, message):
        new_line = "[{}] - {} - {}.".format(self._header, datetime.now(), message)
        print(new_line)
        self._report += new_line + "\n"

    def execute(self, function, prefix="", args=None):
        try:
            status, message = function() if args is None else function(*args)
        except [NiryoRosWrapperException, TestReportException] as e:
            self.append("{}{} - {}".format(prefix, " failed" if prefix else "Failed", e))
        else:
            if status >= 0:
                success_message = " succeed" if prefix else "Succeed"
            else:
                success_message = " failed" if prefix else "Failed"

            self.append("{}{} - {} - {}".format(prefix, success_message, status, message))


class TestProduction(object):

    def __init__(self):
        rospy.sleep(2)
        self.__robot = NiryoRosWrapper()
        self.run()

    def run(self):
        report = TestReport("Test Report")
        report.append("##### START ######")

        try:
            report += self.test_scan_motors()
            report += self.test_calibration()
            report += self.test_led_ring()
            report += self.test_sound()
            report += self.test_freedrive()
            # self.test_io()
            report += self.test_move()
            report += self.test_pick_and_place()
            report += self.end_test()
        except TestFailure:
            report.append("Test failed")
            self.__robot.led_ring.chase(RED)
            report.execute(self.wait_custom_button_press, "Wait custom button press to end", args=[3600, ])

        report.append("##### END ######")

    def test_scan_motors(self):
        pass

    def test_calibration(self):
        report = TestReport("Calibration Test")
        self.__robot.request_new_calibration()
        rospy.sleep(0.1)

        report.append("In progress")
        report.execute(self.__robot.calibrate_auto, "Calibration")
        report.execute(self.move_and_compare, "Move after calibration", args=[6 * [0], 1])

        self.__robot.led_ring.flashing(BLUE)
        report.append("End")
        report.execute(self.wait_save_button_press, "Wait save button press to validate")

        return report

    def test_led_ring(self):
        report = TestReport("Led Ring Test")

        self.__robot.led_ring.solid(WHITE)
        report.append("Led ring color set to WHITE")
        report.execute(self.wait_custom_button_press, "Wait custom button press to continue", args=[60, ])

        self.__robot.led_ring.rainbow_cycle()
        report.append("Led ring color set to RAINBOW")
        rospy.sleep(0.5)
        report.append("End")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate", args=[60, ])

        return report

    def test_sound(self):
        report = TestReport("Sound Test")

        self.__robot.led_ring.solid(PURPLE)
        report.append("Led ring color set to PURPLE")

        self.__robot.sound.set_volume(100)
        report.append("Volume set to 100%")

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/calibration_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])
        rospy.sleep(0.5)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/connection_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])
        rospy.sleep(0.5)

        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/robot_ready_sound")
        report.execute(self.__robot.sound.play, "Play {} sound".format(sound_name), [sound_name, True])

        self.__robot.led_ring.flashing(PURPLE)
        report.append("End")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

        return report

    def test_freedrive(self):
        report = TestReport("Freedrive Test")
        joint_limit = self.__robot.get_axis_limits()[1]['joint_limits']

        self.__robot.led_ring.solid(GREEN)
        report.append("Led ring color set to GREEN")

        report.append("Wait learning mode")
        report.execute(self.wait_learning_mode, "Wait learning mode")

        for i in range(0, 30, 3):
            self.__robot.led_ring.set_led_color(i, BLACK)

        report.append("Wait joint1 minimum limit")
        start_time = rospy.Time.now()
        while not self.__robot.get_joints()[0] < joint_limit['joint_1']['min'] + 0.2:
            if (rospy.Time.now() - start_time).to_sec() > 20:
                report.append("Joint1 minimum limit not reached")
                break
        else:
            report.append("Joint1 minimum limit reached")

        for i in range(1, 30, 3):
            self.__robot.led_ring.set_led_color(i, BLACK)

        report.append("Wait joint1 minimum limit")
        start_time = rospy.Time.now()
        while not self.__robot.get_joints()[0] > joint_limit['joint_1']['max'] - 0.2:
            if (rospy.Time.now() - start_time).to_sec() > 20:
                report.append("Joint1 maximum limit not reached")
                break
        else:
            report.append("Joint1 maximum limit reached")

        for i in range(2, 30, 3):
            self.__robot.led_ring.set_led_color(i, BLACK)

        rospy.sleep(1)
        self.__robot.led_ring.flashing(GREEN)
        report.append("End")
        report.execute(self.wait_custom_button_press, "Wait custom button press to validate")

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

    def test_move(self, loops=2):
        report = TestReport("Test joints limits")
        self.__robot.led_ring.rainbow_cycle()
        self.test_joint_limits()

        for _ in range(loops):
            self.test_fun_poses()
            self.test_spiral()
        self.wait_custom_button_press()
        return report


    def test_joint_limits(self, loops=2):
        report = TestReport("Test joints limits")

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

        for loop_index in range(loops):
            for position_index, joint_position in enumerate(poses):
                report.execute(self.move_and_compare_without_moveit,
                               "Move number {}.{}".format(loop_index, position_index),
                               args=[joint_position, 1, 2])

        return report

    def test_fun_poses(self):
        report = TestReport("Test some moves")

        waypoints = [[0.16, 0.00, -0.75, -0.56, 0.60, -2.26],
                     [2.25, -0.25, -0.90, 1.54, -1.70, 1.70],
                     [1.40, 0.35, -0.34, -1.24, -1.23, -0.10],
                     [0.00, 0.60, 0.46, -1.55, -0.15, 2.50],
                     [-1.0, 0.00, -1.00, -1.70, -1.35, -0.14]]

        for wayoint_index, wayoint in enumerate(waypoints):
            report.execute(self.move_and_compare_without_moveit,
                           "Fun move number {}".format(wayoint_index),
                           args=[wayoint, 1, 2])

        return report

    def test_spiral(self):
        report = TestReport("Test spiral move")
        report.execute(self.__robot.move_pose, "Move to spiral center", [0.3, 0, 0.2, 0, 1.57, 0])
        report.execute(self.__robot.move_spiral, "Execute spiral", [0.1, 5, 216, 3])
        return report

    def test_pick_and_place(self):
        report = TestReport("Test pick and place")

        report.execute(self.__robot.update_tool, "Scan tool")
        report.append("Detected tool: {}".format(self.__robot.get_current_tool_id()))

        self.__robot.enable_tcp(True)

        z_offset = 0.01
        sleep_pose = [0.25, 0, 0.3, 0, 1.57, 0]
        pick_1 = [0, 0.2, z_offset, 0, 1.57, 0]
        pick_2 = [0, -0.2, z_offset, 0, 1.57, 0]
        place_1 = [0.15, 0, z_offset, 0, 1.57, 0]
        place_2 = [0.2, 0, z_offset, 0, 1.57, 0]

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 1st piece", pick_1)
        report.execute(self.__robot.place_from_pose, "Place 1st piece", place_1)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 2nd piece", pick_2)
        report.execute(self.__robot.place_from_pose, "Place 2nd piece", place_2)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 1st piece from center", place_1)
        report.execute(self.__robot.place_from_pose, "Replace 1st piece", pick_1)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        report.execute(self.__robot.pick_from_pose, "Pick 2nd piece from center", place_2)
        report.execute(self.__robot.place_from_pose, "Replace 2nd piece", pick_2)

        report.execute(self.__robot.move_pose, "Move to sleep pose", sleep_pose)

        self.__robot.enable_tcp(False)

    def end_test(self):
        report = TestReport("Last Test")

        report.execute(self.move_and_compare, "Move to 0.0", args=[6 * [0], 1])

        self.__robot.led_ring.flashing(BLUE)
        report.append("End")
        report.execute(self.wait_save_button_press, "Wait save button press to validate")

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

    def move_and_compare(self, target, precision_decimal=1):
        status, message = self.__robot.move_joints(*target)
        if status >= 0:
            current_joints = self.__robot.get_joints()
            if not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
                raise TestReportException("Target not reached - Actual {} - Target {}".format(current_joints, target))
        return status, message

    def move_and_compare_without_moveit(self, target, precision_decimal=1, duration=2):
        status, message = self.__robot.move_without_moveit(target, duration)

        start_time = rospy.Time.now()
        while not almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal):
            if (rospy.Time.now() - start_time).to_sec() > 5:
                raise TestReportException(
                    "Target not reached - Actual {} - Target {}".format(self.__robot.get_joints(), target))
            rospy.sleep(0.1)
        return status, message

    def play_sound(self, sound_name):
        start_time = rospy.Time.now()
        self.__robot.sound.play(sound_name, wait_end=True)

        sound_duration = self.__robot.sound.get_sound_duration(sound_name)
        if sound_duration - 0.5 < (rospy.Time.now() - start_time).to_sec() < sound_duration + 1:
            return -1, "Sound {} runtime error".format(sound_name)

        return 1, "Success"


TestProduction()
