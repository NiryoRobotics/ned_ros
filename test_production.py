from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
import rospy
from end_effector_interface.msg import EEButtonStatus
from std_msgs.msg import Int32
import numpy as np
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import ButtonAction

rospy.init_node('niryo_robot_python_test_production')

print "--- Start"

WHITE = [255, 255, 255]
GREEN = [0, 255, 0]
BLACK = [0, 0, 0]
BLUE = [0, 0, 255]
PURPLE = [100, 0, 255]
PINK = [255, 0, 255]

def almost_equal_array(a, b, decimal=1):
    try:
        np.testing.assert_almost_equal(a, b, decimal)
        return True
    except AssertionError:
        return False


class TestProduction(object):

    def __init__(self):
        self.__robot = NiryoRosWrapper()
        self.run()

    def run(self):
        self.test_calibration()
        self.test_led_ring()
        self.test_sound()
        self.test_freedrive()
        self.test_io()
        self.test_move()

    def test_calibration(self):
        self.__robot.request_new_calibration()
        rospy.sleep(0.1)
        self.__robot.calibrate_auto()
        self.move_and_compare(6 * [0], 1)
        self.__robot.led_ring.flashing(BLUE)
        self.wait_save_button_press()

    def test_led_ring(self):
        self.__robot.led_ring.solid(WHITE)
        self.wait_custom_button_press()
        self.__robot.led_ring.rainbow_cycle()
        rospy.sleep(0.5)
        self.wait_custom_button_press()

    def test_sound(self):
        self.__robot.led_ring.solid(PURPLE)
        self.__robot.led_ring.set_volume(100)
        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/calibration_sound")
        self.__robot.led_ring.play(sound_name, wait_end=True)
        rospy.sleep(0.5)
        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/connection_sound")
        self.__robot.led_ring.play(sound_name, wait_end=True)
        rospy.sleep(0.5)
        sound_name = rospy.get_param("/niryo_robot_sound/robot_sounds/robot_ready_sound")
        self.__robot.led_ring.play(sound_name, wait_end=True)
        self.__robot.led_ring.flashing(PURPLE)
        self.wait_custom_button_press()

    def test_freedrive(self):
        self.__robot.led_ring.solid(GREEN)

        while not self.__robot.get_learning_mode():
            rospy.sleep(0.1)

        for i in range(0, 30, 3):
            self.__robot.led_ring.set_led(i, BLACK)

        joint_limit = self.__robot.get_axis_limits()
        while not self.__robot.get_joints()[0] < joint_limit['joint_1']['min'] + 0.2:
            rospy.sleep(0.1)

        for i in range(1, 30, 3):
            self.__robot.led_ring.set_led(i, BLACK)

        while not self.__robot.get_joints()[0] < joint_limit['joint_1']['max'] + 0.2:
            rospy.sleep(0.1)

        for i in range(2, 30, 3):
            self.__robot.led_ring.set_led(i, BLACK)

        rospy.sleep(0.5)
        self.__robot.led_ring.flashing(GREEN)
        self.wait_custom_button_press()

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

    def test_move(self):
        pass

    def wait_custom_button_press(self):
        print("Please press custom button to continue")
        action = self.__robot.__custom_button.wait_for_any_action(timeout=20)

        if action == ButtonAction.NO_ACTION:
            print("Timeout: no press detected, test aborted")
            return False

        return True


    @staticmethod
    def wait_save_button_press():
        print("Please save custom button to continue")
        try:
            rospy.wait_for_message("/niryo_robot/blockly/save_current_point", Int32, timeout=20)
            return True
        except rospy.ROSException:
            print("Timeout: no press detected, test aborted")
            return False

    def move_and_compare(self, target, precision_decimal=1):
        self.__robot.move_joints(*target)
        rospy.sleep(0.1)
        return almost_equal_array(self.__robot.get_joints(), target, decimal=precision_decimal)


TestProduction()
