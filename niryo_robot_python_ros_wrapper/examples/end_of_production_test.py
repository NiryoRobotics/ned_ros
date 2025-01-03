import argparse
import math
import subprocess
from io import BytesIO
from pathlib import Path
from typing import List, Type
import rospy

from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_utils import NiryoRosWrapperException
from niryo_robot_utils.dataclasses.JointsPosition import JointsPosition
from niryo_robot_utils.dataclasses.Pose import Pose
from niryo_robot_utils.end_of_production_tests.LedRingManager import LedRingManager
from niryo_robot_utils.end_of_production_tests.TestReport import TestReport
from niryo_robot_utils.end_of_production_tests import utils

#############
# Constants #
#############

WAIT_FOR_BUTTON_TIMEOUT = 600
DEFAULT_VELOCITY = 100
DEFAULT_ACCELERATION = 100

MAX_ACCELERATION = 100
LONG_TEST_DURATION = utils.parse_duration('2h30m')
CHANGE_TOOL_POSITION = JointsPosition(0, 0, 0, 0, 0, 0)

################################
# HW type dependant constants #
################################


def get_max_velocity(robot: NiryoRosWrapper) -> int:
    return {
        'ned2': 150,
        'ned3pro': 105,
    }[robot.get_hardware_version()]


###################
# Tests functions #
###################


class TestConnectionRFM(utils.BaseTest):

    def __call__(self):
        from niryo_robot_reports.srv import CheckConnection, CheckConnectionRequest
        from niryo_robot_reports.msg import Service
        check_connection_service = rospy.ServiceProxy('/niryo_robot_reports/check_connection', CheckConnection)
        request = CheckConnectionRequest(service=Service(to_test=Service.TEST_REPORTS))
        response = check_connection_service(request)
        if not response.success:
            raise RuntimeError("Can't connect to rfm")


class TestElbow90Degrés(utils.BaseTest):

    def __call__(self):
        self._robot.move(JointsPosition(0, 0, 0, 0, 0, 0))
        self._wait_for_save_button()


class TestHautsParleurs(utils.BaseTest):

    def __call__(self):
        from gtts import gTTS

        for channel, channel_name in {0: 'gauche', 1: 'droit'}.items():
            with BytesIO() as sound_file:
                tts = gTTS(f'Test haut-parleurs {channel_name}', lang='fr')
                tts.write_to_fp(sound_file)
                sound_file.seek(0)
                process = subprocess.run(['ffplay', '-nodisp', '-autoexit', '-volume', '100', '-i', '-'],
                                         input=sound_file.read(),
                                         capture_output=True)
                if process.returncode != 0:
                    raise RuntimeError(f'Error while playing sound: {process.stderr}')

        self._wait_for_save_button()


class TestLedRing(utils.BaseTest):

    def __call__(self, *args, **kwargs):
        led_ring_manager = LedRingManager(self._robot)
        led_ring_manager.run()


class TestGrippeur(utils.BaseTest):

    def pre_test(self):
        self._robot.move(CHANGE_TOOL_POSITION)
        self._wait_for_custom_button('Mettez un grippeur puis appuyez sur le bouton custom pour continuer')
        self._robot.update_tool()

    def __call__(self, *args, **kwargs):
        neutral_pose = Pose(x=0.3, y=0, z=0.3, roll=math.pi, pitch=0.00, yaw=0.00)
        pick_1 = Pose(x=0, y=0.2, z=0.15, roll=math.pi, pitch=-0.00, yaw=0.00)
        pick_2 = Pose(x=0, y=-0.2, z=0.15, roll=math.pi, pitch=-0.00, yaw=0.00)
        place_1 = Pose(x=0.15, y=0, z=0.15, roll=math.pi, pitch=-0.00, yaw=0.00)
        place_2 = Pose(x=0.22, y=0, z=0.15, roll=math.pi, pitch=-0.00, yaw=0.00)
        self._robot.move(neutral_pose)
        self._robot.pick_and_place(pick_pose=pick_1, place_pose=place_1)
        self._robot.move(neutral_pose)
        self._robot.pick_and_place(pick_pose=pick_2, place_pose=place_2)
        self._robot.move(neutral_pose)


class TestHauteVitesse(utils.BaseTest):

    def pre_test(self):
        self._robot.move(CHANGE_TOOL_POSITION)
        self._wait_for_custom_button('Mettez la masse puis appuyez sur le bouton custom pour continuer')

        self._robot.set_arm_max_velocity(get_max_velocity(self._robot))
        self._robot.set_arm_max_acceleration(MAX_ACCELERATION)

    def __call__(self, *args, **kwargs):
        joints_limits = self._get_joints_limits()

        poses = [
            JointsPosition(joints_limits[1]['min'], joints_limits[2]['min'], joints_limits[3]['max'], 0, 0, 0),
            JointsPosition(joints_limits[1]['max'], joints_limits[2]['min'], joints_limits[3]['max'], 0, 0, 0),
            JointsPosition(0, joints_limits[2]['min'], joints_limits[3]['max'], 0, 0, 0),
            JointsPosition(0, joints_limits[2]['max'], joints_limits[3]['max'], 0, 0, 0),
            JointsPosition(0, joints_limits[2]['max'], joints_limits[3]['min'], 0, 0, 0),
            JointsPosition(0, joints_limits[2]['max'], joints_limits[3]['max'], 0, 0, 0),
            JointsPosition(0, 0, 0, joints_limits[4]['min'], joints_limits[5]['min'], joints_limits[6]['min']),
            JointsPosition(0, 0, 0, joints_limits[4]['max'], joints_limits[5]['max'], joints_limits[6]['max']),
        ]
        for _ in range(10):
            for pose in poses:
                status, message = self._robot.move(pose)
                if status < 0:
                    raise RuntimeError(f'Error while moving to the pose {pose}: {message}')
                if self._robot.collision_detected:
                    raise RuntimeError(f'A collision has been detected while moving to the pose {pose}')

    def post_test(self):
        self._robot.clear_collision_detected()
        self._robot.set_arm_max_velocity(DEFAULT_VELOCITY)
        self._robot.set_arm_max_acceleration(DEFAULT_ACCELERATION)

        self._robot.move(CHANGE_TOOL_POSITION)
        self._wait_for_freemotion_button('Retirez la masse puis appuyez sur le bouton freemotion pour continuer')


class TestCalibration(utils.BaseTest):

    def __ned2_calibration(self):
        self._robot.calibrate_auto()

    def __ned3pro_calibration(self):
        from joints_interface.srv import FactoryCalibration, FactoryCalibrationRequest
        factory_calibrate_service = rospy.ServiceProxy('/niryo_self._robot/joints_interface/factory_calibrate_motors',
                                                       FactoryCalibration)
        request = FactoryCalibrationRequest(command=FactoryCalibrationRequest.START, ids=[2, 3, 4])
        factory_calibrate_service(request)

        self._wait_for_custom_button('Calibrez le robot puis appuyez sur le bouton custom pour continuer')

        request.command = FactoryCalibrationRequest.STOP
        response = factory_calibrate_service(request)
        if response.status < 0:
            self._say('La calibration a échoué')
            rospy.logerr(f'Calibration failed with error: {response.message}')
            self.__ned3pro_calibration()

    def __call__(self, *args, **kwargs):
        hardware_version = self._robot.get_hardware_version()
        if hardware_version == 'ned2':
            self.__ned2_calibration()
        elif hardware_version == 'ned3pro':
            self.__ned3pro_calibration()
        else:
            raise RuntimeError(f'Unknown hardware version: {hardware_version}')


class TestLongueDurée(utils.BaseTest):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__nb_collision = 0
        self.__nb_loops = 0

    def pre_test(self):
        self._robot.move_to_sleep_pose()

    def __handle_move_exception(self, e: NiryoRosWrapperException):
        if not self._robot.collision_detected:
            raise e from None

        self.__nb_collision += 1
        if self.__nb_collision >= 3:
            self._robot.led_ring.flashing([255, 255, 0])
        elif self.__nb_collision >= 5:
            raise RuntimeError(f'More than 5 collisions. Ending the test after {self.__nb_loops} loop(s)')

        self._robot.clear_collision_detected()

    def __call__(self, *args, **kwargs):
        joints_limits = self._get_joints_limits()
        waypoints = [
            JointsPosition(
                joints_limits[1]['min'],
                joints_limits[2]['max'],
                -0.96,
                joints_limits[4]['max'],
                joints_limits[5]['min'],
                joints_limits[6]['max'],
            ),
            JointsPosition(joints_limits[1]['max'],
                           joints_limits[2]['max'],
                           joints_limits[3]['min'],
                           joints_limits[4]['min'],
                           joints_limits[5]['max'],
                           joints_limits[6]['min']),
            JointsPosition(joints_limits[1]['max'],
                           joints_limits[2]['max'],
                           joints_limits[3]['max'],
                           joints_limits[4]['min'],
                           joints_limits[5]['max'],
                           joints_limits[6]['min']),
            JointsPosition(joints_limits[1]['min'],
                           joints_limits[2]['max'],
                           joints_limits[3]['max'],
                           joints_limits[4]['max'],
                           joints_limits[5]['min'],
                           joints_limits[6]['max']),
            JointsPosition(joints_limits[1]['min'],
                           joints_limits[2]['min'],
                           joints_limits[3]['max'],
                           joints_limits[4]['max'],
                           joints_limits[5]['max'],
                           joints_limits[6]['max']),
            JointsPosition(joints_limits[1]['max'],
                           joints_limits[2]['min'],
                           joints_limits[3]['max'],
                           joints_limits[4]['min'],
                           joints_limits[5]['min'],
                           joints_limits[6]['min']),
            JointsPosition(0, 0.5, -1.25, 0, 0, 0),
        ]

        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - start_time < LONG_TEST_DURATION:
            self.__nb_loops += 1
            for waypoint in waypoints:
                try:
                    status, message = self._robot.move(waypoint)
                    if status < 0:
                        raise RuntimeError(f'Error while moving to the pose {waypoint}: {message}')
                except NiryoRosWrapperException as e:
                    self.__handle_move_exception(e)

        rospy.loginfo(f'End of the test with {self.__nb_loops} loop and {self.__nb_collision} collision(s)')

    def post_test(self):
        self._robot.clear_collision_detected()
        self._robot.move_to_sleep_pose()


class TestStatusDuRobot(utils.BaseTest):

    def __call__(self, *args, **kwargs):
        try:
            hardware_status = self._robot.get_hardware_status()
        except rospy.exceptions.ROSException as e:
            raise RuntimeError(e) from None

        if hardware_status.error_message:
            raise RuntimeError(f'Hardware status error: {hardware_status.error_message}')

        if any(hardware_status.hardware_errors):
            raise RuntimeError(f'Hardware status Motor Error - {hardware_status.hardware_errors_message}')

        if hardware_status.rpi_temperature > 70:
            raise RuntimeError(f'Rpi temperature is over 70°C: {hardware_status.rpi_temperature}°C')

        rospy.loginfo("Hardware status ok")

        try:
            self._robot_status = self._robot.get_robot_status()
        except rospy.exceptions.ROSException as e:
            raise RuntimeError(e) from None

        if self._robot_status.robot_status < 0:
            raise RuntimeError(
                f'self._robot status - {self._robot_status.robot_status_str} - {self._robot_status.robot_message}')

        rospy.loginfo("self._robot status ok")


########
# main #
########


def parse_args():
    parser = argparse.ArgumentParser(description='Run end of production tests')
    parser.add_argument('--no-sound', action='store_true', help='Disable sound')
    parser.add_argument('--long-test-duration',
                        type=utils.parse_duration,
                        default=LONG_TEST_DURATION,
                        help='Duration of the long test. format: {number}{unit}, e.g., 3h')
    parser.add_argument('--blacklist', '-b', nargs='+', help='Blacklist of tests to skip', default=[])
    parser.add_argument('--whitelist', '-w', nargs='+', help='Whitelist of tests to run', default=None)
    return parser.parse_args()


def auto_destroy(robot: NiryoRosWrapper) -> None:
    """
    Set the demo program as autorun and delete this file
    """
    from niryo_robot_programs_manager_v2.srv import SetProgramAutorun, SetProgramAutorunRequest

    set_autorun_service = rospy.ServiceProxy('/niryo_robot_programs_manager_v2/set_program_autorun', SetProgramAutorun)
    resp = set_autorun_service(SetProgramAutorunRequest(program_id='demo', mode=SetProgramAutorunRequest.ONE_SHOT))

    if resp.status < 0:
        utils.say(robot, "Erreur lors du changement de l'autorun")
        raise RuntimeError(f"Error while setting autorun: {resp.message}")

    self_file_path = Path(__file__)
    self_file_path.unlink()


def main(playbook: List[Type[utils.BaseTest]]):
    rospy.init_node('end_of_production_tests', anonymous=True)

    robot_instance = NiryoRosWrapper()

    test_report = TestReport(robot_instance)
    test_report.run_playbook(playbook)
    test_report.send()

    print(test_report.json)

    utils.say(robot_instance, 'Fin du test.')

    if test_report.success:
        utils.say(robot_instance, 'Remplacement du programme par la démo')
        auto_destroy(robot_instance)


#############
# Playbooks #
#############

default_playbook = [
    TestCalibration,
    TestLedRing,
    TestConnectionRFM,
    TestStatusDuRobot,
    TestElbow90Degrés,
    TestHautsParleurs,
    TestGrippeur,
    TestHauteVitesse,
    TestLongueDurée,
]

if __name__ == '__main__':
    cli_args = parse_args()
    if cli_args.no_sound:
        utils.use_sound = False

    LONG_TEST_DURATION = cli_args.long_test_duration

    playbook = [test for test in default_playbook if test.__name__ not in cli_args.blacklist]
    playbook = [test for test in playbook if test.__name__ in cli_args.whitelist] if cli_args.whitelist else playbook

    main(playbook)
