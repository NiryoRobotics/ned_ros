import json
import logging
from contextlib import contextmanager
from io import StringIO
from typing import List, Type

import rospy
from niryo_robot_python_ros_wrapper import NiryoRosWrapper

from std_msgs.msg import String

from .BaseTest import BaseTest
from .utils import say


class TestReport(object):
    """
    This class handle the execution of the tests and the generation of the resulting test report
    """

    def __init__(self, robot: NiryoRosWrapper):
        self.__robot = robot
        self.__report = []

    @property
    def json(self):
        whole_report = {'success': self.success, 'details': self.__report}
        return json.dumps(whole_report, indent=4)

    @property
    def success(self) -> bool:
        """
        Check if all the tests were successful
        :return:
        """
        return all([test['status'] == 1 for test in self.__report])

    @contextmanager
    def __capture_logs(self):
        """
        Utility function which captures the logs emitted on the rosout logger for the time of the context
        """
        log_buffer = StringIO()
        rospy_logger = logging.getLogger("rosout")

        stringio_handler = logging.StreamHandler(log_buffer)
        stringio_handler.setLevel(logging.INFO)
        stringio_handler.setFormatter(rospy_logger.handlers[0].formatter)

        rospy_logger.addHandler(stringio_handler)

        try:
            yield log_buffer
        finally:
            rospy_logger.removeHandler(stringio_handler)
            log_buffer.close()

    def run_test(self, test: Type[BaseTest]):
        """
        Run a test and add the result to the report
        """
        test_inst = test(self.__robot)
        say(self.__robot, test_inst.name)
        with self.__capture_logs() as log_buffer:
            try:
                test_inst.pre_test()
                test_inst()
                success = True
            except Exception as e:
                rospy.logerr(f'Test {test_inst.name} failed with exception: {e}')
                say(self.__robot, f'{test_inst.name} a échoué')
                success = False
            finally:
                test_inst.post_test()
                # Reset the state of the robot between each test
                self.__robot.clear_collision_detected()

            logs = log_buffer.getvalue()
        self.__report.append({'name': test_inst.name, 'status': int(success), 'report': logs})

    def run_playbook(self, playbook: List[Type[BaseTest]]):
        """
        Run a series of tests
        """
        for test in playbook:
            self.run_test(test)

    def send(self):
        new_report_publisher = rospy.Publisher('/niryo_robot_reports/test_report', String, queue_size=1)

        # Wait for the publisher initialization
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and new_report_publisher.get_num_connections() == 0:
            if (rospy.Time.now() - start_time).to_sec() > 1:
                raise TimeoutError("Timeout while waiting for the publisher to initialize")
            rospy.sleep(0.1)

        new_report_publisher.publish(self.json)
