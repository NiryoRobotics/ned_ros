#!/usr/bin/env python3

import roslaunch
import rospy
import rospkg
from threading import Thread
import logging

from niryo_robot_status.msg import RobotStatus

# thread safe ros logger
ros_logger = logging.getLogger('rosout')


class ProcessListener(roslaunch.pmon.ProcessListener):
    """
    A simple listener that calls a callback function when the listened process died.
    """

    def __init__(self):
        self.__on_death_callback = None

    @property
    def on_death(self):
        return self.__on_death_callback

    @on_death.setter
    def on_death(self, on_death_callback):
        self.__on_death_callback = on_death_callback

    def process_died(self, name, exit_code):
        # Run the callback in a separate thread to avoid any deadlocks.
        Thread(target=self.__on_death_callback, args=[name, exit_code]).start()


class FoxgloveSpawner:
    """
    Manages the lifecycle of the Foxglove bridge.
    """

    def __init__(self):
        ros_packages = rospkg.RosPack()
        self.__launchfile_path = ros_packages.get_path('foxglove_bridge') + '/launch/foxglove_bridge.launch'
        self.__rosargs = {'send_buffer_limit': 200000}
        self.__launch = None
        self.__process_listener = ProcessListener()
        self.__process_listener.on_death = self.__on_death

    def start(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        rosargs = [f'{k}:={v}' for k, v in self.__rosargs.items()]
        self.__launch = roslaunch.parent.ROSLaunchParent(
            uuid,
            [(self.__launchfile_path, rosargs)],
            process_listeners=[self.__process_listener],
        )
        self.__launch.start()

    def stop(self):
        try:
            self.__launch.shutdown()
        except RuntimeError as runtime_error:
            rospy.logerr(f'Failed to stop foxglove bridge: {runtime_error}')

    def __on_death(self, name, exit_code):
        ros_logger.error(f'{name} died with exit code {exit_code}. Respawning a new foxglove bridge instance...')
        self.stop()
        self.start()


if __name__ == '__main__':
    rospy.init_node('foxglove_spawner')

    robot_status = RobotStatus.BOOTING

    while robot_status == RobotStatus.BOOTING and not rospy.is_shutdown():
        robot_status_msg = rospy.wait_for_message('/niryo_robot_status/robot_status', RobotStatus)
        robot_status = robot_status_msg.robot_status

    rospy.loginfo('Starting foxglove bridge...')
    foxglove_spawner = FoxgloveSpawner()
    foxglove_spawner.start()

    rospy.spin()
