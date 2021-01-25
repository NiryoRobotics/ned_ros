#!/usr/bin/env python

import rospy
from data_block import DataBlock

# Messages
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32

from conveyor_interface.msg import ConveyorFeedbackArray
from niryo_robot_msgs.msg import HardwareStatus, RobotState, SoftwareVersion
from niryo_robot_rpi.msg import LogStatus

"""
 - Each address contains a 16 bits value
 - READ ONLY registers

 --> State of the robot
"""

IR_JOINTS = 0
IR_POSITION_X = 10
IR_POSITION_Y = 11
IR_POSITION_Z = 12
IR_ORIENTATION_X = 13
IR_ORIENTATION_Y = 14
IR_ORIENTATION_Z = 15

IR_SELECTED_TOOL_ID = 200

IR_LEARNING_MODE = 300

IR_MOTORS_CONNECTION_UP = 400
IR_CALIBR_NEEDED = 401
IR_CALIBR_IN_PROGRESS = 402
IR_RPI_TEMPERATURE = 403
IR_RPI_AVAILABLE_SPACE = 404
IR_RPI_ROS_LOG_SIZE = 405
IR_RPI_VERSION_N1 = 406
IR_RPI_VERSION_N2 = 407
IR_RPI_VERSION_N3 = 408
IR_HARDWARE_VERSION = 409

IR_CONVEYOR_1_CONNECTION_STATE = 530
IR_CONVEYOR_1_CONTROL_STATUS = 531
IR_CONVEYOR_1_SPEED = 532
IR_CONVEYOR_1_DIRECTION = 533

IR_CONVEYOR_2_CONNECTION_STATE = 540
IR_CONVEYOR_2_CONTROL_STATUS = 541
IR_CONVEYOR_2_SPEED = 542
IR_CONVEYOR_2_DIRECTION = 543


def handle_negative(val):
    """
    Positive number : 0 - 32767
    Negative number : 32768 - 65535
    """
    if val < 0:
        val = (1 << 15) - val
    return val


class InputRegisterDataBlock(DataBlock):

    def __init__(self):
        super(InputRegisterDataBlock, self).__init__()
        self.joint_state_sub = None
        self.robot_state_sub = None
        self.selected_tool_id_sub = None
        self.learning_mode_sub = None
        self.hardware_status_sub = None
        self.ros_log_status_sub = None
        self.software_version_sub = None
        self.conveyor_feedback_sub = None

    def start_ros_subscribers(self):
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.sub_joint_states)
        self.robot_state_sub = rospy.Subscriber('/niryo_robot/robot_state', RobotState, self.sub_robot_state)
        self.selected_tool_id_sub = rospy.Subscriber('/niryo_robot_hardware/tools/current_id', Int32, self.sub_selected_tool_id)
        self.learning_mode_sub = rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.sub_learning_mode)
        self.hardware_status_sub = rospy.Subscriber('/niryo_robot_hardware_interface/hardware_status', HardwareStatus,
                                                    self.sub_hardware_status)
        self.ros_log_status_sub = rospy.Subscriber('/niryo_robot_rpi/ros_log_status', LogStatus,
                                                   self.sub_ros_log_status)
        self.software_version_sub = rospy.Subscriber('/niryo_robot_hardware_interface/software_version',
                                                     SoftwareVersion, self.sub_software_version)

        self.conveyor_feedback_sub = rospy.Subscriber('/niryo_robot/conveyor/feedback', ConveyorFeedbackArray,
                                                      self.sub_conveyor_feedback)

    def stop_ros_subscribers(self):
        self.joint_state_sub.unregister()
        self.robot_state_sub.unregister()
        self.selected_tool_id_sub.unregister()
        self.learning_mode_sub.unregister()
        self.hardware_status_sub.unregister()
        self.ros_log_status_sub.unregister()
        self.software_version_sub.unregister()
        self.conveyor_feedback_sub.unregister()

    def sub_joint_states(self, msg):
        joints = []
        for i, joint in enumerate(msg.position):
            joints.append(handle_negative(int(joint * 1000)))

        self.setValuesOffset(IR_JOINTS, joints)

    def sub_robot_state(self, msg):
        pos = msg.position
        rpy = msg.rpy
        self.setValuesOffset(IR_POSITION_X, handle_negative(int(pos.x * 1000)))
        self.setValuesOffset(IR_POSITION_Y, handle_negative(int(pos.y * 1000)))
        self.setValuesOffset(IR_POSITION_Z, handle_negative(int(pos.z * 1000)))
        self.setValuesOffset(IR_ORIENTATION_X, handle_negative(int(rpy.roll * 1000)))
        self.setValuesOffset(IR_ORIENTATION_Y, handle_negative(int(rpy.pitch * 1000)))
        self.setValuesOffset(IR_ORIENTATION_Z, handle_negative(int(rpy.yaw * 1000)))

    def sub_selected_tool_id(self, msg):
        value = int(msg.data)
        self.setValuesOffset(IR_SELECTED_TOOL_ID, value)

    def sub_learning_mode(self, msg):
        value = int(msg.data)
        self.setValuesOffset(IR_LEARNING_MODE, [value])

    def sub_hardware_status(self, msg):
        self.setValuesOffset(IR_MOTORS_CONNECTION_UP, [int(msg.connection_up)])
        self.setValuesOffset(IR_CALIBR_NEEDED, [int(msg.calibration_needed)])
        self.setValuesOffset(IR_CALIBR_IN_PROGRESS, [int(msg.calibration_in_progress)])
        self.setValuesOffset(IR_RPI_TEMPERATURE, [int(msg.rpi_temperature)])
        self.setValuesOffset(IR_HARDWARE_VERSION, [int(msg.hardware_version)])

    def sub_ros_log_status(self, msg):
        self.setValuesOffset(IR_RPI_AVAILABLE_SPACE, [int(msg.available_disk_size)])
        self.setValuesOffset(IR_RPI_ROS_LOG_SIZE, [int(msg.log_size)])

    def sub_software_version(self, msg):
        version = msg.rpi_image_version
        if version != '':
            v_maj, v_min, v_patch = version.split('.')
            self.setValuesOffset(IR_RPI_VERSION_N1, [int(v_maj)])
            self.setValuesOffset(IR_RPI_VERSION_N2, [int(v_min)])
            self.setValuesOffset(IR_RPI_VERSION_N3, [int(v_patch)])

    def sub_conveyor_feedback(self, conveyor_array_feedback):
        if len(conveyor_array_feedback.conveyors) > 0:
            conveyor_1 = conveyor_array_feedback.conveyors[0]
            self.setValuesOffset(IR_CONVEYOR_1_CONNECTION_STATE, int(conveyor_1.connection_state))
            self.setValuesOffset(IR_CONVEYOR_1_CONTROL_STATUS, int(conveyor_1.running))
            self.setValuesOffset(IR_CONVEYOR_1_SPEED, int(conveyor_1.speed))
            self.setValuesOffset(IR_CONVEYOR_1_DIRECTION, handle_negative(int(conveyor_1.direction)))
        if len(conveyor_array_feedback.conveyors) > 1:
            conveyor_2 = conveyor_array_feedback.conveyors[1]
            self.setValuesOffset(IR_CONVEYOR_2_CONNECTION_STATE, int(conveyor_2.connection_state))
            self.setValuesOffset(IR_CONVEYOR_2_CONTROL_STATUS, int(conveyor_2.running))
            self.setValuesOffset(IR_CONVEYOR_2_SPEED, int(conveyor_2.speed))
            self.setValuesOffset(IR_CONVEYOR_2_DIRECTION, handle_negative(int(conveyor_2.direction)))
