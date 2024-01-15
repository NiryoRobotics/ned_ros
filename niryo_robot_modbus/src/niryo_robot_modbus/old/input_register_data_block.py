#!/usr/bin/env python

import rospy
from collections import OrderedDict

from .data_block import DataBlock

# Messages
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32

from conveyor_interface.msg import ConveyorFeedbackArray
from niryo_robot_msgs.msg import HardwareStatus, RobotState, SoftwareVersion
from niryo_robot_rpi.msg import StorageStatus
from niryo_robot_rpi.msg import AnalogIOState
from niryo_robot_rpi.srv import SetIOModeRequest
"""
 - Each address contains a 16 bits value
 - READ ONLY registers

 --> State of the robot
"""
MULT = 1000

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
IR_RPI_TOTAL_SPACE = 409

IR_CONVEYOR_1_CONNECTION_STATE = 530
IR_CONVEYOR_1_CONTROL_STATUS = 531
IR_CONVEYOR_1_SPEED = 532
IR_CONVEYOR_1_DIRECTION = 533

IR_CONVEYOR_2_CONNECTION_STATE = 540
IR_CONVEYOR_2_CONTROL_STATUS = 541
IR_CONVEYOR_2_SPEED = 542
IR_CONVEYOR_2_DIRECTION = 543

IR_AIO_MODE = 600
IR_AIO_STATE = 610

AIO_MODE_OUTPUT = SetIOModeRequest.OUTPUT
AIO_MODE_INPUT = SetIOModeRequest.INPUT

AIO_NAME_TO_ADDRESS = OrderedDict({
    "AI1": 0,
    "AI2": 1,
    "AO1": 2,
    "AO2": 3,
})


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
        self.ros_storage_status_sub = None
        self.software_version_sub = None
        self.conveyor_feedback_sub = None
        self.analog_io_state_sub = None

    def start_ros_subscribers(self):
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.sub_joint_states)
        self.robot_state_sub = rospy.Subscriber('/niryo_robot/robot_state', RobotState, self.sub_robot_state)
        self.selected_tool_id_sub = rospy.Subscriber('/niryo_robot_tools_commander/current_id',
                                                     Int32,
                                                     self.sub_selected_tool_id)
        self.learning_mode_sub = rospy.Subscriber('/niryo_robot/learning_mode/state', Bool, self.sub_learning_mode)
        self.hardware_status_sub = rospy.Subscriber('/niryo_robot_hardware_interface/hardware_status',
                                                    HardwareStatus,
                                                    self.sub_hardware_status)
        self.ros_storage_status_sub = rospy.Subscriber('/niryo_robot_rpi/storage_status',
                                                       StorageStatus,
                                                       self.sub_storage_status)
        self.software_version_sub = rospy.Subscriber('/niryo_robot_hardware_interface/software_version',
                                                     SoftwareVersion,
                                                     self.sub_software_version)
        self.conveyor_feedback_sub = rospy.Subscriber('/niryo_robot/conveyor/feedback',
                                                      ConveyorFeedbackArray,
                                                      self.sub_conveyor_feedback)
        self.analog_io_state_sub = rospy.Subscriber('/niryo_robot_rpi/analog_io_state',
                                                    AnalogIOState,
                                                    self.sub_analog_io_state)

    def stop_ros_subscribers(self):
        self.joint_state_sub.unregister()
        self.robot_state_sub.unregister()
        self.selected_tool_id_sub.unregister()
        self.learning_mode_sub.unregister()
        self.hardware_status_sub.unregister()
        self.ros_storage_status_sub.unregister()
        self.software_version_sub.unregister()
        self.conveyor_feedback_sub.unregister()
        self.analog_io_state_sub.unregister()

    def sub_joint_states(self, msg):
        joints = []
        for i, joint in enumerate(msg.position):
            joints.append(handle_negative(int(joint * MULT)))

        self.setValuesOffset(IR_JOINTS, joints)

    def sub_robot_state(self, msg):
        pos = msg.position
        rpy = msg.rpy
        self.setValuesOffset(IR_POSITION_X, handle_negative(int(pos.x * MULT)))
        self.setValuesOffset(IR_POSITION_Y, handle_negative(int(pos.y * MULT)))
        self.setValuesOffset(IR_POSITION_Z, handle_negative(int(pos.z * MULT)))
        self.setValuesOffset(IR_ORIENTATION_X, handle_negative(int(rpy.roll * MULT)))
        self.setValuesOffset(IR_ORIENTATION_Y, handle_negative(int(rpy.pitch * MULT)))
        self.setValuesOffset(IR_ORIENTATION_Z, handle_negative(int(rpy.yaw * MULT)))

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

    def sub_storage_status(self, msg):
        self.setValuesOffset(IR_RPI_AVAILABLE_SPACE, [int(msg.available_disk_size)])
        self.setValuesOffset(IR_RPI_ROS_LOG_SIZE, [int(msg.log_size)])
        self.setValuesOffset(IR_RPI_TOTAL_SPACE, [int(msg.total_disk_size)])

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

    def sub_analog_io_state(self, msg):
        for ain in msg.analog_inputs:
            self.setValuesOffset(IR_AIO_MODE + AIO_NAME_TO_ADDRESS[ain.name], AIO_MODE_INPUT)
            self.setValuesOffset(IR_AIO_STATE + AIO_NAME_TO_ADDRESS[ain.name], int(ain.value * MULT))

        for aout in msg.analog_outputs:
            self.setValuesOffset(IR_AIO_MODE + AIO_NAME_TO_ADDRESS[aout.name], AIO_MODE_OUTPUT)
            self.setValuesOffset(IR_AIO_STATE + AIO_NAME_TO_ADDRESS[aout.name], int(aout.value * MULT))
