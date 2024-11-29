#!/usr/bin/env python
import rospy
from enum import Enum

from niryo_robot_arm_commander.msg import ArmMoveCommand
from niryo_robot_python_ros_wrapper.ros_wrapper_enums import (ShiftPose,
                                                              PinMode,
                                                              PinState,
                                                              ToolID,
                                                              ConveyorID,
                                                              ConveyorDirection,
                                                              CommandEnum,
                                                              ConveyorTTL,
                                                              ConveyorCan)
from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper
from niryo_robot_utils.dataclasses.PoseMetadata import PoseMetadata
from niryo_robot_utils.dataclasses.JointsPosition import JointsPosition
from niryo_robot_utils.dataclasses.Pose import Pose

from .communication_functions import dict_to_packet
from .const_communication import HANDSHAKE_MSG_CLIENT_OUTDATED, minimum_client_version


class TcpCommandException(Exception):
    pass


def check_nb_args(expected_nbr):
    """
    Decorator used to check number of arguments before running a function
    """

    if isinstance(expected_nbr, int):
        expected_nbr = [expected_nbr]

    def decorator(function):

        def wrapper(*args):
            nbr_args = len(args) - 1  # Because "self"
            if nbr_args not in expected_nbr:
                err = "{} parameters expected but {} given".format(expected_nbr, nbr_args)
                raise TcpCommandException(err + "\nIn function {}".format(function.__name__))
            result = function(*args)
            return result

        return wrapper

    return decorator


class CommandInterpreter:
    """
    Object which interpret commands from TCP Client, and then, call Niryo Python ROS Wrapper to execute these commands
    """

    def __init__(self):
        # Niryo Python Ros Wrapper instance
        self.__niryo_robot = NiryoRosWrapper()

        # Dict containing a mapping between
        # "Command" enumeration (defined in const_communication) and interpreter's functions
        self.__commands_dict = self.__generate_functions_dict()
        # - Enumerations mapping
        self.__axis_string_dict_convertor = {
            "X": ShiftPose.AXIS_X,
            "Y": ShiftPose.AXIS_Y,
            "Z": ShiftPose.AXIS_Z,
            "ROLL": ShiftPose.ROT_ROLL,
            "PITCH": ShiftPose.ROT_PITCH,
            "YAW": ShiftPose.ROT_YAW,
        }

        self.__pin_mode_string_dict_convertor = {
            "OUTPUT": PinMode.OUTPUT,
            "INPUT": PinMode.INPUT,
        }

        self.__digital_state_string_dict_convertor = {
            "HIGH": PinState.HIGH,
            "LOW": PinState.LOW,
        }

        self.__digital_state_string_dict_convertor_inv = {
            index: string
            for string, index in self.__digital_state_string_dict_convertor.items()
        }

        self.__boolean_string_dict_converter = {"TRUE": True, "FALSE": False}

        self.__tools_string_dict_convertor = {
            "NONE": ToolID.NONE,
            "GRIPPER_1": ToolID.GRIPPER_1,
            "GRIPPER_2": ToolID.GRIPPER_2,
            "GRIPPER_3": ToolID.GRIPPER_3,
            "GRIPPER_4": ToolID.GRIPPER_4,
            "ELECTROMAGNET_1": ToolID.ELECTROMAGNET_1,
            "VACUUM_PUMP_1": ToolID.VACUUM_PUMP_1,
            "VACUUM_PUMP_2": ToolID.VACUUM_PUMP_2
        }

        self.__grippers_string_dict_convertor = {
            "NONE": ToolID.NONE,
            "GRIPPER_1": ToolID.GRIPPER_1,
            "GRIPPER_2": ToolID.GRIPPER_2,
            "GRIPPER_3": ToolID.GRIPPER_3,
            "GRIPPER_4": ToolID.GRIPPER_4,
        }

        self.__conveyor_id_string_dict_convertor = {
            "NONE": ConveyorID.NONE,
            "ID_1": ConveyorID.ID_1,
            "ID_2": ConveyorID.ID_2,
        }

        self.__conveyor_id_string_dict_convertor_inv = {
            ConveyorID.NONE: "NONE",
            ConveyorID.ID_1: "ID_1",
            ConveyorID.ID_2: "ID_2",
            ConveyorTTL.ID_1: "ID_1",
            ConveyorTTL.ID_2: "ID_2",
            ConveyorCan.ID_1: "ID_1",
            ConveyorCan.ID_2: "ID_2"
        }

        self.__conveyor_direction_string_dict_convertor = {
            "FORWARD": ConveyorDirection.FORWARD,
            "BACKWARD": ConveyorDirection.BACKWARD,
        }

        self.__available_tools_string_dict_convertor_inv = {
            index: string
            for string, index in self.__tools_string_dict_convertor.items()
        }

        self.__handshake_made = False

    # Error Handlers
    def __raise_exception_expected_choice(self, expected_choice, given):
        raise TcpCommandException("Expected one of the following: {}.\nGiven: {}".format(expected_choice, given))

    def __raise_exception_expected_type(self, expected_type, given):
        raise TcpCommandException("Expected the following type: " + expected_type + ".\nGiven: " + given)

    def __raise_exception_expected_parameters_nbr(self, expected_nbr, given):
        raise TcpCommandException(str(expected_nbr) + " parameters expected, given: " + str(given))

    # Command interpreter
    def interpret_command(self, dict_command_received):
        rospy.logdebug("Command Interpreter - Dict Received : {}".format(dict_command_received))
        # Check if command is a dict
        if not isinstance(dict_command_received, dict):
            msg = f"Cannot interpret command of incorrect type: {type(dict_command_received)}"
            return self.generate_dict_failure(message=msg)
        # Check if the dict is well formed
        if set(dict_command_received.keys()) != {"command", "param_list"}:
            msg = "Incorrect command format: " + str(dict_command_received)
            return self.generate_dict_failure(message=msg)
        # Check if command exists
        command_name = dict_command_received["command"].upper()
        if command_name not in self.__commands_dict:
            return dict_to_packet((self.generate_dict_failure(message="Unknown command")))

        # Execute command
        try:
            param_list = dict_command_received["param_list"]
            # noinspection PyArgumentList
            status, list_ret_param, payload = self.__commands_dict[command_name](*param_list)
            rospy.logdebug("Command Interpreter - {} - {}".format(status, list_ret_param))
        except (TcpCommandException, TypeError, NotImplementedError) as e:
            rospy.logerr(f"Command Interpreter - {e}, dict_command_received: {dict_command_received}", exc_info=True)
            return dict_to_packet(self.generate_dict_failure(command=command_name, message=str(e)))

        # Save answer Format parameters
        new_param_list = []
        for parameter in list_ret_param:
            if isinstance(parameter, Enum):
                new_param_list.append(parameter.name)
            elif isinstance(parameter, bool):
                new_param_list.append(str(parameter))
            else:
                new_param_list.append(parameter)

        dict_ret = {
            "status": status, "command": command_name, "list_ret_param": new_param_list, "payload_size": len(payload)
        }

        packet_data = dict_to_packet(dict_ret)
        rospy.logdebug("Command Interpreter - Packet response size {}\nDict response : {}".format(
            len(packet_data), dict_ret))

        return packet_data + payload

    # Dictionaries handler
    def __generate_functions_dict(self):
        """
        Generate a dict with the format : {command_1 : function_to_call_1, command_2 : function_to_call_2...}
        WARNING -> function name should have the following format : function_to_call_1 = __command_1.lower()
        For instance GRASP_WITH_TOOL is associated to "__grasp_with_tool"
        :return: dict
        """
        dict_c = dict()
        # noinspection PyTypeChecker
        for command in CommandEnum:
            try:
                func_adr = getattr(self, "_{}__{}".format(self.__class__.__name__, command.name.lower()))
            except AttributeError:
                func_adr = self.__not_implemented_function
            dict_c[command.name] = func_adr

        return dict_c

    def generate_dict_failure(self, command="", message="Failure in command_interpreter"):
        if not self.__handshake_made:
            message += ("\nNo handshake has been made between the client and the server. "
                        "This error could be the cause of an incompatible versions of the server and the client")
        return {"command": command, "status": "KO", "message": message, "list_ret_param": [], "payload_size": 0}

    def __send_answer(self, *params):
        """
        Return success with empty payload
        """
        return self.__send_answer_with_payload("", *params)

    @staticmethod
    def __send_answer_with_payload(payload, *params):
        encoded_payload = payload if isinstance(payload, bytes) else payload.encode()
        return "OK", params, encoded_payload

    def __check_list_belonging(self, value, list_):
        """
        Check if a value belong to a list
        """
        if value not in list_:
            self.__raise_exception_expected_choice(list_, value)

    def __check_dict_belonging(self, value, dict_):
        """
        Check if a value belong to a dictionary
        """
        if value not in dict_.keys():
            self.__raise_exception_expected_choice(dict_.keys(), value)

    def __check_type(self, value, type_):
        if type(value) is not type_:
            self.__raise_exception_expected_type(type_.__name__, value)

    def __check_instance(self, value, type_):
        if not isinstance(value, type_):
            self.__raise_exception_expected_type(
                type_.__name__ if not isinstance(type_, tuple) else " or ".join([type__.__name__ for type__ in type_]),
                value)

    def __check_list_type(self, list_, type_):
        for value in list_:
            self.__check_type(value, type_)

    def __check_and_get_from_dict(self, value, dict_):
        """
        Check if a value belong to a dictionary and return it
        """
        self.__check_dict_belonging(value, dict_)
        return dict_[value]

    def __map_list(self, list_, type_):
        """
        Try to map a list to another type (Very useful for list like joints
        which are acquired as string)
        """
        try:
            map_list = list(map(type_, list_))
            return map_list
        except ValueError:
            self.__raise_exception_expected_type(type_.__name__, list_)

    def __transform_to_type(self, value, type_):
        """
        Try to change value type to another
        """
        try:
            value = type_(value)
            return value
        except ValueError:
            self.__raise_exception_expected_type(type_.__name__, value)

    def __obj_from_dict(self, obj_dict):
        obj_class = JointsPosition if obj_dict.pop('obj_type') == 'JOINTS' else Pose
        return obj_class.from_dict(obj_dict)

    # --- FUNCTION LIST
    def __not_implemented_function(self, *_):
        raise NotImplementedError

    # - Main Purpose
    @check_nb_args(1)
    def __calibrate(self, calibrate_mode):
        self.__check_list_belonging(calibrate_mode, ["MANUAL", "AUTO"])
        self.__niryo_robot.calibrate_manual() if calibrate_mode == "MANUAL" else self.__niryo_robot.calibrate_auto()

        return self.__send_answer()

    @check_nb_args(0)
    def __get_learning_mode(self):
        is_learning_mode_enabled = self.__niryo_robot.get_learning_mode()
        return self.__send_answer(is_learning_mode_enabled)

    @check_nb_args(1)
    def __set_learning_mode(self, state_string):
        state = self.__check_and_get_from_dict(state_string, self.__boolean_string_dict_converter)
        ret = self.__niryo_robot.set_learning_mode(state)
        return self.__send_answer(ret)

    @check_nb_args(1)
    def __set_arm_max_velocity(self, max_velocity_percentage):
        if type(max_velocity_percentage) not in [int, float] or not (0 < max_velocity_percentage <= 100):
            self.__raise_exception_expected_type("float/integer [1 -100]", max_velocity_percentage)

        self.__niryo_robot.set_arm_max_velocity(max_velocity_percentage)
        return self.__send_answer()

    @check_nb_args(1)
    def __set_jog_control(self, state_string):
        state = self.__check_and_get_from_dict(state_string, self.__boolean_string_dict_converter)

        ret = self.__niryo_robot.set_jog_use_state(state)
        return self.__send_answer(ret)

    @check_nb_args(0)
    def __get_collision_detected(self):
        return self.__send_answer(self.__niryo_robot.collision_detected)

    @check_nb_args(0)
    def __clear_collision_detected(self):
        self.__niryo_robot.clear_collision_detected()
        return self.__send_answer()

    @check_nb_args(1)
    def __handshake(self, client_version):
        response = {}
        if client_version < minimum_client_version:
            response['message'] = HANDSHAKE_MSG_CLIENT_OUTDATED

        self.__handshake_made = True
        return self.__send_answer(response)

    # - Pose

    @check_nb_args(0)
    def __get_joints(self):
        return self.__send_answer(*self.__niryo_robot.get_joints())

    @check_nb_args(0)
    def __get_pose(self):
        arm_pose = self.__niryo_robot.get_pose()
        data_answer = [
            arm_pose.position.x,
            arm_pose.position.y,
            arm_pose.position.z,
            arm_pose.rpy.roll,
            arm_pose.rpy.pitch,
            arm_pose.rpy.yaw
        ]
        return self.__send_answer(*data_answer)

    @check_nb_args(0)
    def __get_pose_quat(self):
        arm_pose = self.__niryo_robot.get_pose()
        data_answer = [
            arm_pose.position.x,
            arm_pose.position.y,
            arm_pose.position.z,
            arm_pose.orientation.x,
            arm_pose.orientation.y,
            arm_pose.orientation.z,
            arm_pose.orientation.w
        ]
        return self.__send_answer(*data_answer)

    @check_nb_args(0)
    def __get_pose_v2(self):
        return self.__send_answer(self.__niryo_robot.get_pose_v2_as_list().to_dict())

    @check_nb_args(2)
    def __move(self, robot_position, linear):
        linear = self.__boolean_string_dict_converter[linear]
        robot_position_obj = self.__obj_from_dict(robot_position)

        if isinstance(robot_position_obj, JointsPosition) and linear:
            raise TcpCommandException("The robot can only do a linear move with a pose, not with joints")
        elif isinstance(robot_position_obj, JointsPosition):
            move_cmd = ArmMoveCommand.JOINTS
        else:
            move_cmd = ArmMoveCommand.LINEAR_POSE if linear else ArmMoveCommand.POSE

        self.__niryo_robot.move(robot_position_obj, move_cmd)
        return self.__send_answer()

    @check_nb_args(6)
    def __move_joints(self, *param_list):
        joint_list = self.__map_list(param_list, float)

        self.__niryo_robot.move_joints(*joint_list)
        return self.__send_answer()

    @check_nb_args(7)
    def __move_pose(self, *param_list):
        parameters_value_array = self.__map_list(param_list[:-1], float)

        param = parameters_value_array + [param_list[-1]]

        self.__niryo_robot.move_pose(*param)
        return self.__send_answer()

    @check_nb_args([2, 3])
    def __shift_pose(self, axis_string, value_string, linear="FALSE"):
        axis = self.__check_and_get_from_dict(axis_string, self.__axis_string_dict_convertor)
        value = self.__transform_to_type(value_string, float)
        linear = self.__boolean_string_dict_converter[linear]

        self.__niryo_robot.shift_pose(axis, value, linear)
        return self.__send_answer()

    @check_nb_args(2)
    def __shift_linear_pose(self, axis_string, value_string):
        axis = self.__check_and_get_from_dict(axis_string, self.__axis_string_dict_convertor)
        value = self.__transform_to_type(value_string, float)

        self.__niryo_robot.shift_linear_pose(axis, value)
        return self.__send_answer()

    @check_nb_args(6)
    def __jog_joints(self, *param_list):
        joint_list = self.__map_list(param_list, float)

        self.__niryo_robot.jog_joints_shift(joint_list)
        return self.__send_answer()

    @check_nb_args(6)
    def __jog_pose(self, *param_list):
        shift_values_list = self.__map_list(param_list, float)

        self.__niryo_robot.jog_pose_shift(shift_values_list)
        return self.__send_answer()

    @check_nb_args(1)
    def __jog(self, robot_position_dict):
        robot_position = self.__obj_from_dict(robot_position_dict)
        self.__niryo_robot.jog_shift(robot_position)
        return self.__send_answer()

    @check_nb_args(7)
    def __move_linear_pose(self, *param_list):
        parameters_value_array = self.__map_list(param_list[:-1], float)

        param = parameters_value_array + [param_list[-1]]

        self.__niryo_robot.move_linear_pose(*param)
        return self.__send_answer()

    @check_nb_args(1)
    def __forward_kinematics(self, joints_position):
        joints_position_obj = JointsPosition.from_dict(joints_position)
        return self.__send_answer(self.__niryo_robot.forward_kinematics(joints_position_obj).to_dict())

    @check_nb_args(1)
    def __inverse_kinematics(self, pose):
        pose_obj = Pose.from_dict(pose)
        return self.__send_answer(self.__niryo_robot.inverse_kinematics(pose_obj).to_dict())

    @check_nb_args(1)
    def __forward_kinematics_v2(self, joints_position):
        joints_position_obj = JointsPosition.from_dict(joints_position)
        return self.__send_answer(self.__niryo_robot.forward_kinematics_v2(joints_position_obj).to_dict())

    @check_nb_args(1)
    def __inverse_kinematics_v2(self, pose):
        pose_obj = Pose.from_dict(pose)
        return self.__send_answer(self.__niryo_robot.inverse_kinematics_v2(pose_obj).to_dict())

    # - Saved Pose

    @check_nb_args(1)
    def __get_pose_saved(self, *param_list):
        pose = self.__niryo_robot.get_pose_saved(*param_list)
        return self.__send_answer(pose.to_dict())

    @check_nb_args([2, 7])
    def __save_pose(self, *param_list):
        if len(param_list) == 7:
            pose = Pose(*param_list[1:], metadata=PoseMetadata.v1())
        else:
            pose = Pose.from_dict(param_list[1])

        self.__niryo_robot.save_pose(param_list[0], pose)
        return self.__send_answer()

    @check_nb_args(1)
    def __delete_pose(self, *param_list):
        self.__niryo_robot.delete_pose(*param_list)
        return self.__send_answer()

    @check_nb_args(0)
    def __get_saved_pose_list(self):
        pose_list = self.__niryo_robot.get_saved_pose_list()
        return self.__send_answer(pose_list)

    # - Pick/Place

    @check_nb_args(6)
    def __pick_from_pose(self, *param_list):
        parameters_value_array = self.__map_list(param_list, float)

        self.__niryo_robot.pick_from_pose(*parameters_value_array)
        return self.__send_answer()

    @check_nb_args(6)
    def __place_from_pose(self, *param_list):
        parameters_value_array = self.__map_list(param_list, float)

        self.__niryo_robot.place_from_pose(*parameters_value_array)
        return self.__send_answer()

    @check_nb_args(1)
    def __pick(self, robot_position):
        robot_position_obj = self.__obj_from_dict(robot_position)
        self.__niryo_robot.pick(robot_position_obj)
        return self.__send_answer()

    @check_nb_args(1)
    def __place(self, robot_position):
        robot_position_obj = self.__obj_from_dict(robot_position)
        self.__niryo_robot.place(robot_position_obj)
        return self.__send_answer()

    @check_nb_args(3)
    def __pick_and_place(self, pick_pose, place_pos, dist_smoothing):
        if not isinstance(pick_pose, dict):
            pick_pose = Pose(*pick_pose, metadata=PoseMetadata.v1())
        else:
            pick_pose = self.__obj_from_dict(pick_pose)

        if not isinstance(place_pos, dict):
            place_pose = Pose(*place_pos, metadata=PoseMetadata.v1())
        else:
            place_pose = self.__obj_from_dict(place_pos)

        self.__check_type(dist_smoothing, float)

        self.__niryo_robot.pick_and_place(pick_pose, place_pose, dist_smoothing)
        return self.__send_answer()

    # - Trajectories
    @check_nb_args(1)
    def __get_trajectory_saved(self, *param_list):
        traj = self.__niryo_robot.get_trajectory_saved(*param_list)
        dict_traj = [waypoint.to_dict() for waypoint in traj]
        return self.__send_answer(dict_traj)

    @check_nb_args(0)
    def __get_saved_trajectory_list(self):
        traj_list = self.__niryo_robot.get_saved_trajectory_list()
        return self.__send_answer(traj_list)

    @check_nb_args(1)
    def __execute_registered_trajectory(self, *param_list):
        self.__niryo_robot.execute_registered_trajectory(*param_list)
        return self.__send_answer()

    @check_nb_args(2)
    def __execute_trajectory_from_poses(self, *param_list):
        list_poses = []
        for pose in param_list[0]:
            if len(pose) != 7 and len(pose) != 6:
                self.__raise_exception_expected_parameters_nbr('7 or 6', len(pose))
            list_poses.append(self.__map_list(pose, float))

        dist_smoothing = param_list[1]
        self.__check_type(dist_smoothing, float)

        self.__niryo_robot.execute_trajectory_from_poses(list_poses, dist_smoothing)
        return self.__send_answer()

    @check_nb_args(3)
    def __execute_trajectory_from_poses_and_joints(self, *param_list):
        list_poses_joints = []
        for pose_joint in param_list[0]:
            if not (6 <= len(pose_joint) <= 7):
                self.__raise_exception_expected_parameters_nbr('7 or 6', len(pose_joint))
            list_poses_joints.append(self.__map_list(pose_joint, float))

        list_type = []
        for type_ in param_list[1]:
            if type_ not in ['joints', 'pose']:
                self.__raise_exception_expected_choice("'pose' or 'joint'", type_)
            list_type.append(type_)

        dist_smoothing = param_list[2]
        self.__check_type(dist_smoothing, float)

        self.__niryo_robot.execute_trajectory_from_poses_and_joints(list_poses_joints, list_type, dist_smoothing)
        return self.__send_answer()

    @check_nb_args(2)
    def __execute_trajectory(self, robot_positions, dist_smoothing):
        robot_positions_obj = []
        for robot_position in robot_positions:
            robot_position_obj = self.__obj_from_dict(robot_position)
            robot_positions_obj.append(robot_position_obj)
        self.__niryo_robot.execute_trajectory(robot_positions_obj, dist_smoothing)
        return self.__send_answer()

    @check_nb_args(3)
    def __save_trajectory(self, trajectory, trajectory_name, trajectory_description):
        obj_list = []
        for traj_point in trajectory:
            if isinstance(traj_point, dict):
                obj_list.append(JointsPosition.from_dict(traj_point))
            else:
                obj_list.append(self.__map_list(traj_point, float))
        self.__niryo_robot.save_trajectory(obj_list, trajectory_name, trajectory_description)
        return self.__send_answer()

    @check_nb_args(2)
    def __save_last_learned_trajectory(self, *param_list):
        self.__niryo_robot.save_last_learned_trajectory(*param_list)
        return self.__send_answer()

    @check_nb_args(3)
    def __update_trajectory_infos(self, *param_list):
        self.__niryo_robot.update_trajectory_infos(*param_list)
        return self.__send_answer()

    @check_nb_args(1)
    def __delete_trajectory(self, *param_list):
        self.__niryo_robot.delete_trajectory(*param_list)
        return self.__send_answer()

    @check_nb_args(0)
    def __clean_trajectory_memory(self):
        self.__niryo_robot.clean_trajectory_memory()
        return self.__send_answer()

    # Dynamic frames
    @check_nb_args(0)
    def __get_saved_dynamic_frame_list(self):
        frames_list, frames_description = self.__niryo_robot.get_saved_dynamic_frame_list()
        return self.__send_answer((frames_list, frames_description))

    @check_nb_args(1)
    def __get_saved_dynamic_frame(self, *param_list):
        frame = self.__niryo_robot.get_saved_dynamic_frame(*param_list)
        return self.__send_answer(*frame)

    @check_nb_args(6)
    def __save_dynamic_frame_from_poses(self, *param_list):
        name = param_list[0]
        description = param_list[1]

        list_poses = []
        for raw_pose in param_list[2:-1]:
            if isinstance(raw_pose, dict):
                list_poses.append(Pose.from_dict(raw_pose))
            else:
                list_poses.append(Pose(*raw_pose, metadata=PoseMetadata.v1()))

        belong_to_workspace = self.__boolean_string_dict_converter[param_list[-1]]

        self.__niryo_robot.save_dynamic_frame_from_poses(name, description, list_poses, belong_to_workspace)
        return self.__send_answer()

    @check_nb_args(6)
    def __save_dynamic_frame_from_points(self, *param_list):
        name = param_list[0]
        description = param_list[1]
        list_poses = list(param_list[2:-1])
        belong_to_workspace = self.__boolean_string_dict_converter[param_list[-1]]

        self.__niryo_robot.save_dynamic_frame_from_points(name, description, list_poses, belong_to_workspace)
        return self.__send_answer()

    @check_nb_args(3)
    def __edit_dynamic_frame(self, *param_list):
        self.__niryo_robot.edit_dynamic_frame(*param_list)
        return self.__send_answer()

    @check_nb_args(2)
    def __delete_dynamic_frame(self, *param_list):
        name = param_list[0]
        belong_to_workspace = self.__boolean_string_dict_converter[param_list[1]]
        self.__niryo_robot.delete_dynamic_frame(name, belong_to_workspace)
        return self.__send_answer()

    @check_nb_args(2)
    def __move_relative(self, *param_list):
        self.__niryo_robot.move_relative(*param_list)
        return self.__send_answer()

    @check_nb_args(2)
    def __move_linear_relative(self, *param_list):
        self.__niryo_robot.move_linear_relative(*param_list)
        return self.__send_answer()

    # -- Tools

    @check_nb_args(0)
    def __update_tool(self):
        self.__niryo_robot.update_tool()
        return self.__send_answer()

    @check_nb_args(0)
    def __get_current_tool_id(self):
        tool_id = self.__niryo_robot.get_current_tool_id()
        return self.__send_answer(self.__available_tools_string_dict_convertor_inv[tool_id])

    @check_nb_args(0)
    def __grasp_with_tool(self):
        self.__niryo_robot.grasp_with_tool()
        return self.__send_answer()

    @check_nb_args(0)
    def __release_with_tool(self):
        self.__niryo_robot.release_with_tool()
        return self.__send_answer()

    # - Gripper
    @check_nb_args(3)
    def __open_gripper(self, speed, max_troque_percentage, hold_torque_percentage):
        self.__check_type(speed, int)
        self.__check_type(max_troque_percentage, int)
        self.__check_type(hold_torque_percentage, int)

        self.__niryo_robot.open_gripper(speed, max_troque_percentage, hold_torque_percentage)
        return self.__send_answer()

    @check_nb_args(3)
    def __close_gripper(self, speed, max_troque_percentage, hold_torque_percentage):
        self.__check_type(speed, int)
        self.__check_type(max_troque_percentage, int)
        self.__check_type(hold_torque_percentage, int)

        self.__niryo_robot.close_gripper(speed, max_troque_percentage, hold_torque_percentage)
        return self.__send_answer()

    # - Vacuum
    @check_nb_args(0)
    def __pull_air_vacuum_pump(self):
        self.__niryo_robot.pull_air_vacuum_pump()
        return self.__send_answer()

    @check_nb_args(0)
    def __push_air_vacuum_pump(self):
        self.__niryo_robot.push_air_vacuum_pump()
        return self.__send_answer()

    # - Electromagnet
    @check_nb_args(1)
    def __setup_electromagnet(self, pin_string):
        self.__niryo_robot.setup_electromagnet(pin_string)
        return self.__send_answer()

    @check_nb_args(1)
    def __activate_electromagnet(self, pin_string):
        self.__niryo_robot.activate_electromagnet(pin_string)
        return self.__send_answer()

    @check_nb_args(1)
    def __deactivate_electromagnet(self, pin_string):
        self.__niryo_robot.deactivate_electromagnet(pin_string)
        return self.__send_answer()

    # TCP
    @check_nb_args(1)
    def __enable_tcp(self, enable):
        boolean_enable = self.__check_and_get_from_dict(enable, self.__boolean_string_dict_converter)
        self.__niryo_robot.enable_tcp(boolean_enable)
        return self.__send_answer()

    @check_nb_args(6)
    def __set_tcp(self, *param_list):
        parameters_value_array = self.__map_list(param_list, float)
        self.__niryo_robot.set_tcp(*parameters_value_array)
        return self.__send_answer()

    @check_nb_args(0)
    def __reset_tcp(self):
        self.__niryo_robot.reset_tcp()
        return self.__send_answer()

    @check_nb_args(0)
    def __tool_reboot(self):
        self.__niryo_robot.tool_reboot()
        return self.__send_answer()

    @check_nb_args(0)
    def __get_tcp(self):
        tcp = self.__niryo_robot.get_tcp(as_list=True)
        return self.__send_answer(*tcp)

    # - Hardware

    @check_nb_args(2)
    def __set_pin_mode(self, pin_string, pin_mode_string):
        pin_mode = self.__check_and_get_from_dict(pin_mode_string, self.__pin_mode_string_dict_convertor)
        self.__niryo_robot.set_pin_mode(pin_string, pin_mode)
        return self.__send_answer()

    @check_nb_args(2)
    def __digital_write(self, pin_string, state_string):
        state = self.__check_and_get_from_dict(state_string, self.__digital_state_string_dict_convertor)
        self.__niryo_robot.digital_write(pin_string, state)
        return self.__send_answer()

    @check_nb_args(1)
    def __digital_read(self, pin_string):
        digital_state = self.__niryo_robot.digital_read(pin_string)
        return self.__send_answer(self.__digital_state_string_dict_convertor_inv[digital_state])

    @check_nb_args(0)
    def __get_hardware_status(self):
        hw_status = self.__niryo_robot.get_hardware_status()
        data_answer = [
            hw_status.rpi_temperature,
            hw_status.hardware_version,
            hw_status.connection_up,
            "\'" + hw_status.error_message + "\'",
            hw_status.calibration_needed,
            hw_status.calibration_in_progress,
            hw_status.motor_names,
            hw_status.motor_types,
            hw_status.temperatures,
            hw_status.voltages,
            hw_status.hardware_errors
        ]
        return self.__send_answer(*data_answer)

    @check_nb_args(0)
    def __get_digital_io_state(self):
        digital_io_state_array = self.__niryo_robot.get_digital_io_state()

        data_answer = []
        data_answer += [[
            di.name,
            PinMode.INPUT,
            int(di.value),
        ] for di in digital_io_state_array.digital_inputs]
        data_answer += [[
            do.name,
            PinMode.OUTPUT,
            int(do.value),
        ] for do in digital_io_state_array.digital_outputs]
        data_answer.sort(key=lambda x: x[0])
        return self.__send_answer(*data_answer)

    @check_nb_args(0)
    def __get_analog_io_state(self):
        analog_io_state_array = self.__niryo_robot.get_analog_io_state()

        data_answer = []
        data_answer += [[
            ai.name,
            PinMode.INPUT,
            ai.value,
        ] for ai in analog_io_state_array.analog_inputs]
        data_answer += [[
            ao.name,
            PinMode.OUTPUT,
            ao.value,
        ] for ao in analog_io_state_array.analog_outputs]
        data_answer.sort(key=lambda x: x[0])
        return self.__send_answer(*data_answer)

    @check_nb_args(2)
    def __analog_write(self, pin_string, voltage):
        self.__check_instance(voltage, (float, int))
        self.__niryo_robot.analog_write(pin_string, voltage)
        return self.__send_answer()

    @check_nb_args(1)
    def __analog_read(self, pin_string):
        analog_state = self.__niryo_robot.analog_read(pin_string)
        return self.__send_answer(analog_state)

    @check_nb_args(0)
    def __custom_button_state(self):
        is_pressed = self.__niryo_robot.custom_button.is_pressed()
        return self.__send_answer(is_pressed)

    # - Conveyor
    @check_nb_args(0)
    def __set_conveyor(self):
        conveyor_id = self.__niryo_robot.set_conveyor()
        return self.__send_answer(self.__conveyor_id_string_dict_convertor_inv[conveyor_id])

    @check_nb_args(1)
    def __unset_conveyor(self, conveyor_id_string):
        conveyor_id = self.__check_and_get_from_dict(conveyor_id_string, self.__conveyor_id_string_dict_convertor)

        status, message = self.__niryo_robot.unset_conveyor(conveyor_id)
        return self.__send_answer(status, message)

    @check_nb_args(4)
    def __control_conveyor(self, conveyor_id_string, control_on_string, speed, direction_string):
        conveyor_id = self.__check_and_get_from_dict(conveyor_id_string, self.__conveyor_id_string_dict_convertor)
        control_on = self.__check_and_get_from_dict(control_on_string, self.__boolean_string_dict_converter)

        self.__check_type(speed, int)
        if not (0 <= speed <= 100):
            self.__raise_exception_expected_choice("[0 => 100]", speed)

        direction = self.__check_and_get_from_dict(direction_string, self.__conveyor_direction_string_dict_convertor)

        self.__niryo_robot.control_conveyor(conveyor_id, control_on, speed, direction)
        return self.__send_answer()

    @check_nb_args(0)
    def __get_connected_conveyors_id(self):
        conveyors = self.__niryo_robot.get_conveyors_number()
        conveyors_list = [self.__conveyor_id_string_dict_convertor_inv[conveyor_id] for conveyor_id in conveyors]
        return self.__send_answer(conveyors_list)

    @check_nb_args(0)
    def __get_conveyors_feedback(self):
        feedback = self.__niryo_robot.get_conveyors_feedback()
        for i in range(len(feedback)):
            feedback[i]['conveyor_id'] = self.__conveyor_id_string_dict_convertor_inv[feedback[i]['conveyor_id']]
            feedback[i]['connection_state'] = str(feedback[i]['connection_state'])
        return self.__send_answer(feedback)

    # - Vision

    @check_nb_args(0)
    def __get_image_compressed(self):
        compressed_image = self.__niryo_robot.get_compressed_image()
        return self.__send_answer_with_payload(compressed_image)

    @check_nb_args(1)
    def __set_image_brightness(self, brightness_factor):
        self.__niryo_robot.set_brightness(brightness_factor)
        return self.__send_answer()

    @check_nb_args(1)
    def __set_image_contrast(self, contrast_factor):
        self.__niryo_robot.set_contrast(contrast_factor)
        return self.__send_answer()

    @check_nb_args(1)
    def __set_image_saturation(self, saturation_factor):
        self.__niryo_robot.set_saturation(saturation_factor)
        return self.__send_answer()

    @check_nb_args(0)
    def __get_image_parameters(self):
        brightness_factor, contrast_factor, saturation_factor = self.__niryo_robot.get_image_parameters()
        return self.__send_answer(brightness_factor, contrast_factor, saturation_factor)

    @check_nb_args(5)
    def __get_target_pose_from_rel(self, *param_list):
        workspace = param_list[0]
        distance_params = param_list[1:5]
        height_offset, x_rel, y_rel, yaw_rel = self.__map_list(distance_params, float)

        target_msg = self.__niryo_robot.get_target_pose_from_rel(workspace, height_offset, x_rel, y_rel, yaw_rel)

        pose_list = self.__niryo_robot.pose_from_msg(target_msg).to_list()
        return self.__send_answer(pose_list)

    @check_nb_args(4)
    def __get_target_pose_from_cam(self, *param_list):
        self.__check_type(param_list[1], float)

        obj_found, target_msg, obj_shape, obj_color = self.__niryo_robot.get_target_pose_from_cam(*param_list)

        pose_list = []
        if obj_found:
            pose_list = self.__niryo_robot.pose_from_msg(target_msg).to_list()

        return self.__send_answer(obj_found, pose_list, obj_shape, obj_color)

    @check_nb_args([4, 5])
    def __vision_pick(self, *param_list):
        self.__check_type(param_list[1], float)

        data_list = self.__niryo_robot.vision_pick(*param_list)
        return self.__send_answer(*data_list)

    @check_nb_args(4)
    def __move_to_object(self, *param_list):
        self.__check_type(param_list[1], float)

        data_list = self.__niryo_robot.move_to_object(*param_list)
        return self.__send_answer(*data_list)

    @check_nb_args(3)
    def __detect_object(self, *param_list):
        object_found, rel_pose, shape, color = self.__niryo_robot.detect_object(*param_list)
        if not object_found:
            return self.__send_answer(object_found, 0, 0, 0, shape, color)
        return self.__send_answer(object_found, rel_pose.x, rel_pose.y, rel_pose.yaw, shape, color)

    @check_nb_args(5)
    def __save_workspace_from_poses(self, *param_list):
        name = param_list[0]

        list_poses = []
        for raw_pose in param_list[1:]:
            if isinstance(raw_pose, dict):
                list_poses.append(Pose.from_dict(raw_pose))
            else:
                list_poses.append(Pose(*raw_pose, metadata=PoseMetadata.v1()))

        self.__niryo_robot.save_workspace_from_poses(name, list_poses)
        return self.__send_answer()

    @check_nb_args(5)
    def __save_workspace_from_points(self, *param_list):
        name = param_list[0]
        list_points = list(param_list[1:])

        self.__niryo_robot.save_workspace_from_points(name, list_points)
        return self.__send_answer()

    @check_nb_args(1)
    def __delete_workspace(self, name):
        self.__niryo_robot.delete_workspace(name)
        return self.__send_answer()

    @check_nb_args(1)
    def __get_workspace_ratio(self, workspace_name):
        return self.__send_answer(self.__niryo_robot.get_workspace_ratio(workspace_name))

    @check_nb_args(0)
    def __get_workspace_list(self):
        return self.__send_answer(self.__niryo_robot.get_workspace_list())

    @check_nb_args(0)
    def __get_camera_intrinsics(self):
        mat_k, mat_d = self.__niryo_robot.get_camera_intrinsics()
        return self.__send_answer(mat_k, mat_d)

    # - Led ring
    @check_nb_args(2)
    def __led_ring_solid(self, color, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__niryo_robot.led_ring.solid(color, wait)
        return self.__send_answer()

    @check_nb_args(1)
    def __led_ring_turn_off(self, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        self.__niryo_robot.led_ring.turn_off(wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_flash(self, color, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.flashing(color, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_alternate(self, color_list, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        for index, color in enumerate(color_list):
            color = self.__check_color_led_ring(color)
            color_list[index] = color
        self.__check_instance(period, (float, int))
        self.__niryo_robot.led_ring.alternate(color_list, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_chase(self, color, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)

        self.__niryo_robot.led_ring.chase(color, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(3)
    def __led_ring_wipe(self, color, period, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__niryo_robot.led_ring.wipe(color, period, wait)
        return self.__send_answer()

    @check_nb_args(3)
    def __led_ring_rainbow(self, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.rainbow(period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(3)
    def __led_ring_rainbow_cycle(self, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.rainbow_cycle(period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(3)
    def __led_ring_rainbow_chase(self, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.rainbow_chase(period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_go_up(self, color, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.go_up(color, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_go_up_down(self, color, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.go_up_down(color, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_breath(self, color, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.breath(color, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(4)
    def __led_ring_snake(self, color, period, iterations, wait):
        wait = self.__check_and_get_from_dict(wait, self.__boolean_string_dict_converter)
        color = self.__check_color_led_ring(color)
        self.__check_instance(period, (float, int))
        self.__check_type(iterations, int)
        self.__niryo_robot.led_ring.snake(color, period, iterations, wait)
        return self.__send_answer()

    @check_nb_args(1)
    def __led_ring_custom(self, color_list):
        verified_color_list = [self.__check_color_led_ring(color) for color in color_list]
        self.__niryo_robot.led_ring.custom(verified_color_list)
        return self.__send_answer()

    @check_nb_args(2)
    def __led_ring_set_led(self, led_id, color):
        color = self.__check_color_led_ring(color)
        self.__check_type(led_id, int)
        self.__niryo_robot.led_ring.set_led_color(led_id, color)
        return self.__send_answer()

    # Usefull method Led Ring
    def __check_color_led_ring(self, color):
        self.__check_type(color, list)
        if len(color) != 3:
            self.__raise_exception_expected_type("Color must be a list of size 3: [r, g, b]", color)
        for index, color_elem in enumerate(color):
            if not (0 <= color_elem <= 255):
                self.__raise_exception_expected_choice("[0 => 255]", color_elem)
            color[index] = self.__transform_to_type(color_elem, float)
        return color

    # - Sound
    @check_nb_args(4)
    def __play_sound(self, sound_name, wait_end, start_time_sec, end_time_sec):
        return self.__send_answer(
            self.__niryo_robot.sound.play(sound_name,
                                          self.__boolean_string_dict_converter[wait_end],
                                          start_time_sec,
                                          end_time_sec))

    @check_nb_args(1)
    def __set_volume(self, sound_volume):
        return self.__send_answer(self.__niryo_robot.sound.set_volume(sound_volume))

    @check_nb_args(0)
    def __stop_sound(self):
        return self.__send_answer(self.__niryo_robot.sound.stop())

    @check_nb_args(0)
    def __get_sounds(self):
        return self.__send_answer(self.__niryo_robot.sound.sounds)

    @check_nb_args(1)
    def __get_sound_duration(self, sound_name):
        return self.__send_answer(self.__niryo_robot.sound.get_sound_duration(sound_name))

    @check_nb_args(2)
    def __say(self, text, language):
        success, message = self.__niryo_robot.sound.say(text, language)
        return self.__send_answer(-1 if success else 1, message)
