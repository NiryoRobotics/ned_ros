from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper


def get_mapping(ros_wrapper: NiryoRosWrapper):
    return {
        0: lambda: ros_wrapper.get_joints()[0],
        1: lambda: ros_wrapper.get_joints()[1],
        2: lambda: ros_wrapper.get_joints()[2],
        3: lambda: ros_wrapper.get_joints()[3],
        4: lambda: ros_wrapper.get_joints()[4],
        5: lambda: ros_wrapper.get_joints()[5],
        6: lambda: ros_wrapper.get_joints()[6],
        10: lambda: ros_wrapper.get_pose().position.x,
        11: lambda: ros_wrapper.get_pose().position.y,
        12: lambda: ros_wrapper.get_pose().position.z,
        13: lambda: ros_wrapper.get_pose().rpy.roll,
        14: lambda: ros_wrapper.get_pose().rpy.pitch,
        15: lambda: ros_wrapper.get_pose().rpy.yaw,
        403: lambda: ros_wrapper.get_hardware_status().rpi_temperature,
        404: lambda: ros_wrapper
    }
