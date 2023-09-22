from niryo_robot_python_ros_wrapper.ros_wrapper import NiryoRosWrapper

n = NiryoRosWrapper()
"""
TODO: pass these values to discrete input
300: Learning Mode activated
400: Motors connection up (Ok = 1, Not ok = 0)
401: Calibration needed flag
402: Calibration in progress flag
530: Conveyor 1 connection state (Connected = 1 , Not connected = 0)
531: Conveyor 1 control status ( On = 0, Off = 1)
540: Conveyor 2 connection state (Connected = 1 , Not connected = 0)
541: Conveyor 2 control status ( On = 0, Off = 1)
600 - 604: Analog IO mode
"""

mapping = {
    0: lambda: n.get_joints()[0],
    1: lambda: n.get_joints()[1],
    2: lambda: n.get_joints()[2],
    3: lambda: n.get_joints()[3],
    4: lambda: n.get_joints()[4],
    5: lambda: n.get_joints()[5],
    10: lambda: n.get_pose().position.x,
    11: lambda: n.get_pose().position.y,
    12: lambda: n.get_pose().position.z,
    13: lambda: n.get_pose().rpy.roll,
    14: lambda: n.get_pose().rpy.pitch,
    15: lambda: n.get_pose().rpy.yaw,
    200: lambda: n.get_current_tool_id(),
}
