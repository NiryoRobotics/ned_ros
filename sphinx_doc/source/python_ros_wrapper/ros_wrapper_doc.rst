Python ROS Wrapper documentation
=====================================

This file presents the different Functions, Classes & Enums available with the API.

.. contents::
   :local:
   :depth: 1

API functions
------------------------------------

| This class allows you to control the robot via internal API.
| By controlling, we mean:

- Moving the robot.
- Using Vision.
- Controlling Conveyors Belt.
- Playing with hardware.

List of functions subsections:

.. contents::
   :local:
   :depth: 1

.. automodule:: niryo_robot_python_ros_wrapper.ros_wrapper
   :members: NiryoRosWrapper

Main purpose functions
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: calibrate_auto, calibrate_manual, get_learning_mode, set_learning_mode,
              set_arm_max_velocity
    :member-order: bysource

Joints & Pose
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: get_joints, get_pose, get_pose_as_list, move_joints, move_to_sleep_pose, move_pose,  move_linear_pose,
              shift_pose, shift_linear_pose, 
              set_jog_use_state, jog_joints_shift, jog_pose_shift,
              forward_kinematics, inverse_kinematics
    :member-order: bysource

Saved poses
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: move_pose_saved, get_pose_saved, save_pose, delete_pose, get_saved_pose_list
    :member-order: bysource

Pick & place
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: pick_from_pose, place_from_pose, pick_and_place
    :member-order: bysource

Trajectories
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: execute_trajectory_from_poses, execute_trajectory_from_poses_and_joints, execute_trajectory_saved, get_trajectory_saved,
              save_trajectory, delete_trajectory, get_saved_trajectory_list
    :member-order: bysource

Tools
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: get_current_tool_id, update_tool, grasp_with_tool,release_with_tool,
              open_gripper, close_gripper, pull_air_vacuum_pump, push_air_vacuum_pump,
              setup_electromagnet, activate_electromagnet, deactivate_electromagnet,
              enable_tcp, set_tcp, reset_tcp
    :member-order: bysource

Hardware
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: set_pin_mode, digital_write, digital_read,
              get_hardware_status, get_digital_io_state
    :member-order: bysource

Conveyor Belt
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: set_conveyor, unset_conveyor, control_conveyor
    :member-order: bysource

Vision
^^^^^^^^^^^^^

.. autoclass:: NiryoRosWrapper
    :members: get_compressed_image, get_target_pose_from_rel, get_target_pose_from_cam,
              vision_pick_w_obs_joints, vision_pick_w_obs_pose, vision_pick, move_to_object, detect_object, get_camera_intrinsics,
              save_workspace_from_poses, save_workspace_from_points,
              delete_workspace, get_workspace_ratio, get_workspace_list, get_workspace_poses,
              set_brightness, set_contrast, set_saturation
    :member-order: bysource


Enums
------------------------------------

.. automodule:: niryo_robot_python_ros_wrapper.ros_wrapper_enums
    :members:
    :undoc-members:
    :exclude-members: CommandEnum
    :member-order: bysource
