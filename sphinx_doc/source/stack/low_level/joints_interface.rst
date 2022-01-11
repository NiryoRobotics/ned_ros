Joints Interface
====================================

| This package handles packages related to the robot's joints controller.

| It provides an interface to :wiki_ros:`ros_control`.

Joints interface node
--------------------------
It is instantiated in :doc:`niryo_robot_hardware_interface` package.

It has been conceived to:
 - Interface robot's motors to joint trajectory controller, from :wiki_ros:`ros_control` package.
 - Create a controller manager, from :wiki_ros:`controller_manager` package, that provides the infrastructure to load, unload, start and stop controllers.
 - Interface with motors calibration.
 - Initialize motors parameters.


It belongs to the ROS namespace: |namespace_emphasize|.

Parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Joints Interface's default Parameters 
*************************************

.. list-table:: *default.yaml* file
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
      -  Default value
      -  Unit
   *  -  ``ros_control_loop_frequency``
      -  | Controls loop frequency.
      -  100
      -  Hz


Joints Interface's hardware specific Parameters 
**************************************************

These parameters are specific to the hardware version (Ned, Niryo One or Ned2).
This file comes in a different version for each hardware version. They are located in a directory of the hardware version name.

.. list-table:: *joints_params.yaml* file
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
      -  Supported Hardware versions
   *  -  ``joint_N/id``
      -  | Joint N (1, 2, 3, 4, 5 or 6) id
         | Default: -1 (invalid id)
      -  All versions
   *  -  ``joint_N/type``
      -  | Joint N (1, 2, 3, 4, 5 or 6) motor type among: "stepper", "xl320", "xl430", "fakeStepper" or "fakeDxl"
         | Default: ""
      -  All versions
   *  -  ``joint_N/bus``
      -  | Joint N (1, 2, 3, 4, 5 or 6) bus ("ttl" or "can")
         | Default: ""
      -  All versions

.. list-table:: *calibration_params.yaml* file
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
      -  Default value
      -  Unit
      -  Supported Hardware versions
   *  -  ``calibration_timeout``
      -  | Waiting time between 2 commands during the calibration process.
      -  30
      -  seconds
      -  All versions
   *  -  ``calibration_file``
      -  | File path where is saved motors calibration value.
      -  | */home/niryo/niryo_robot_saved_files*
         | */stepper_motor_calibration_offsets.txt*
      -  N.A.
      -  All versions
   *  -  ``stepper_N/id``
      -  | Stepper N (1, 2 or 3) id
      -  -1 (invalid id)
      -  N.A.
      -  All versions
   *  -  ``stepper_N/v_start``
      -  | Stepper N (1, 2 or 3) starting velocity for the acceleration profile
      -  1
      -  0.01 RPM
      -  Ned 2 only
   *  -  ``stepper_N/a_1``
      -  | Stepper N (1, 2 or 3) first acceleration for the acceleration profile
      -  0
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/v_1``
      -  | Stepper N (1, 2 or 3) first velocity for the acceleration profile
      -  0
      -  0.01 RPM
      -  Ned 2 only
   *  -  ``stepper_N/a_max``
      -  | Stepper N (1, 2 or 3) max acceleration for the acceleration profile
      -  6000
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/v_max``
      -  | Stepper N (1, 2 or 3) max velocity for the acceleration profile
      -  6
      -  0.01 RPM
      -    Ned 2 only
   *  -  ``stepper_N/d_max``
      -  | Stepper N (1, 2 or 3) max deceleration for the acceleration profile
      -  6000
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/d_1``
      -  | Stepper N (1, 2 or 3) last deceleration for the acceleration profile
      -  0
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/v_stop``
      -  | Stepper N (1, 2 or 3) stop velocity for the acceleration profile
      -  2
      -  0.01 RPM
      -  Ned 2 only
   *  -  ``stepper_N/stall_threshold``
      -  | Stepper N (1, 2 or 3) stall threshold for which we detect
         | the end of the joint course for the calibration process
      -  0
      -  N.A.
      -  Ned 2 only
   *  -  ``stepper_N/direction``
      -  | Stepper N (1, 2 or 3) direction for the calibration
         | (1 = same as motor direction, -1 = against motor direction)
      -  1
      -  N.A.
      -  All versions
   *  -  ``stepper_N/delay``
      -  | Stepper N (1, 2 or 3) delay
      -  0
      -  milliseconds
      -  All versions

.. list-table:: *dynamixels_params.yaml* file
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
      -  Unit
      -  Supported Hardware versions
   *  - ``dxl_N/offset_position``
      -  | Dynamixel N (1, 2 or 3) offset position for the zero position
         | Default: '0.0'
      -  Rad
      -  All versions
   *  - ``dxl_N/home_position``
      -  | Dynamixel N (1, 2 or 3) home position
         | Default: '0.0'
      -  Rad
      -  All versions
   *  - ``dxl_N/direction``
      -  | Dynamixel N (1, 2 or 3) direction (1 = ClockWise, -1 = Counter ClockWise)
         | Default: '1'
      -  N.A.
      -  All versions
   *  - ``dxl_N/limit_position_max``
      -  | Dynamixel N (1, 2 or 3) maximal position allowed
         | Default: '0.0'
      -  Rad
      -  All versions
   *  - ``dxl_N/limit_position_min``
      -  | Dynamixel N (1, 2 or 3) minimal position allowed
         | Default: '0.0'
      -  Rad
      -  All versions
   *  - ``dxl_N/position_P_gain``
      -  | Dynamixel N (1, 2 or 3) Proportional gain of the position PID controller 
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/position_I_gain``
      -  | Dynamixel N (1, 2 or 3) Integral gain of the position PID controller 
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/position_D_gain``
      -  | Dynamixel N (1, 2 or 3) Derivative gain of the position PID controller 
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/velocity_P_gain``
      -  | Dynamixel N (1, 2 or 3) Proportional gain of the velocity PID controller 
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/velocity_I_gain``
      -  | Dynamixel N (1, 2 or 3) Integral gain of the velocity PID controller 
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/FF1_gain``
      -  | Dynamixel N (1, 2 or 3) Feed Forward velocity Gain
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/FF2_gain``
      -  | Dynamixel N (1, 2 or 3) Feed Forward acceleration Gain
         | Default: '0.0'
      -  N.A.
      -  All versions
   *  - ``dxl_N/acceleration_profile``
      -  | Dynamixel N (1, 2 or 3) acceleration profile parameter [*]_
         | Default: '0.0'
      -  RPM²
      -  All versions
   *  - ``dxl_N/velocity_profile``
      -  | Dynamixel N (1, 2 or 3) velocity profile parameter
         | Default: '0.0'
      -  RPM
      -  All versions

.. [*] refers to the dedicated motor `reference documentation <https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#what-is-the-profile>`_.

.. list-table:: *steppers_params.yaml* file
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
      -  Unit
      -  Supported Hardware versions
   *  -  ``stepper_N/id``
      -  | Stepper N (1, 2 or 3) id
         | Default: -1 (invalid id)
      -  N.A.
      -  All versions
   *  -  ``stepper_N/gear_ratio``
      -  | Stepper N (1, 2 or 3) gear ratio
         | Default: 1
      -  N.A.
      -  Ned and One only
   *  -  ``stepper_N/max_effort``
      -  | Stepper N (1, 2 or 3) max effort
         | Default: 0
      -  N.A.
      -  Ned and One only
   *  -  ``stepper_N/motor_ratio``
      -  | Stepper N (1, 2 or 3) motor ratio for conversion into radian
         | Default: 1
      -  N.A.
      -  Ned 2 only
   *  -  ``stepper_N/offset_position``
      -  | Stepper N (1, 2 or 3) offset position to position limit min
         | Default: 0
      -  Rad
      -  All versions
   *  -  ``stepper_N/home_position``
      -  | Stepper N (1, 2 or 3) Home position of the motor
         | Default: 0
      -  Rad
      -  All versions
   *  -  ``stepper_N/limit_position_min``
      -  | Stepper N (1, 2 or 3) position limit min of the motor
         | Default: 0
      -  Rad
      -  All versions
   *  -  ``stepper_N/limit_position_max``
      -  | Stepper N (1, 2 or 3) position limit max of the motor
         | Default: 0
      -  Rad
      -  All versions
   *  -  ``stepper_N/direction``
      -  | Stepper N (1, 2 or 3) assembly direction of the motor (1 = CW, -1 = CCW)
         | Default: 1
      -  N.A.
      -  All versions
   *  -  ``stepper_N/v_start``
      -  | Stepper N (1, 2 or 3) starting velocity for the acceleration profile
         | Default: 1
      -  RPM
      -  Ned 2 only
   *  -  ``stepper_N/a_1``
      -  | Stepper N (1, 2 or 3) first acceleration for the acceleration profile
         | Default: 0
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/v_1``
      -  | Stepper N (1, 2 or 3) first velocity for the acceleration profile
         | Default: 0
      -  RPM
      -  Ned 2 only
   *  -  ``stepper_N/a_max``
      -  | Stepper N (1, 2 or 3) max acceleration for the acceleration profile
         | Default: 6000
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/v_max``
      -  | Stepper N (1, 2 or 3) max velocity for the acceleration profile
         | Default: 6
      -  RPM
      -  Ned 2 only
   *  -  ``stepper_N/d_max``
      -  | Stepper N (1, 2 or 3) max deceleration for the acceleration profile
         | Default: 6000
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/d_1``
      -  | Stepper N (1, 2 or 3) last deceleration for the acceleration profile
         | Default: 0
      -  RPM²
      -  Ned 2 only
   *  -  ``stepper_N/v_stop``
      -  | Stepper N (1, 2 or 3) stop velocity for the acceleration profile
         | Default: 2
      -  RPM
      -  Ned 2 only
   *  -  ``stepper_N/stall_threshold``
      -  | Stepper N (1, 2 or 3) stall threshold for which we detect the end of the joint course
         | Default:
      -  N.A.
      -  Ned 2 only

The velocity profiles for the Stepper motors (in *calibration_params.yaml* and *steppers_params.yaml* files) can be defined for TTL bus only (thus for Ned2 only).
They are defined according to the following graph:

.. figure:: ../../../images/stack/low_level/steppers_velocity_profiles.png
   :alt: TTL steppers velocity profiles
   :width: 600px
   :align: center

Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`hardware_interface <hardware_interface>`
- :wiki_ros:`controller_manager <controller_manager>`
- :doc:`ttl_driver`
- :doc:`can_driver`
- :doc:`../high_level/niryo_robot_msgs`
- :msgs_index:`control_msgs`

Services, Topics and Messages
-------------------------------------------------

Subscribed topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Joints Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory/result``
      -  :control_msgs:`FollowJointTrajectoryActionResult`
      -  Trajectory results from controller

Published topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Joints Interface's Published Topics
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/learning_mode/state``
      -  :std_msgs:`Bool`
      -  Learning mode state

Services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Joints Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/joints_interface/calibrate_motors``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:SetInt`
      -  Starts motors calibration - value can be 1 for auto calibration, 2 for manual
   *  -  ``/niryo_robot/joints_interface/request_new_calibration``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Resets motor calibration state to "uncalibrated". This will allow the user to ask a new calibration.
   *  -  ``niryo_robot/learning_mode/activate``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Changes learning mode (Free Motion) state. When learning mode is activated, torques are disabled and the joints can move freely.
   *  -  ``niryo_robot/joints_interface/steppers_reset_controller``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:Trigger`
      -  Resets the controller



Errors and warning messages
-------------------------------------------------


.. list-table:: List of Errors and warning messages
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Type
      -  Message
      -  Description
   *  -  Error
      -  JointHardwareInterface::init - Fail to add joint, return :
      -  The joint is not correctly initialized
   *  -  Error
      -  JointHardwareInterface::init - stepper state init failed
      -  The stepper state parameters are not correctly retrieved
   *  -  Error
      -  JointHardwareInterface::init - dxl state init failed
      -  The dynamixel state parameters are not correctly retrieved
   *  -  Error
      -  JointHardwareInterface::init - Dynamixel motors are not available on CAN Bus
      -  The robot wrongly tries to initialize a dynamixel motor for the CAN bus (works only on TTL)
   *  -  Error
      -  JointHardwareInterface::init - Fail to reboot motor id
      -  The motor failed to reboot. Try rebooting it again
   *  -  WARNING
      -  JointHardwareInterface::init - initialize stepper joint failure, return %d. Retrying
      -  Failed to initialize a stepper. Will try again up to 3 times
   *  -  WARNING
      -  JointHardwareInterface::init - add stepper joint failure, return %d. Retrying
      -  Failed to add a stepper joint. Will try again up to 3 times
   *  -  WARNING
      -  JointHardwareInterface::init - init dxl joint failure, return : %d. Retrying
      -  Failed to initialize a dynamixel joint. Will try again up to 3 times
   *  -  WARNING
      -  JointHardwareInterface::init - add dxl joint failure, return : %d. Retrying
      -  Failed to add a dynamixel joint. Will try again up to 3 times


.. |namespace_cpp| replace:: joints_interface
.. |namespace| replace:: /joints_interface/
.. |namespace_emphasize| replace:: ``/joints_interface/``
.. |package_path| replace:: ../../../../niryo_robot_hardware_stack/joints_interface

.. |br| raw:: html

     <br>