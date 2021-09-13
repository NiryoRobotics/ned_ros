Joints_interface
====================================

| This package handles packages related to the robot's joints controller.
| It provides an interface to :wiki_ros:`ros_control`.

Joints interface node
--------------------------
The ROS Node is made to:
 - Interface robot's motors to joint trajectory controller, from :wiki_ros:`ros_control` package.
 - Create a controller manager, from :wiki_ros:`controller_manager` package, provides the infrastructure to load, unload, start and stop controllers.
 - Interface with motors calibration.
 - Initialize motors parameters.


Parameters - joints interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Joint Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``ros_control_loop_frequency``
      -  | Controls loop frequency.
         | Default: '100.0'
   *  -  ``publish_learning_mode_frequency``
      -  | Publishes rate for learning mode state.
         | Default: '2.0'
   *  -  ``calibration_timeout``
      -  | Waiting time between 2 commands during the calibration process.
         | Default: '30'
   *  -  ``calibration_file``
      -  | File directory where is saved motors calibration value.
         | Default: '/home/niryo/niryo_robot_saved_files/stepper_motor_calibration_offsets.txt'

Published topics - joints interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Joint Interface's Published Topics
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

Services - joints interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Joint Interface Package Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/joints_interface/calibrate_motors``
      -  :ref:`source/ros/niryo_robot_msgs:SetInt`
      -  Starst motors calibration - value can be 1 for auto calibration, 2 for manual
   *  -  ``/niryo_robot/joints_interface/request_new_calibration``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Unsets motors calibration
   *  -  ``niryo_robot/learning_mode/activate``
      -  :ref:`source/ros/niryo_robot_msgs:Trigger`
      -  Either activates or deactivates learning mode

Dependencies - joints interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :wiki_ros:`hardware_interface <hardware_interface>`
- :wiki_ros:`controller_manager <controller_manager>`
- :doc:`dynamixel_driver`
- :doc:`stepper_driver`
- :doc:`../ros/niryo_robot_msgs`
