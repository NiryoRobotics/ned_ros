January 2022 release - Niryo One & Ned compatibility - Hardware Stack refinement
===========================================================================================================

Requirements
-----------------------------------------------------------
Ubuntu packages
***********************************************************
* sqlite3
* ffmpeg
* build-essential
* catkin
* python-catkin-pkg
* python-pymodbus
* python-rosdistro
* python-rospkg
* python-rosdep-modules
* python-rosinstall python-rosinstall-generator
* python-wstool
* ros-melodic-moveit
* ros-melodic-control
* ros-melodic-controllers
* ros-melodic-tf2-web-republisher
* ros-melodic-rosbridge-server
* ros-melodic-joint-state-publisher-gui

Python libraries
***********************************************************

See *src/requirements_ned2.txt* file

Packages
-----------------------------------------------------------

New packages
***********************************************************
* niryo_robot_database
* niryo_robot_led_ring
* niryo_robot_metrics
* niryo_robot_reports
* niryo_robot_sound
* niryo_robot_status
* niryo_robot_hardware_stack/can_debug_tools
* niryo_robot_hardware_stack/common
* niryo_robot_hardware_stack/end_effector_interface
* niryo_robot_hardware_stack/serial

Renamed packages
***********************************************************
* niryo_ned_moveit_config_standalone becomes niryo_moveit_config_standalone
* niryo_ned_moveit_config_w_gripper1 becomes niryo_moveit_config_w_gripper1
* niryo_robot_hardware_stack/stepper_driver becomes niryo_robot_hardware_stack/can_driver
* niryo_robot_hardware_stack/dynamixel_driver becomes niryo_robot_hardware_stack/ttl_driver
* niryo_robot_hardware_stack/niryo_robot_debug becomes niryo_robot_hardware_stack/ttl_debug_tools

Removed packages
***********************************************************
* niryo_robot_serial_number
* niryo_robot_unit_tests
* niryo_robot_hardware_stack/fake_interface

Cleaning and Refactoring
***********************************************************
* roslint compliant
* catkin lint compliant for most part
* add xsd validation for launch files and package.xml files
* updated packages format to version 3
* updated c++ version to c++14
* clang and clazy compliance improvement
* rosdoc_lite set up in all packages
* catkin_tools compliant
* install space working
* sphinx_doc restructuration
* add hardware_version discrimination between ned, one and ned2
* add ned2 configuration files in all packages
* niryo_robot_arm_commmander refactoring
* niryo_robot_python_ros_wrapper refactoring

Features (for Ned and One only)
-----------------------------------------------------------
* add VERSION file at root 
* add CHANGELOG.rst in every package (using catkin_generate_changelog tool)
* update PID values for Dynamixels
* Replace fake interface by mock drivers for steppers and Dynamixels
* Add compatibility for TTL conveyor belts (upcoming)
* Add Ned2 features (upcoming)
* niryo_robot_bringup refactoring
* improve control loops for ttl_driver and joints interface

Know issues (for Ned and One only)
-----------------------------------------------------------

Can't scan 2 conveyors at the same time. Please scan the conveyors one by one.

Limitations
-----------------------------------------------------------
* Calibration deactivated on Simulated Ned and One
* Not officially supporting Ned2 hardware version
* Hotspot mode is always on by default on reboot for the Niryo One

Niryo Studio
-----------------------------------------------------------

New features
***********************************************************

- Network settings (DHCP / Static IP)
- Hardware detection One / Ned / Ned2
- Display TCP Speed
- Blockly - Dynamic blocks (Saved pose, workspace)

Bugs fix
***********************************************************
- Blockly - Conversion RAD / DEG in block


September release - New features batch
===========================================================

Features
-----------------------------------------------------------

Tool commander package
***********************************************************

- TCP service settings
  
  TCP.msg

  SetTCP.srv


Arm commander package
***********************************************************

- New movements available in ArmMoveCommand.msg

    linear pose

    shift linear pose

    trajectory


Python ROS Wrapper package
****************************  

- New movement functions available

    move linear pose

    linear pose

    jog pose shift

    jog joints shift

    shift linear pose

    execute trajectory from pose

- New TCP functions available

    set_tcp

    enable_tcp

    reset_tcp  

- New camera settings functions available

    set_brightness

    set_contrast

    set_saturation

Improvements
---------------------------

- Refactoring Tool Commander and Robot Commander packages.

    Remove Robot Commander package

    Reorder Robot Commander package between Tool Commander and Arm Commander packages. 

- Self collision detection

    Add self-collision detection via MoveIt.

- Collision detection

    Collision detection improvement on each joints.

    Learning mode activation in case of a collision. 