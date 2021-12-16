^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package niryo_robot_hardware_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'post_tests_corrections' into 'ned2_devel'
  merge before tag
  See merge request `niryo/niryo-one-s/ned_ros_stack!188 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/188>`_
* merge before tag
* Merge branch 'various_optims' into ned2_devel
* add simu_conveyor, remove gripper for rviz, add use_gripper for simulation
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* Merge branch 'clement_various_optims' into 'ned2_devel'
  Clement various optims
  See merge request `niryo/niryo-one-s/ned_ros_stack!178 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/178>`_
* Clement various optims
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* Merge branch 'update_status' into 'ned2_devel'
  Update status
  See merge request `niryo/niryo-one-s/ned_ros_stack!174 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/174>`_
* Update status
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* Merge branch 'sync_read_profile_pid' into 'ned2_devel'
  Sync read profile pid
  See merge request `niryo/niryo-one-s/ned_ros_stack!171 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/171>`_
* Sync read profile pid
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'unittests-stack' into ned2_devel
* fix ned2 test in niryo hw interface package
* niryo_robot_hardware_interface tests
* Merge branch 'sync_read_consec_bytes' into 'ned2_devel'
  Sync read consec bytes
  See merge request `niryo/niryo-one-s/ned_ros_stack!167 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/167>`_
* Sync read consec bytes
* update calibration
* still have to correct calibration
* debug
* check if tool state is ok
* debug
* add tool hw status in hardware_status topic
* reset sleep on stack
* out of vector size error
* Merge branch 'ned2_devel' into 'december_candidate'
  Ned2 devel
  See merge request `niryo/niryo-one-s/ned_ros_stack!163 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/163>`_
* Ned2 devel
* Merge branch 'time_optimizations' into 'ned2_devel'
  Time optimizations
  See merge request `niryo/niryo-one-s/ned_ros_stack!162 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/162>`_
* Time optimizations
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into moveit_ned2_dev
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into etienne_debug
* add correction
* correction on crash
* Merge branch 'december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_bringup/launch/niryo_robot_base_common.launch.xml
  #	niryo_robot_hardware_stack/end_effector_interface/src/end_effector_interface_core.cpp
  #	niryo_robot_hardware_stack/joints_interface/include/joints_interface/joint_hardware_interface.hpp
  #	niryo_robot_hardware_stack/joints_interface/src/joints_interface_core.cpp
  #	niryo_robot_hardware_stack/ttl_driver/CMakeLists.txt
  #	niryo_robot_hardware_stack/ttl_driver/src/abstract_dxl_driver.cpp
  #	niryo_robot_hardware_stack/ttl_driver/src/abstract_end_effector_driver.cpp
  #	niryo_robot_hardware_stack/ttl_driver/src/abstract_motor_driver.cpp
  #	niryo_robot_hardware_stack/ttl_driver/src/abstract_stepper_driver.cpp
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
* Merge branch 'december_candidate' into fw_changes_integration
* Merge branch 'hw_stack_rework' into 'december_candidate'
  Hw stack rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!146 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/146>`_
* Hw stack rework
* using unique pointer instead of shared pointer for cmds used
* Merge branch 'mock_end_effector_corrections' into 'december_candidate'
  Mock end effector corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!145 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/145>`_
* Mock end effector corrections
* Merge branch 'clang_only_almost_everything' into december_candidate
* Corrected anything I could with clang tidy
* more clang tidy
* correct nearly everything. Need to test
* post merge conveyor improvement
* coquille
* Merge branch 'rework_ros_timers' into 'december_candidate'
  add ros timer in all publishers except conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!139 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/139>`_
* add ros timer in all publishers except conveyor
* Merge branch 'Learning_mode_ned2' into sound_led_minor_improvements
* compiling
* Merge branch 'Fix_bugs_hw_stack_dev' into 'december_candidate'
  Fix tickect calibration failed sometimes
  See merge request `niryo/niryo-one-s/ned_ros_stack!136 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/136>`_
* Fix tickect calibration failed sometimes
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* Merge branch 'roslaunch-standalone' into 'december_candidate'
  roslaunch standalone + add some comments
  See merge request `niryo/niryo-one-s/ned_ros_stack!132 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/132>`_
* roslaunch standalone + add some comments
* draft
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_led_ring/src/niryo_robot_led_ring/led_ring_commander.py
* roslint
* post merge corrections (roslint, catkin lint)
* Merge branch 'december_candidate' into fake_driver_config
* Merge branch 'corrections_clement' into december_candidate
* correction du "marteau piqueur"
* small addition
* Merge branch 'fake_can_dev' into 'december_candidate'
  Fake can driver
  See merge request `niryo/niryo-one-s/ned_ros_stack!124 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/124>`_
* Fake can driver
* small correction
* Merge branch 'cpu_simu_correction' into december_candidate
* prevent shutdown for simulation if temperature is too high
* Merge branch 'learning_mode_rework' into december_candidate
* merge learning_mode_rework
* merge ee dev
* Merge branch 'end_effector_dev' into december_candidate
* Merge branch 'stepper_acceleration' into 'december_candidate'
  Stepper acceleration
  See merge request `niryo/niryo-one-s/ned_ros_stack!115 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/115>`_
* Stepper acceleration
* Merge branch 'december_candidate' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into december_candidate
* fake end effector
* Merge branch 'hardware_version_refacto' into 'december_candidate'
  fine tuning of simulation_mode
  See merge request `niryo/niryo-one-s/ned_ros_stack!114 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/114>`_
* fine tuning of simulation_mode
* debug
* debug
* Merge branch 'unit_tests_fix' into 'december_candidate'
  Unit tests fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!110 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/110>`_
* Unit tests fix
* Merge branch 'december_candidate' into can_manager_split
* Merge branch 'io_panel_w_new_HS' into 'december_candidate'
  IO Panel + EE Panel + Top button + Wifi Button
  See merge request `niryo/niryo-one-s/ned_ros_stack!109 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/109>`_
* IO Panel + EE Panel + Top button + Wifi Button
* small correction
* protection for can driver if stepper driver not found
* Merge branch 'conveyor_handle_disconnection' into 'december_candidate'
  Fix bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!108 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/108>`_
* Fix bugs
* Merge branch 'december_candidate' into calibration_refinement
* Merge branch 'conveyor_ttl' into december_candidate
* resolved unittest common + roslint
* Merge branch 'fake_ned_addition' into 'december_candidate'
  Fake ned addition
  See merge request `niryo/niryo-one-s/ned_ros_stack!98 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/98>`_
* Fake ned addition
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'hw_stack_improve' into 'december_candidate'
  Hw stack improve
  See merge request `niryo/niryo-one-s/ned_ros_stack!96 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/96>`_
* Hw stack improve
* built
* add hw and sw states from end effector in topics published
* Merge branch 'end_effector_driver_update' into december_candidate
* create addJoint in ttl_manager to add joints (same as setTool and setConveyor)
* Move bus protocol inside states
  Add default ctor for states
  Remove bus protocol from to_motor_pos and to_rad_pos
  change addHardwareComponent into template
  add addHardwareDriver methode in ttl manager
  ttl manager should now have states has defined in the interface it was setup
* remove JointIdToJointName and getHwStatus
* remove getHwStatus from publish software version
* image version corrected
* voltage conversion enhancement
* Merge branch 'simu_gripper_dev' into 'december_candidate'
  simu gripper
  See merge request `niryo/niryo-one-s/ned_ros_stack!88 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/88>`_
* Merge branch 'ned2_proto_work' into 'december_candidate'
  Ned2 proto work
  See merge request `niryo/niryo-one-s/ned_ros_stack!90 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/90>`_
* Ned2 proto work
* simu gripper
* Merge branch 'december_candidate_fix_fake_drivers' into december_candidate
* Fix ttl_driver tests
* unittests for hw stack with fake_driver
* post merge corrections
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* working !
* revert urdf names to niryo\_$(hardware_version)
* using simple controller for fake driver
* Merge branch 'fake_drivers_thuc' into fake_drivers
* correction in progress for joints controller not loaded correctly
* some changes for ttl stepper. need to test move joints
* handle fake calibration
* continue adding fake config
* Remove Fake_interface
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'end_effector_package' into 'v3.2.0_with_HW_stack'
  End effector package
  See merge request `niryo/niryo-one-s/ned_ros_stack!69 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/69>`_
* fake ttl dxl ran with bring up launch file
* correction post merge
* correction post merge
* Merge branch 'v3.2.0_niryo_one' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into end_effector_package
* Improvement for EndEffector. Add commands for end effector, change buttons with array of 3 buttons
* Merge branch 'common_unit_tests_additions_dev_thuc' into 'v3.2.0_with_HW_stack'
  tests run on hw
  See merge request `niryo/niryo-one-s/ned_ros_stack!66 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/66>`_
* tests run on hw
* correction roslint + run can tests only when hw is ned
* add end_effector_state. temperature, voltage and error retrieved from ttl_interface_core
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* Hardware interface tests - need checking launch report
* improvement of launch files. Begin work on EndEffectorInterfaceCore
* end effector driver implemented
* catkin lint
* Merge remote-tracking branch 'origin/v3.2.0' into v3.2.0_niryo_one
* correction on wrong cmakelists for installing doc
* small correction and validation with lint and run_tests on dev machine
* Merge branch 'joints_driver_review' into v3.2.0_with_HW_stack
* Remove joints_driver, simplify the process. Need to be tested
* Remove joints_driver, simplify the process. Need to be tested
* Merge branch 'v3.2.0_with_HW_stack_upgrade_cicd' into 'v3.2.0_with_HW_stack'
  Update CICD + various fixes related to CICD testing
  See merge request `niryo/niryo-one-s/ned_ros_stack!55 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/55>`_
* Update CICD + various fixes related to CICD testing
  Fix catkin_lint errors + missing controller for simulation launches
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' into 'v3.2.0_with_HW_stack'
  Ajout du driver stepper TTL, generalisation des drivers et des commandes
  See merge request `niryo/niryo-one-s/ned_ros_stack!57 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/57>`_
* merge changes
* dxl_debug_tools corrections
* catkin_lint --ignore missing_directory -W2 src/ find no error
* Change naming for can_driver and can_driver_core to can_manager and can_interface_core. Changed also cpp interface names to follow the new naming
* Merge branch 'v3.2.0_with_HW_stack' into 'v3.2.0_with_HW_stack_dev_thuc'
  retrieve last V3.2.0 with hw stack changes
  See merge request `niryo/niryo-one-s/ned_ros_stack!56 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/56>`_
* retrieve last V3.2.0 with hw stack changes
* Post merge changes
* Merge branch 'v3.2.0_with_HW_stack' into v3.2.0_with_HW_stack_dev_thuc
* Merge branch 'ttl_stepper_driver' into 'v3.2.0_with_HW_stack_dev_thuc'
  Changes in structure for drivers and commands.
  See merge request `niryo/niryo-one-s/ned_ros_stack!53 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/53>`_
* Changes in structure for drivers and commands.
* Hardware version in hardware_status topic
* Merge branch 'catkin_lint_check' into 'v3.2.0'
  Fix all catkin_lint erros/warns/notices
  See merge request `niryo/niryo-one-s/ned_ros_stack!51 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/51>`_
* Fix all catkin_lint erros/warns/notices
* Merge branch 'v3.2.0' into system_software_api
* fix xacro imports
* Niryo One config
* Fix missing params when launching files
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* remove can driver and dxl_debug tools dependencies to wiringpi
* simplify message if roslint not present
* retrieve architecture in CMakeLists
* correction on parameters for simulation launches
* Correction on all tests. Add tcp port as param for tcp server. Add protection to modbus server and tcp server (try catch)
* first version make ttl driver and joint interface more compatible with stepper
* additions for tests. Works on dev machine but still failing on hw specific tests
* use parameter instead of attribute for starting services in nodes
* make ttldriver less dependent on dxl motors
* changed namespace to relative in all initParameters whenever possible
* Fix duplicate id + do ttldriver more generic
* Fix missmatch of name
* update launch file in hw interface for new config files
* Merge branch 'resolve_roslint' into 'v3.2.0_with_HW_stack'
  Resolve roslint
  See merge request `niryo/niryo-one-s/ned_ros_stack!41 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/41>`_
* Resolve roslint
* small additions (working robot)
* finish integration of changes from v3.2.0_with_hw_stack
* change motors_param config files
* more additions
* add modifs from hw interface
* add tools interface, ttl_driver, joints_interface
* add ros nodehandle to Core ctors
* add iinterfaceCore. Begin to adapt can_driver
* add tools interface confi
* change ttl config files
* retrieve changes for joints and fake interface
* change can config
* repair moveit_config
* repair hw stack launch files
* restore docs changes (CMakeLists and dox)
* add corrections to namespaces for drivers
* add namespace into hardware_interface_standalone.launch
* add ned2 hardware for all impacted packages
* all nodes can launch separately on dev machine.
* add logging system in all py nodes
* modifications to be able to launch each node separately. Add debug logs for param loading in py files. Not finished yet
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* node handle modification on all nodes (access via relative path). Standardize init methods for interfaceCore nodes (add iinterface_core.hpp interface)
* standardize initialization methods
* add logs to conveyor interface
* correction on integration tests
* correction on CMakeLists not installing some executable at the correct place. Add installation of tcp_server for niryo_robot_user_interface
* add missing config files in install in CMakeLists.txt files
* Merge branch 'cmakelist_additions_branch' into 'v3.2.0_with_HW_stack'
  merge into v3.2.0 with hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!29 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/29>`_
* small correction on doc installation
* Merge branch 'apply_roslint_branch' into 'cmakelist_additions_branch'
  merge rolint correction in cmake addition branch
  See merge request `niryo/niryo-one-s/ned_ros_stack!28 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/28>`_
* roslint done for cpp
* correction on doc install
* add documentation installation
* add template doc for each package. Add install operation in cmakelists.txt files
* merging last 5 commits
* merge HW stack into v3.2.0. A new branch has been defined for this purpose
* Merge branch 'refacto_tool_commander' into 'v3.2.0'
  Refacto tool commander
  See merge request `niryo/niryo-one-s/ned_ros_stack!22 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/22>`_
* Refacto tool commander
* made the code compliant with catkin_make_isolated
* Merge branch 'HWStack_improvement' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into HWStack_improvement
* correction on namespace naming
* merge v3.2.0 in moveit_add_collision
* First base for executing C++ test in Gitlab pipeline
* Merge branch 'multi_machine_moveit' into 'v3.2.0'
  Multi machine moveit
  See merge request `niryo/niryo-one-s/ned_ros_stack!15 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/15>`_
* Multi machine moveit
* correction on logging for tests. Add namespace into test launch files
* correction on conveyor
* switching to C++14
* correction on integration tests
* adding integration tests. Conveyor and tools integration test structure ok
* adding xsd link into launch files. Correcting tests for launch on dev machine
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* correction on jointIdToJointName() method
* change stepper_driver to can_driver
* change dynamixel_driver to ttl_driver everywhere
* changing dynamixel_driver package into ttl_driver package to prepare the passage of steppers in ttl
* change niryo_robot_debug into dxl_debug_tools
* update cpp unit tests
* correction on v2 config files
* adding configurations for ned V1 and V2
* stable version, calibration ok, tool ok, stepper and dxl drivers ok, motor report ok
* move publish cmd of stepper into dedicated thread
* standardize tool and conveyor interfaces
* corrected crash of stepper joints
* settup of the documentation generation using rosdoc_lite
* adding doc and tests building for dynamixel, stepper and common
* refactorize calibration
* add interface IDriverCore. Add queue to StepperDriver
* remove delay wake up for gdb attachement
* add configuration into dxl state and stepper state. Inherit DxlState and StepperState from JointState. Add rad_pos_to_motor_pos() and to_rad_pos() in jointstate interface
* adding AbstractMotorCmd and IObject interfaces
* join StepperMotorEnum and DxlMotorEnum into MotorEnum; simplify jointInterface
* corrections for shared_ptr, unique_ptr, adding reallyAsync method in util, remove dependency of jointInterface to drivers
* adding a common lib with model and utils subdirs. All classes refering to a State, a Cmd, an enum have been moved into model. Created a new enum structure, based on the CRTP design pattern
* bugs corrections on dynamixel driver
* small corrections following hw tests
* adding logger configuration file in niryo_robot_bringup
* optimized states, begin work on stepper and conveyor
* add namespaces to interfaces, change DxlMotorType into DxlMotorType_t to include conversions from and to string
* adding const protection to getters methods of DxlMotorState
* use std::shared_ptr instead of boost::shared_ptr (needed for future ROS2 compatibility anyway)
* adding new abstract class XDriver to generalize the XLAAADriver classes. Add new XL330Driver and XC430Driver
* Contributors: AdminIT, Clément Cocquempot, Corentin Ducatez, Etienne Rey-Coquais, Justin, Minh Thuc, Nicolas Guy, Pauline Odet, Thuc PHAM, Valentin Pitre, ValentinPitre, ccocquempot, f.dupuis, minh thuc, minhthuc

3.2.0 (2021-09-23)
------------------
* Merge branch 'develop' into 'master'
  v3.2.0
  See merge request `niryo/niryo-one-s/ned_ros_stack!113 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/113>`_
* Release September: v3.2.0
* Merge branch 'fix/SAV_dxl_1' into 'develop'
  Fix issue about unresponsive DXL motors with any commands in some situations
  See merge request `niryo/niryo-one-s/ned_ros_stack!5 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/5>`_
* Fix issue about unresponsive DXL motors with any commands in some situations
* Contributors: Ducatez Corentin

3.1.2 (2021-08-13)
------------------

3.1.1 (2021-06-21)
------------------
* Merge branch 'develop' into 'master'
  Release v3.1.0
  See merge request `niryo/niryo-one-s/ned_ros_stack!9 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/9>`_
* Release v3.1.0
* Contributors: Ducatez Corentin

3.1.0 (2021-05-06)
------------------
* Merge branch 'fix/SAV_dxl_1' into 'develop'
  Fix issue about unresponsive DXL motors with any commands in some situations
  See merge request `niryo/niryo-one-s/ned_ros_stack!5 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/5>`_
* Fix issue about unresponsive DXL motors with any commands in some situations
* Contributors: Ducatez Corentin

3.0.0 (2021-01-25)
------------------
