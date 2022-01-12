^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package can_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* correction for can_driver laucnh using simu_conveyor
* Merge branch 'various_optims' into ned2_devel
* add simu_conveyor, remove gripper for rviz, add use_gripper for simulation
* Merge branch 'clement_compat_ned_one' into ned2_devel
* roslint
* put freq not met as warn throttle
* correction on reboot fail for can interface
* roslint and build tests correction
* Merge remote-tracking branch 'origin/motor_limit_dev' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/motor_limit_dev' into ned2_vaml
* standardise names
* Merge branch 'fix_unstable_branch' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* correct add joint can interface for ned and one simu
* Merge branch 'reboot_correction' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* update reboot hardware for end effector
* Merge branch 'update_status' into 'ned2_devel'
  Update status
  See merge request `niryo/niryo-one-s/ned_ros_stack!174 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/174>`_
* Update status
* Merge branch 'update_calibration' into 'ned2_devel'
  Update calibration
  See merge request `niryo/niryo-one-s/ned_ros_stack!172 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/172>`_
* Update calibration
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'unittests-stack' into ned2_devel
* Merge branch 'sync_read_consec_bytes' into ned2_devel
* last corrections
* niryo_robot_hardware_interface tests
* Merge branch 'sync_read_consec_bytes' into 'ned2_devel'
  Sync read consec bytes
  See merge request `niryo/niryo-one-s/ned_ros_stack!167 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/167>`_
* Sync read consec bytes
* fix fake ned and one and calibration can
* update calibration
* can driver tests
* add voltage conversion in state + read velocity only if torque off + optimize calib status
* small bug
* out of vector size error
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* Merge branch 'queue_optimization' into ned2_devel
* addsynccmd for stepper learning mode
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
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
* Merge branch 'simu_ned_bug_fix' into 'december_candidate'
  Simu ned bug fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!149 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/149>`_
* Simu ned bug fix
* clang tidy
* Merge branch 'fw_changes_integration' into december_candidate
* Merge branch 'december_candidate' into fw_changes_integration
* Add velocity  in joint state publisher
* Merge branch 'hw_stack_rework' into 'december_candidate'
  Hw stack rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!146 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/146>`_
* Hw stack rework
* roslint
* solved ned2 simulation
* move for add joint + fix mutex scope in readStatus can interface
* using unique pointer instead of shared pointer for cmds used
* using move instead of copy for add cmds
* clean fake driver
* fix fake conveyor
* std move in can drivers
* Merge branch 'recover_conveyor_logic' into 'december_candidate'
  refix the logic of direction in conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!144 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/144>`_
* refix the logic of direction in conveyor
* Merge branch 'clang_only_almost_everything' into december_candidate
* Merge branch 'conveyor_direction_improvement' into 'december_candidate'
  add config for assembly direction of conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!142 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/142>`_
* add config for assembly direction of conveyor
* roslint
* post merge changes
* Merge branch 'december_candidate' into clang_only_almost_everything
* Corrected anything I could with clang tidy
* Merge branch 'fix_conveyor_compatiblity' into 'december_candidate'
  conveyor improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!140 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/140>`_
* conveyor improvement
* more clang tidy
* enhance fakeCanData usage
* correct nearly everything. Need to test
* begin clang tidy on common. not sure to be very usefull...
* post merge conveyor improvement
* worked with conveyor. no calibration needed
* wrong error message during scan
* add default id in conveyor state to check validity
* add mutex when changing id to prevent wrong disconnection status
* correction for wrong calibration asked when conveyor connected
* Merge branch 'clang_tidy' into conveyor_improvement
* Merge branch 'conveyor_improvement' into clang_tidy
* can correction
* first commit clang tidy
* Corrections for anormal error on conveyor deconnection
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* add similar template structure for can
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* Merge branch 'roslaunch-standalone' into 'december_candidate'
  roslaunch standalone + add some comments
  See merge request `niryo/niryo-one-s/ned_ros_stack!132 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/132>`_
* roslaunch standalone + add some comments
* Merge branch 'fix_fake_driver' into 'december_candidate'
  Fix fake can driver
  See merge request `niryo/niryo-one-s/ned_ros_stack!131 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/131>`_
* Fix fake can driver
* draft
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_led_ring/src/niryo_robot_led_ring/led_ring_commander.py
* Merge branch 'fake_driver_config' into december_candidate
* roslint
* post merge corrections (roslint, catkin lint)
* Merge branch 'december_candidate' into fake_driver_config
* worked with ned + one
* Merge remote-tracking branch 'origin/december_candidate' into december_candidate
* Merge branch 'stack_corrections' into 'december_candidate'
  Stack corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!126 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/126>`_
* Stack corrections
* Merge branch 'fake_can_dev' into 'december_candidate'
  Fake can driver
  See merge request `niryo/niryo-one-s/ned_ros_stack!124 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/124>`_
* Fake can driver
* add linking to pthread. Not sure it is usefull
* roslint
* Merge branch 'open_close_tool' into 'december_candidate'
  Open close tool
  See merge request `niryo/niryo-one-s/ned_ros_stack!116 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/116>`_
* Merge branch 'stepper_acceleration' into 'december_candidate'
  Stepper acceleration
  See merge request `niryo/niryo-one-s/ned_ros_stack!115 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/115>`_
* Stepper acceleration
* Fix conveyor with new can driver
* Merge branch 'december_candidate' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into december_candidate
* Merge branch 'hardware_version_refacto' into 'december_candidate'
  fine tuning of simulation_mode
  See merge request `niryo/niryo-one-s/ned_ros_stack!114 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/114>`_
* fine tuning of simulation_mode
* handle state button of ee
* Merge branch 'can_manager_split' into december_candidate
* Merge branch 'unit_tests_fix' into 'december_candidate'
  Unit tests fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!110 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/110>`_
* Unit tests fix
* small sleep in can init
* Merge branch 'december_candidate' into can_manager_split
* Merge branch 'io_panel_w_new_HS' into 'december_candidate'
  IO Panel + EE Panel + Top button + Wifi Button
  See merge request `niryo/niryo-one-s/ned_ros_stack!109 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/109>`_
* IO Panel + EE Panel + Top button + Wifi Button
* small correction on calibration status not update
* protection for can driver if stepper driver not found
* Merge branch 'conveyor_handle_disconnection' into 'december_candidate'
  Fix bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!108 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/108>`_
* Fix bugs
* roslint and catkin lint
* added abstract_can_driver and stepper_driver into can_driver package
* Merge branch 'calibration_refinement' into 'december_candidate'
  Calibration refinement
  See merge request `niryo/niryo-one-s/ned_ros_stack!103 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/103>`_
* Merge branch 'december_candidate' into calibration_refinement
* Merge branch 'conveyor_ttl' into december_candidate
* update state for conveyor before added in can
* update conveyor ttl, tested with fakeStepper
* refacto of calibration manager
* small correction on can_manager
* Merge branch 'cleaning_config_ned2' into december_candidate
* move steppers config from can_driver to joints_interface
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
* Merge branch 'end_effector_driver_update' into december_candidate
* replace dynamic_cast to dynamic_pointer_cast in getHardwareState
* correction for invalid id fo steppers
* add getHardwareState into can_manager
* add addJoint to can_interface_core
* Move bus protocol inside states
  Add default ctor for states
  Remove bus protocol from to_motor_pos and to_rad_pos
  change addHardwareComponent into template
  add addHardwareDriver methode in ttl manager
  ttl manager should now have states has defined in the interface it was setup
* remove JointIdToJointName and getHwStatus
* end effector driver addresses correction
* Merge branch 'missing_visualization_bug' into 'december_candidate'
  Missing visualization bug
  See merge request `niryo/niryo-one-s/ned_ros_stack!84 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/84>`_
* Missing visualization bug
* post merge corrections
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* correction in progress for joints controller not loaded correctly
* small additions
  correction on rpi_model usage
  small correction
  standardize srdf and xacro files
  small correction
  small correction on ttl_debug_tools
  correction on tools_interface
  correction on new steppers_params format
* small correction on ROS_WARN %lu not valid
  correction for fake moveit with niryo one
  small corrections on launch files in niryo_robot_bringup
  correction on urdf for niryo one incorrect
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'end_effector_package' into 'v3.2.0_with_HW_stack'
  End effector package
  See merge request `niryo/niryo-one-s/ned_ros_stack!69 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/69>`_
* correction for wrong config loaded
* corrections on conveyor conf
* small additions
* catkin lint
* small corrections after merge
* Merge branch 'v3.2.0_with_HW_stack' into end_effector_package
* Improvement for EndEffector. Add commands for end effector, change buttons with array of 3 buttons
* Merge branch 'common_unit_tests_additions_dev_thuc' into 'v3.2.0_with_HW_stack'
  tests run on hw
  See merge request `niryo/niryo-one-s/ned_ros_stack!66 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/66>`_
* tests run on hw
* correction roslint + run can tests only when hw is ned
* add end_effector_state. temperature, voltage and error retrieved from ttl_interface_core
* can_driver tests on HW
* Fix motor report in can_interface
* improvement of launch files. Begin work on EndEffectorInterfaceCore
* end effector driver implemented
* correction on wrong cmakelists for installing doc
* Merge branch 'joints_driver_review' into v3.2.0_with_HW_stack
* Merge branch 'conveyor_adapt_ttl_stepper' into 'v3.2.0_with_HW_stack'
  Fix calibration failed when add conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!58 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/58>`_
* Remove joints_driver, simplify the process. Need to be tested
* Remove joints_driver, simplify the process. Need to be tested
* Fix calibration failed when add conveyor
* Merge branch 'v3.2.0_with_HW_stack_upgrade_cicd' into 'v3.2.0_with_HW_stack'
  Update CICD + various fixes related to CICD testing
  See merge request `niryo/niryo-one-s/ned_ros_stack!55 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/55>`_
* Update CICD + various fixes related to CICD testing
  Fix catkin_lint errors + missing controller for simulation launches
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' into 'v3.2.0_with_HW_stack'
  Ajout du driver stepper TTL, generalisation des drivers et des commandes
  See merge request `niryo/niryo-one-s/ned_ros_stack!57 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/57>`_
* adapt roslint
* Fix crash when launching file
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack_dev_thuc
* merge changes
* update conveyor interface for ttl
* catkin_lint and catkin_make install last corrections
* catkin_lint --ignore missing_directory -W2 src/ find no error
* catkin_make roslint corrected
* Change naming for can_driver and can_driver_core to can_manager and can_interface_core. Changed also cpp interface names to follow the new naming
* Merge branch 'v3.2.0_with_HW_stack' into v3.2.0_with_HW_stack_dev_thuc
* Merge branch 'ttl_stepper_driver' into 'v3.2.0_with_HW_stack_dev_thuc'
  Changes in structure for drivers and commands.
  See merge request `niryo/niryo-one-s/ned_ros_stack!53 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/53>`_
* Changes in structure for drivers and commands.
* remove abstract_motor_cmd (introduce unneeded complexity)
* corrections for makint it compile
* Simplifying single and synchronize motor cmds
* Remove unused files from merge. Change back config names for can and ttl
* Fix missing params when launching files
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* remove dynamic_cast with sync cmd
* remove dynamic_cast for single cmd
* Fixed calibration + ttl driver
* make calibration work with ttl first version, joint_interface finish first changes (not tested)
* post merge correction
* post merge correction
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* Merge branch 'v3.2.0_with_HW_stack' into 'relative_namespaces_branch'
  # Conflicts:
  #   niryo_robot_hardware_stack/ttl_driver/launch/ttl_driver.launch
  #   niryo_robot_hardware_stack/ttl_driver/test/ttl_driver_unit_tests.cpp
* remove can driver and dxl_debug tools dependencies to wiringpi
* simplify message if roslint not present
* Revert "remove dependency to ros for dxl_debug_tools"
  This reverts commit ba5537157ffd8e9618c202ddf84326f6c4bced7a.
* correction on can driver test
* retrieve architecture in CMakeLists
* correction on parameters for simulation launches
* Correction on all tests. Add tcp port as param for tcp server. Add protection to modbus server and tcp server (try catch)
* Fix getStates in can driver core
* first version make ttl driver and joint interface more compatible with stepper
* correction on ttl tests
* use parameter instead of attribute for starting services in nodes
* changed namespace to relative in all initParameters whenever possible
* fix delete failed id not use in driver
* Fix duplicate id + do ttldriver more generic
* Revert "set namespace to relative for ttl and can driver"
  This reverts commit 3a0c4c8c273896d42ecf4ca8ab656f330eac8c5a.
* set namespace to relative for ttl and can driver
* Fix missmatch of name
* reorganize config files for motors
* Merge branch 'resolve_roslint' into 'v3.2.0_with_HW_stack'
  Resolve roslint
  See merge request `niryo/niryo-one-s/ned_ros_stack!41 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/41>`_
* Resolve roslint
* Add velocity pid
* finish integration of changes from v3.2.0_with_hw_stack
* change motors_param config files
* can driver standard init
* Last changes before merge
* add ros nodehandle to Core ctors
* add iinterfaceCore. Begin to adapt can_driver
* change can config
* restore docs changes (CMakeLists and dox)
* add corrections to namespaces for drivers
* all nodes can launch separately on dev machine.
* add logging system in all py nodes
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* node handle modification on all nodes (access via relative path). Standardize init methods for interfaceCore nodes (add iinterface_core.hpp interface)
* small correction on conveyor node lanch
* standardize initialization methods
* corrections on tests
* correction on integration tests
* Merge branch 'v3.2.0_with_HW_stack' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack
* small correction on conveyor reset id
* correction on CMakeLists not installing some executable at the correct place. Add installation of tcp_server for niryo_robot_user_interface
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
* merge HW stack into v3.2.0. A new branch has been defined for this purpose
* made the code compliant with catkin_make_isolated
* correction on namespace naming
* correction on logging for tests. Add namespace into test launch files
* correction on conveyor
* switching to C++14
* correction on integration tests
* adding integration tests. Conveyor and tools integration test structure ok
* adding xsd link into launch files. Correcting tests for launch on dev machine
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* change stepper_driver to can_driver
* Contributors: AdminIT, Clément Cocquempot, Corentin Ducatez, Minh Thuc, Thuc PHAM, Valentin Pitre, ccocquempot, f.dupuis, minhthuc

3.2.0 (2021-09-23)
------------------

3.1.2 (2021-08-13)
------------------

3.1.1 (2021-06-21)
------------------

3.1.0 (2021-05-06)
------------------

3.0.0 (2021-01-25)
------------------
