^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'post_tests_corrections' into 'ned2_devel'
  merge before tag
  See merge request `niryo/niryo-one-s/ned_ros_stack!188 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/188>`_
* merge before tag
* Merge branch 'standardize_drivers' into 'ned2_devel'
  standardize drivers, compile, launch and calib on ned, ned2 and one and simu
  See merge request `niryo/niryo-one-s/ned_ros_stack!182 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/182>`_
* standardize drivers, compile, launch and calib on ned, ned2 and one and simu
* Merge branch 'clement_compat_ned_one' into ned2_devel
* Merge branch 'ned2_devel' into clement_compat_ned_one
* Merge branch 'motor_limit_dev' into 'ned2_devel'
  Fix small bugs in hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!181 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/181>`_
* Fix small bugs in hw stack
* correction set Led for add tool ned
* Merge remote-tracking branch 'origin/motor_limit_dev' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/motor_limit_dev' into ned2_vaml
* vel + acc profile
* Merge branch 'fix_unstable_branch' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* correct sync read for mock stepper driver
* Merge branch 'motor_limit_dev' into fix_unstable_branch
* fix protection in dxl
* config joints for ned + one
* fix reboot tool and protect joint by limit
* protect joint go out of bound
* Merge branch 'motor_limit_dev' into 'ned2_devel'
  Fix joints limit
  See merge request `niryo/niryo-one-s/ned_ros_stack!177 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/177>`_
* Fix joints limit
* fix out of bound stepper
* Merge branch 'reboot_correction' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* update reboot hardware for end effector
* Merge branch 'update_status' into 'ned2_devel'
  Update status
  See merge request `niryo/niryo-one-s/ned_ros_stack!174 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/174>`_
* Update status
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'unittests-stack' into ned2_devel
* Merge branch 'sync_read_consec_bytes' into ned2_devel
* last corrections
* Merge branch 'sync_read_consec_bytes' into 'ned2_devel'
  Sync read consec bytes
  See merge request `niryo/niryo-one-s/ned_ros_stack!167 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/167>`_
* Sync read consec bytes
* fix fake ned and one and calibration can
* small changes
* small addition
* update calibration
* common test
* post merge changes
* add voltage conversion in state + read velocity only if torque off + optimize calib status
* update temperature type to uint8_t, add syncreadHwStatus
* Merge branch 'clement_lint' into ned2_devel
* roslint ok
* Merge branch 'optimize_calibration' into 'ned2_devel'
  Optimize calibration
  See merge request `niryo/niryo-one-s/ned_ros_stack!165 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/165>`_
* Optimize calibration
* correction to fix hotfix on conveyor direction
* hotfix : conveyor params direction = -1 + conveoyr state = inverse state
* change conveyor param direction
* Merge branch 'ned2_devel' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into ned2_devel
* hot fix : change goal_direction conveyor
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* Merge branch 'queue_optimization' into ned2_devel
* Merge branch 'queue_optimization' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into queue_optimization
* small addition
* Merge remote-tracking branch 'origin/december_candidate' into tools_for_ned_2
* Merge branch 'ned2_devel' into 'december_candidate'
  end effector improvement + write executor trajectory (built + test with...
  See merge request `niryo/niryo-one-s/ned_ros_stack!154 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/154>`_
* end effector improvement + write executor trajectory (built + test with...
* addsynccmd for stepper learning mode
* Merge branch 'moveit_ned2_dev' into 'ned2_devel'
  fix somes bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!153 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/153>`_
* fix somes bugs
* fix end effector delay on long push or hand hold action
* end effector improvement + write executor trajectory (built + test with simulation), not ready for pulling
* Merge remote-tracking branch 'origin/moveit_ned2_dev' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/include/ttl_driver/end_effector_reg.hpp
  #	niryo_robot_sound/config/default.yaml
* post merge correction_bus_ttl
* some changes for calibration
* add sync read for N blockes of bytes
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
* update display of stepper state
* change place of stall threshold
* addition
* add changes from end effector fw
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
* small improvement of stall threshold
* Merge branch 'rework_bus_ttl_blocked' into 'december_candidate'
  Rework bus ttl blocked
  See merge request `niryo/niryo-one-s/ned_ros_stack!151 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/151>`_
* Rework bus ttl blocked
* Merge branch 'stall_threshold_dev' into 'december_candidate'
  Stall threshold separated
  See merge request `niryo/niryo-one-s/ned_ros_stack!150 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/150>`_
* Stall threshold separated
* Merge branch 'simu_ned_bug_fix' into 'december_candidate'
  Simu ned bug fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!149 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/149>`_
* Simu ned bug fix
* clang tidy
* Merge branch 'fw_changes_integration' into december_candidate
* Merge branch 'december_candidate' into fw_changes_integration
* Add velocity  in joint state publisher
* add velocity profile service. Improve PID and velocity profile methods
* Merge branch 'hw_stack_rework' into 'december_candidate'
  Hw stack rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!146 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/146>`_
* Hw stack rework
* solved ned2 simulation
* update calibration to integrate stall threshold
* move for add joint + fix mutex scope in readStatus can interface
* using unique pointer instead of shared pointer for cmds used
* using move instead of copy for add cmds
* fix fake conveyor
* fix fake tool + copy/move ctor
* Merge branch 'mock_end_effector_corrections' into 'december_candidate'
  Mock end effector corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!145 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/145>`_
* Mock end effector corrections
* Merge branch 'recover_conveyor_logic' into 'december_candidate'
  refix the logic of direction in conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!144 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/144>`_
* refix the logic of direction in conveyor
* correction on tests
* Merge branch 'clang_only_almost_everything' into december_candidate
* Merge branch 'conveyor_direction_improvement' into 'december_candidate'
  add config for assembly direction of conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!142 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/142>`_
* add config for assembly direction of conveyor
* small addition
* roslint
* post merge changes
* roslint
* Corrected anything I could with clang tidy
* more clang tidy
* correct nearly everything. Need to test
* begin clang tidy on common. not sure to be very usefull...
* post merge conveyor improvement
* add default id in conveyor state to check validity
* Merge branch 'clang_tidy' into conveyor_improvement
* first commit clang tidy
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* add similar template structure for can
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* draft
* Merge branch 'december_candidate' into fake_driver_config
* Merge remote-tracking branch 'origin/december_candidate' into december_candidate
* Merge branch 'stack_corrections' into 'december_candidate'
  Stack corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!126 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/126>`_
* Stack corrections
* Merge branch 'learning_mode_rework' into december_candidate
* merge learning_mode_rework
* not push no action with empty queue
* ee resolved for long push and hand hold - delay reduced
* merge ee dev
* Merge branch 'end_effector_dev' into december_candidate
* ee worked all actions
* button state utilise shared ptr
* roslint
* Merge branch 'open_close_tool' into 'december_candidate'
  Open close tool
  See merge request `niryo/niryo-one-s/ned_ros_stack!116 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/116>`_
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
* handle state button of ee
* Merge branch 'can_manager_split' into december_candidate
* test handle button ee
* merge ee refactor
* Merge branch 'december_candidate' into can_manager_split
* add digital io service in end effector
* Merge branch 'conveyor_handle_disconnection' into 'december_candidate'
  Fix bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!108 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/108>`_
* Fix bugs
* roslint and catkin lint
* added abstract_can_driver and stepper_driver into can_driver package
* Merge branch 'package_standardization' into 'december_candidate'
  Package standardization
  See merge request `niryo/niryo-one-s/ned_ros_stack!107 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/107>`_
* Package standardization
* Merge branch 'calibration_refinement' into 'december_candidate'
  Calibration refinement
  See merge request `niryo/niryo-one-s/ned_ros_stack!103 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/103>`_
* Merge branch 'december_candidate' into calibration_refinement
* Merge branch 'conveyor_ttl' into december_candidate
* reformat all str() in states
* resolved unittest common + roslint
* resolve roslint, some points can't be solved
* add missing specialization for sync stepper ttl cmd
* update conveyor ttl, tested with fakeStepper
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'hw_stack_improve' into 'december_candidate'
  Hw stack improve
  See merge request `niryo/niryo-one-s/ned_ros_stack!96 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/96>`_
* Hw stack improve
* built
* Merge branch 'end_effector_driver_update' into december_candidate
* replace dynamic_cast to dynamic_pointer_cast in getHardwareState
* correction for invalid id fo steppers
* Merge branch 'end_effector_driver_update' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into end_effector_driver_update
* Merge branch 'december_candidate' into 'end_effector_driver_update'
  December candidate
  See merge request `niryo/niryo-one-s/ned_ros_stack!93 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/93>`_
* December candidate
* Move bus protocol inside states
  Add default ctor for states
  Remove bus protocol from to_motor_pos and to_rad_pos
  change addHardwareComponent into template
  add addHardwareDriver methode in ttl manager
  ttl manager should now have states has defined in the interface it was setup
* voltage conversion enhancement
* Merge branch 'fix_bug_hw_december_candidate' into 'december_candidate'
  Fix some bugs hw stack december candidate
  See merge request `niryo/niryo-one-s/ned_ros_stack!92 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/92>`_
* Fix some bugs hw stack december candidate
* end effector driver addresses correction
* Merge branch 'december_candidate_new_stepper_ttl_dev' into december_candidate
* Merge branch 'ned2_proto_work' into 'december_candidate'
  Ned2 proto work
  See merge request `niryo/niryo-one-s/ned_ros_stack!90 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/90>`_
* Ned2 proto work
* small update
* Merge branch 'december_candidate_update_fake_driver' into 'december_candidate'
  Fix conversion pos rad stepper ttl
  See merge request `niryo/niryo-one-s/ned_ros_stack!86 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/86>`_
* Fix conversion pos rad stepper ttl
* post merge changes
* Merge branch 'new-stepper-ttl-dev' into december_candidate
* Merge branch 'december_candidate_fix_fake_drivers' into december_candidate
* remove config for end effector interface
* Merge branch 'missing_visualization_bug' into 'december_candidate'
  Missing visualization bug
  See merge request `niryo/niryo-one-s/ned_ros_stack!84 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/84>`_
* Missing visualization bug
* unittests for hw stack with fake_driver
* post merge corrections
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* using simple controller for fake driver
* some changes for ttl stepper. need to test move joints
* small additions
  correction on rpi_model usage
  small correction
  standardize srdf and xacro files
  small correction
  small correction on ttl_debug_tools
  correction on tools_interface
  correction on new steppers_params format
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'end_effector_package' into 'v3.2.0_with_HW_stack'
  End effector package
  See merge request `niryo/niryo-one-s/ned_ros_stack!69 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/69>`_
* fake ttl dxl ran with bring up launch file
* correction for wrong config loaded
* small additions
* catkin lint
* correction on enum string value not good
* fake ttl_driver - need to be tested with joint interface
* Merge branch 'v3.2.0_with_HW_stack' into end_effector_package
* Improvement for EndEffector. Add commands for end effector, change buttons with array of 3 buttons
* Merge branch 'common_unit_tests_additions_dev_thuc' into 'v3.2.0_with_HW_stack'
  tests run on hw
  See merge request `niryo/niryo-one-s/ned_ros_stack!66 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/66>`_
* tests run on hw
* merge conflict resolved for tests
* correction roslint + run can tests only when hw is ned
* Add end effector control loop (retrieve button states et publisher)
* add end_effector_state. temperature, voltage and error retrieved from ttl_interface_core
* end effector driver implemented
* ttl unittest
* stepper ttl unittest
* Merge branch 'common_unit_tests_additions' into 'v3.2.0_with_HW_stack'
  Common unit tests additions
  See merge request `niryo/niryo-one-s/ned_ros_stack!61 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/61>`_
* Common unit tests additions
* common tests pass
* add fixture to tests multiple parameters for dxl states
* correction on wrong cmakelists for installing doc
* small correction and validation with lint and run_tests on dev machine
* Merge branch 'joints_driver_review' into v3.2.0_with_HW_stack
* fix changes from Clement (delete joint driver)
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
* remove ros in abstract_synchronize_motor_cmd
* Fix crash when launching file
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack_dev_thuc
* merge changes
* update conveyor interface for ttl
* catkin_lint and catkin_make install last corrections
* Change naming for can_driver and can_driver_core to can_manager and can_interface_core. Changed also cpp interface names to follow the new naming
* post merge changes
* Merge branch 'v3.2.0_with_HW_stack' into v3.2.0_with_HW_stack_dev_thuc
* Merge branch 'ttl_stepper_driver' into 'v3.2.0_with_HW_stack_dev_thuc'
  Changes in structure for drivers and commands.
  See merge request `niryo/niryo-one-s/ned_ros_stack!53 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/53>`_
* Changes in structure for drivers and commands.
* remove abstract_motor_cmd (introduce unneeded complexity)
* corrections for makint it compile
* Simplifying single and synchronize motor cmds
* Remove unused files from merge. Change back config names for can and ttl
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* remove dynamic_cast with sync cmd
* Fix missing ctor for single cmd interface used by stepper cmd
* remove dynamic_cast for single cmd
* make calibration work with ttl first version, joint_interface finish first changes (not tested)
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* simplify message if roslint not present
* Fix read cmd failed after set by joint hw interface
* correction on parameters for simulation launches
* update ttl_driver_core + fix can't use template cmd
* first version make ttl driver and joint interface more compatible with stepper
* make ttldriver less dependent on dxl motors
* changed namespace to relative in all initParameters whenever possible
* Merge branch 'resolve_roslint' into 'v3.2.0_with_HW_stack'
  Resolve roslint
  See merge request `niryo/niryo-one-s/ned_ros_stack!41 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/41>`_
* Resolve roslint
* Add velocity pid
* small additions (working robot)
* finish integration of changes from v3.2.0_with_hw_stack
* add ros nodehandle to Core ctors
* add iinterfaceCore. Begin to adapt can_driver
* restore docs changes (CMakeLists and dox)
* modifications to be able to launch each node separately. Add debug logs for param loading in py files. Not finished yet
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* node handle modification on all nodes (access via relative path). Standardize init methods for interfaceCore nodes (add iinterface_core.hpp interface)
* correction on integration tests
* correction on roslint
* correction on motor 5 inverted
* Merge branch 'cmakelist_additions_branch' into 'v3.2.0_with_HW_stack'
  merge into v3.2.0 with hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!29 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/29>`_
* small correction on tests
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
* correction on conveyor
* switching to C++14
* corrections on common unit tests
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* adding sizes for motor driver addresses in registers, adding draft for templatized driver
* change niryo_robot_debug into dxl_debug_tools
* update cpp unit tests
* stable version, calibration ok, tool ok, stepper and dxl drivers ok, motor report ok
* change calibration interface into calibration manager
* try corrections
* reducing time in control loops
* small correction
* standardize tool and conveyor interfaces
* corrected crash of stepper joints
* add comments for all methods of common package
* settup of the documentation generation using rosdoc_lite
* adding doc and tests building for dynamixel, stepper and common
* work in progress, calibration and stepper
* last stable commit
* refactorize calibration
* add interface IDriverCore. Add queue to StepperDriver
* corrections on new regressions, only joint 6 not working good, and pb of CAN BUS not detected
* regressions solved. Pb of overflow on the sync command queue to be solved
* add configuration into dxl state and stepper state. Inherit DxlState and StepperState from JointState. Add rad_pos_to_motor_pos() and to_rad_pos() in jointstate interface
* adding AbstractMotorCmd and IObject interfaces
* add ff1 and ff2 gain. Set pid in jointInterface using directly the dynamixel driver
* join StepperMotorEnum and DxlMotorEnum into MotorEnum; simplify jointInterface
* corrections for shared_ptr, unique_ptr, adding reallyAsync method in util, remove dependancy of jointInterface to drivers
* adding a common lib with model and utils subdirs. All classes refering to a State, a Cmd, an enum have been moved into model. Created a new enum structure, based on the CRTP design pattern
* Contributors: AdminIT, Clément Cocquempot, Etienne Rey-Coquais, Minh Thuc, Thuc PHAM, Valentin Pitre, ccocquempot, clement cocquempot, f.dupuis, minh thuc, minhthuc

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
