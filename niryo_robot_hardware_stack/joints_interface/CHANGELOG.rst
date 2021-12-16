^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joints_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* add params from Florian
* Merge branch 'ajustement_pid' into 'ned2_devel'
  ajsutement pid
  See merge request `niryo/niryo-one-s/ned_ros_stack!191 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/191>`_
* ajsutement pid
* Merge branch 'post_tests_corrections' into 'ned2_devel'
  merge before tag
  See merge request `niryo/niryo-one-s/ned_ros_stack!188 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/188>`_
* merge before tag
* Merge branch 'ned2_ralenti' into 'ned2_devel'
  Ned2 ralenti
  See merge request `niryo/niryo-one-s/ned_ros_stack!187 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/187>`_
* Ned2 ralenti
* Merge branch 'various_optims' into ned2_devel
* add simu_conveyor, remove gripper for rviz, add use_gripper for simulation
* Merge branch 'standardize_drivers' into 'ned2_devel'
  standardize drivers, compile, launch and calib on ned, ned2 and one and simu
  See merge request `niryo/niryo-one-s/ned_ros_stack!182 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/182>`_
* standardize drivers, compile, launch and calib on ned, ned2 and one and simu
* new profiles based on new calculus
* Merge branch 'clement_compat_ned_one' into ned2_devel
* better config for smooth movements
* Merge remote-tracking branch 'origin/pid_dev_frequencies' into ned2_vaml
  # Conflicts:
  #	niryo_robot_moveit_config/niryo_moveit_config_standalone/config/ned2/joint_limits.yaml
* Merge branch 'ned2_devel' into clement_compat_ned_one
* Merge branch 'motor_limit_dev' into 'ned2_devel'
  Fix small bugs in hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!181 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/181>`_
* Fix small bugs in hw stack
* roslint
* put freq not met as warn throttle
* roslint
* remove can reboot for reboot all
* correction on reboot fail for can interface
* roslint and build tests correction
* new PID
* change go back home after calib for ned and one
* small config change
* small one config change'
* change dyn params
* one params change
* update with Thuc corrections
* Merge remote-tracking branch 'origin/motor_limit_dev' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/motor_limit_dev' into ned2_vaml
* new pid
* remove log file
* new vel + acc profile
* vel + acc profile
* fix calibration + no write when calibration needed
* hope it can be resolve pbl connection error, tested many times, have to be tested on other robots and be verified
* Merge branch 'fix_unstable_branch' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* roslint
* correct sync read for mock stepper driver
* Merge branch 'motor_limit_dev' into fix_unstable_branch
* roslint
* config joints for ned + one
* fix reboot tool and protect joint by limit
* reset calibration if calibration timeout
* stop robot if a collision detected by moveit
* protect joint go out of bound
* rework ned2 simulation
* Merge branch 'clement_various_optims' into 'ned2_devel'
  Clement various optims
  See merge request `niryo/niryo-one-s/ned_ros_stack!178 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/178>`_
* Clement various optims
* Merge branch 'motor_limit_dev' into 'ned2_devel'
  Fix joints limit
  See merge request `niryo/niryo-one-s/ned_ros_stack!177 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/177>`_
* Fix joints limit
* fix out of bound stepper
* Merge branch 'reboot_correction' into ned2_devel
* sync and single queue wait improvement
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* update reboot hardware for end effector
* roslint
* Merge branch 'update_status' into 'ned2_devel'
  Update status
  See merge request `niryo/niryo-one-s/ned_ros_stack!174 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/174>`_
* Update status
* Merge branch 'diagnostic_bug' into 'ned2_devel'
  Fix end effector
  See merge request `niryo/niryo-one-s/ned_ros_stack!173 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/173>`_
* Fix end effector
* remove duplicately checking of single/sync queue free in calibration
* remove activate 2 times in calibration
* Merge branch 'update_calibration' into 'ned2_devel'
  Update calibration
  See merge request `niryo/niryo-one-s/ned_ros_stack!172 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/172>`_
* Update calibration
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* Merge branch 'sync_read_profile_pid' into 'ned2_devel'
  Sync read profile pid
  See merge request `niryo/niryo-one-s/ned_ros_stack!171 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/171>`_
* Sync read profile pid
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'unittests-stack' into ned2_devel
* Merge branch 'sync_read_consec_bytes' into ned2_devel
* last corrections
* niryo_robot_hardware_interface tests
* joint_interface tests
* Merge branch 'sync_read_consec_bytes' into 'ned2_devel'
  Sync read consec bytes
  See merge request `niryo/niryo-one-s/ned_ros_stack!167 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/167>`_
* Sync read consec bytes
* calibration detected. Pb with go back to home sometime
* fix stepper's direction of niryo one
* fix fake ned and one and calibration can
* update calibration
* ned2 simulation reworked
* Merge branch 'clement_lint' into ned2_devel
* roslint ok
* Merge branch 'optimize_calibration' into 'ned2_devel'
  Optimize calibration
  See merge request `niryo/niryo-one-s/ned_ros_stack!165 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/165>`_
* Optimize calibration
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_devel
* Merge branch 'optimize_delay_ttl_bus' into 'ned2_devel'
  update hot fix conveyor id + delay if read ttl failed + ticket no message if a motor disconnected + best config velocity now
  See merge request `niryo/niryo-one-s/ned_ros_stack!164 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/164>`_
* update hot fix conveyor id + delay if read ttl failed + ticket no message if a motor disconnected + best config velocity now
* pid values param fix
* Merge branch 'ned2_devel' into 'december_candidate'
  Ned2 devel
  See merge request `niryo/niryo-one-s/ned_ros_stack!163 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/163>`_
* Ned2 devel
* Merge branch 'time_optimizations' into 'ned2_devel'
  Time optimizations
  See merge request `niryo/niryo-one-s/ned_ros_stack!162 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/162>`_
* Time optimizations
* some changes for improve freq r/w position and velocity profile
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* roslint
* Merge branch 'queue_optimization' into ned2_devel
* Merge branch 'queue_optimization' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into queue_optimization
* optimize limit and pid
* Merge branch 'improvement_movement' into 'ned2_devel'
  config for speed stepper
  See merge request `niryo/niryo-one-s/ned_ros_stack!155 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/155>`_
* Merge remote-tracking branch 'origin/improvement_movement' into tools_for_ned_2
* config for speed stepper
* add velocity in urdf
* put back torque off when calibrating (to prevent motor 1 from not moving enough to the left)
* correction on wrong params for learning mode
* Merge remote-tracking branch 'origin/december_candidate' into tools_for_ned_2
* debug
* debug
* change set torque on/off with sync write cmd
* Merge branch 'ned2_devel' into 'december_candidate'
  end effector improvement + write executor trajectory (built + test with...
  See merge request `niryo/niryo-one-s/ned_ros_stack!154 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/154>`_
* end effector improvement + write executor trajectory (built + test with...
* sync write try correction
* addsynccmd for stepper learning mode
* change profile values
* add mutex to addsinglecmdtoqueue
* Merge branch 'ned2_devel' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into ned2_devel
* draft profile velocity
* Merge branch 'moveit_ned2_dev' into ned2_devel
* roslint
* Merge branch 'moveit_ned2_dev' into 'ned2_devel'
  fix somes bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!153 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/153>`_
* fix somes bugs
* standardize serial package
* Merge branch 'ned2_devel' into moveit_ned2_dev
* fix ticket return previous position after the calibration
* remove dynamic reconfigure
* change back to one cfg message only
* Merge branch 'december_candidate' into moveit_ned2_dev
* add dynamic reconfigure for the 3 steppers
* sync read velocity for one driver instead of sync read on multiples driver
* change steppers_config.cfg to steppers.cfg
* Merge branch 'etienne_debug' into 'ned2_devel'
  Etienne debug
  See merge request `niryo/niryo-one-s/ned_ros_stack!152 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/152>`_
* Merge branch 'etienne_debug' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into etienne_debug
* add command for velocity profile
* uncomment lines in configCallback
* hddkfk
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into etienne_debug
* fggf
* fix some learning mode behaviours
* fix some learning mode behaviours
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into moveit_ned2_dev
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into etienne_debug
* Merge remote-tracking branch 'origin/moveit_ned2_dev' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/include/ttl_driver/end_effector_reg.hpp
  #	niryo_robot_sound/config/default.yaml
* post merge correction_bus_ttl
* correction on simulation for ned2
* some changes for calibration
* add sync read for N blockes of bytes
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
* using only position of calculate cmd by moveit
* add correction
* change place of stall threshold
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
* roslint + catkin lint
* Merge branch 'fw_changes_integration' into december_candidate
* post merge corrections
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
* learning mode hardware stack
* Merge branch 'clang_only_almost_everything' into december_candidate
* roslint
* Corrected anything I could with clang tidy
* more clang tidy
* correct nearly everything. Need to test
* begin clang tidy on common. not sure to be very usefull...
* post merge conveyor improvement
* Merge branch 'rework_ros_timers' into 'december_candidate'
  add ros timer in all publishers except conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!139 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/139>`_
* add ros timer in all publishers except conveyor
* roslint + catkin lint
* Merge branch 'Learning_mode_ned2' into sound_led_minor_improvements
* compiling
* set FakeTtlData as shared ptr to have common list of ids
* Fix lint errors... again
* Fix lint error
* Rework learning mode for ned 2
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
* Merge branch 'corrections_clement' into december_candidate
* correction du "marteau piqueur"
* Merge branch 'fake_can_dev' into 'december_candidate'
  Fake can driver
  See merge request `niryo/niryo-one-s/ned_ros_stack!124 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/124>`_
* Fake can driver
* Merge branch 'tests_simulation_rework' into 'december_candidate'
  Changes to make tests simulation rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!121 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/121>`_
* Changes to make tests simulation rework
* Merge branch 'learning_mode_rework' into december_candidate
* merge learning_mode_rework
* use single cmds instead of sync for torque enable
* post merge correction. Compiling
* roslint
* Merge branch 'stepper_acceleration' into 'december_candidate'
  Stepper acceleration
  See merge request `niryo/niryo-one-s/ned_ros_stack!115 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/115>`_
* Stepper acceleration
* Merge branch 'december_candidate' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into december_candidate
* Merge branch 'hardware_version_refacto' into 'december_candidate'
  fine tuning of simulation_mode
  See merge request `niryo/niryo-one-s/ned_ros_stack!114 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/114>`_
* fine tuning of simulation_mode
* Merge branch 'december_candidate' into can_manager_split
* Merge branch 'io_panel_w_new_HS' into 'december_candidate'
  IO Panel + EE Panel + Top button + Wifi Button
  See merge request `niryo/niryo-one-s/ned_ros_stack!109 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/109>`_
* IO Panel + EE Panel + Top button + Wifi Button
* add small sleep
* Merge branch 'package_standardization' into 'december_candidate'
  Package standardization
  See merge request `niryo/niryo-one-s/ned_ros_stack!107 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/107>`_
* Package standardization
* Merge branch 'calibration_refinement' into 'december_candidate'
  Calibration refinement
  See merge request `niryo/niryo-one-s/ned_ros_stack!103 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/103>`_
* Merge branch 'december_candidate' into calibration_refinement
* check validity of command before sync command
* Merge branch 'conveyor_ttl' into december_candidate
* reformat all str() in states
* resolved unittest common + roslint
* remove unused parameters
* remove unused config
* add missing specialization for sync stepper ttl cmd
* calibration manager cleaned
* refacto of calibration manager
* improve a bit calibration
* Merge branch 'led_ring_w_new_HS' into 'december_candidate'
  Led Ring
  See merge request `niryo/niryo-one-s/ned_ros_stack!100 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/100>`_
* Led Ring
* Merge branch 'cleaning_config_ned2' into december_candidate
* small correction
* move steppers config from can_driver to joints_interface
* Merge branch 'fake_ned_addition' into 'december_candidate'
  Fake ned addition
  See merge request `niryo/niryo-one-s/ned_ros_stack!98 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/98>`_
* Fake ned addition
* move config files from ttl_manager to joints_interface
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'hw_stack_improve' into 'december_candidate'
  Hw stack improve
  See merge request `niryo/niryo-one-s/ned_ros_stack!96 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/96>`_
* Hw stack improve
* built
* Merge branch 'improve_movement_ned2' into 'december_candidate'
  Fix crash when motor connection problem
  See merge request `niryo/niryo-one-s/ned_ros_stack!95 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/95>`_
* Fix crash when motor connection problem
* add hw and sw states from end effector in topics published
* Merge branch 'end_effector_driver_update' into december_candidate
* correction for invalid id fo steppers
* add addJoint to can_interface_core
* create addJoint in ttl_manager to add joints (same as setTool and setConveyor)
* Compiling, to be tested
* Move bus protocol inside states
  Add default ctor for states
  Remove bus protocol from to_motor_pos and to_rad_pos
  change addHardwareComponent into template
  add addHardwareDriver methode in ttl manager
  ttl manager should now have states has defined in the interface it was setup
* remove JointIdToJointName and getHwStatus
* voltage conversion enhancement
* Merge branch 'december_candidate_new_stepper_ttl_dev' into december_candidate
* small update
* Merge branch 'december_candidate_update_fake_driver' into 'december_candidate'
  Fix conversion pos rad stepper ttl
  See merge request `niryo/niryo-one-s/ned_ros_stack!86 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/86>`_
* Fix conversion pos rad stepper ttl
* Merge branch 'new-stepper-ttl-dev' into december_candidate
* Merge branch 'december_candidate_fix_fake_drivers' into december_candidate
* Merge branch 'missing_visualization_bug' into 'december_candidate'
  Missing visualization bug
  See merge request `niryo/niryo-one-s/ned_ros_stack!84 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/84>`_
* Missing visualization bug
* unittests for hw stack with fake_driver
* fix write single cmd
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* working !
* revert urdf names to niryo\_$(hardware_version)
* using simple controller for fake driver
* Merge branch 'fake_drivers_thuc' into fake_drivers
* correction in progress for joints controller not loaded correctly
* some changes for ttl stepper. need to test move joints
* handle fake calibration
* Remove Fake_interface
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
* changes for stepper ttl
* fake stepper ttl
* fake ttl dxl ran with bring up launch file
* correction post merge
* correction post merge
* Merge branch 'v3.2.0_niryo_one' into december_candidate
* correction for wrong config loaded
* catkin lint
* small corrections after merge
* Merge branch 'v3.2.0_with_HW_stack' into end_effector_package
* Improvement for EndEffector. Add commands for end effector, change buttons with array of 3 buttons
* Merge branch 'common_unit_tests_additions_dev_thuc' into 'v3.2.0_with_HW_stack'
  tests run on hw
  See merge request `niryo/niryo-one-s/ned_ros_stack!66 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/66>`_
* tests run on hw
* add end_effector_state. temperature, voltage and error retrieved from ttl_interface_core
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* joint_interface tests
* improvement of launch files. Begin work on EndEffectorInterfaceCore
* end effector driver implemented
* Add end effector package
* Merge remote-tracking branch 'origin/v3.2.0' into v3.2.0_niryo_one
* ned2 configuration changed (no xc430)
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
* merge changes
* catkin_lint and catkin_make install last corrections
* catkin_lint --ignore missing_directory -W2 src/ find no error
* catkin_make roslint corrected
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
* manual calib
* Merge branch 'catkin_lint_check' into 'v3.2.0'
  Fix all catkin_lint erros/warns/notices
  See merge request `niryo/niryo-one-s/ned_ros_stack!51 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/51>`_
* Fix all catkin_lint erros/warns/notices
* remove abstract_motor_cmd (introduce unneeded complexity)
* corrections for makint it compile
* one compatible
* Merge branch 'v3.2.0' into system_software_api
* Simplifying single and synchronize motor cmds
* fix xacro imports
* Merge corrections for joints_interface
* Niryo One config
* Remove unused files from merge. Change back config names for can and ttl
* Fix missing params when launching files
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* remove dynamic_cast with sync cmd
* remove dynamic_cast for single cmd
* make calibration work with ttl first version, joint_interface finish first changes (not tested)
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* remove can driver and dxl_debug tools dependencies to wiringpi
* typedef cmds
* simplify message if roslint not present
* revert changes to dxl tools
* retrieve architecture in CMakeLists
* correction on parameters for simulation launches
* update ttl_driver_core + fix can't use template cmd
* Correction on all tests. Add tcp port as param for tcp server. Add protection to modbus server and tcp server (try catch)
* first version make ttl driver and joint interface more compatible with stepper
* additions for tests. Works on dev machine but still failing on hw specific tests
* use parameter instead of attribute for starting services in nodes
* make ttldriver less dependent on dxl motors
* correction on calibration manager.
  Changed JointHardwareInterface and
  FakeJointHardwareInterface into camel case
* changed namespace to relative in all initParameters whenever possible
* Fix missmatch of name
* reorganize config files for motors
* Merge branch 'resolve_roslint' into 'v3.2.0_with_HW_stack'
  Resolve roslint
  See merge request `niryo/niryo-one-s/ned_ros_stack!41 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/41>`_
* Resolve roslint
* Add velocity pid
* finish integration of changes from v3.2.0_with_hw_stack
* adapt joints_interface
* change motors_param config files
* small correction to cmd
* Last changes before merge
* more additions
* add tools interface, ttl_driver, joints_interface
* add ros nodehandle to Core ctors
* add iinterfaceCore. Begin to adapt can_driver
* add tools interface confi
* change ttl config files
* retrieve changes for joints and fake interface
* change can config
* restore docs changes (CMakeLists and dox)
* add corrections to namespaces for drivers
* merged v3.2.0 into v3.2.0_with_HW_stack
* add ned2 hardware for all impacted packages
* all nodes can launch separately on dev machine.
* add logging system in all py nodes
* modifications to be able to launch each node separately. Add debug logs for param loading in py files. Not finished yet
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* node handle modification on all nodes (access via relative path). Standardize init methods for interfaceCore nodes (add iinterface_core.hpp interface)
* Merge branch 'jog_joints_ns' into 'v3.2.0'
  Jog joints from NS
  See merge request `niryo/niryo-one-s/ned_ros_stack!34 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/34>`_
* Jog joints from NS
* standardize initialization methods
* correction on integration tests
* Merge branch 'v3.2.0_with_HW_stack' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack
* correction on dxl config for NED v2
* add dynamixel params for Ned V1
* correction on CMakeLists not installing some executable at the correct place. Add installation of tcp_server for niryo_robot_user_interface
* correction on motor 5 inverted
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
* merge HW stack into v3.2.0. A new branch has been defined for this purpose
* small corrections on interface registered multiple time
* made the code compliant with catkin_make_isolated
* correction on namespace naming
* merge v3.2.0 in moveit_add_collision
* correction on logging for tests. Add namespace into test launch files
* correction on conveyor
* switching to C++14
* correction on integration tests
* adding integration tests. Conveyor and tools integration test structure ok
* adding xsd link into launch files. Correcting tests for launch on dev machine
* corrections on common unit tests
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* add open_max_torque as param for tools_interface::OpenGripper::Request
* correction on jointIdToJointName() method
* change stepper_driver to can_driver
* changing dynamixel_driver package into ttl_driver package to prepare the passage of steppers in ttl
* adding sizes for motor driver addresses in registers, adding draft for templatized driver
* change niryo_robot_debug into dxl_debug_tools
* update cpp unit tests
* correction on v2 config files
* set default conf to ned v1
* adding configurations for ned V1 and V2
* stable version, calibration ok, tool ok, stepper and dxl drivers ok, motor report ok
* change calibration interface into calibration manager
* stable version, set tool ok, dxl and stepper ok
* try corrections
* reducing time in control loops
* move publish cmd of stepper into dedicated thread
* small correction
* standardize tool and conveyor interfaces
* corrected crash of stepper joints
* add comments for all methods of common package
* settup of the documentation generation using rosdoc_lite
* adding doc and tests building for dynamixel, stepper and common
* last stable commit
* refactorize calibration
* add interface IDriverCore. Add queue to StepperDriver
* corrections on new regressions, only joint 6 not working good, and pb of CAN BUS not detected
* regressions solved. Pb of overflow on the sync command queue to be solved
* remove delay wake up for gdb attachement
* add configuration into dxl state and stepper state. Inherit DxlState and StepperState from JointState. Add rad_pos_to_motor_pos() and to_rad_pos() in jointstate interface
* adding AbstractMotorCmd and IObject interfaces
* add ff1 and ff2 gain. Set pid in jointInterface using directly the dynamixel driver
* join StepperMotorEnum and DxlMotorEnum into MotorEnum; simplify jointInterface
* corrections for shared_ptr, unique_ptr, adding reallyAsync method in util, remove dependency of jointInterface to drivers
* adding a common lib with model and utils subdirs. All classes refering to a State, a Cmd, an enum have been moved into model. Created a new enum structure, based on the CRTP design pattern
* improve log messages, begin reformating of stepper driver (const getters, private methods)
* bugs corrections on dynamixel driver
* small corrections following hw tests
* adding logger configuration file in niryo_robot_bringup
* optimized states, begin work on stepper and conveyor
* add namespaces to interfaces, change DxlMotorType into DxlMotorType_t to include conversions from and to string
* adding const protection to getters methods of DxlMotorState
* use std::shared_ptr instead of boost::shared_ptr (needed for future ROS2 compatibility anyway)
* corrections on xl330 driver. Working
* correct pb of PID (P and D inverted) in the initialisation of the dxl motors
* correction on xl330 driver
* modify dxl_driver and yaml files to use XC430 and XL330 motors
* introduction of xc430 and xl330 into files. Small improvement of code
* adding new abstract class XDriver to generalize the XLAAADriver classes. Add new XL330Driver and XC430Driver
* Contributors: AdminIT, Clément Cocquempot, Etienne Rey-Coquais, Florian Dupuis, Justin, Minh Thuc, Pauline Odet, Thuc PHAM, Valentin Pitre, ValentinPitre, ccocquempot, clement cocquempot, f.dupuis, minh thuc, minhthuc

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
* Merge remote-tracking branch 'origin/develop' into develop
* Merge branch 'Joint6_pid' into develop
* Script Test Production [skip ci]
* lower joint limits
* Change joint6 pid
* Contributors: Ducatez Corentin, Rémi Lux, ValentinPitre

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
* Merge remote-tracking branch 'origin/develop' into develop
* Merge branch 'Joint6_pid' into develop
* Script Test Production [skip ci]
* lower joint limits
* Change joint6 pid
* Contributors: Ducatez Corentin, Rémi Lux, ValentinPitre

3.0.0 (2021-01-25)
------------------
