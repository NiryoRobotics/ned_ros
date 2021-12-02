^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package conveyor_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'cpu_interface_tests' into ned2_devel_tests
* add code coverage to conveyor
* Merge branch 'post_tests_corrections' into 'ned2_devel'
  merge before tag
  See merge request `niryo/niryo-one-s/ned_ros_stack!188 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/188>`_
* merge before tag
* correction for can_driver laucnh using simu_conveyor
* Merge branch 'ned2_devel' into clement_compat_ned_one
* Merge branch 'motor_limit_dev' into 'ned2_devel'
  Fix small bugs in hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!181 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/181>`_
* Fix small bugs in hw stack
* Merge remote-tracking branch 'origin/motor_limit_dev' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/motor_limit_dev' into ned2_vaml
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* Merge branch 'reboot_correction' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* update reboot hardware for end effector
* roslint
* Merge branch 'update_status' into 'ned2_devel'
  Update status
  See merge request `niryo/niryo-one-s/ned_ros_stack!174 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/174>`_
* Update status
* roslint
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'unittests-stack' into ned2_devel
* fix ned2 test in niryo hw interface package
* roslint
* conveyor test
* Merge branch 'sync_read_consec_bytes' into 'ned2_devel'
  Sync read consec bytes
  See merge request `niryo/niryo-one-s/ned_ros_stack!167 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/167>`_
* Sync read consec bytes
* update calibration
* Merge branch 'ned2_devel' into 'december_candidate'
  Ned2 devel
  See merge request `niryo/niryo-one-s/ned_ros_stack!160 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/160>`_
* Ned2 devel
* correction to fix hotfix on conveyor direction
* hotfix : conveyor params direction = -1 + conveoyr state = inverse state
* change conveyor param direction
* Merge branch 'ned2_devel' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into ned2_devel
* add ROS_INFO to convoyor interface
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* Merge branch 'conveyor_ttl_fix' into 'ned2_devel'
  Conveyor ttl fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!157 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/157>`_
* Conveyor ttl fix
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
* clang tidy
* Merge branch 'december_candidate' into fw_changes_integration
* Merge branch 'hw_stack_rework' into 'december_candidate'
  Hw stack rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!146 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/146>`_
* Hw stack rework
* using unique pointer instead of shared pointer for cmds used
* Merge branch 'recover_conveyor_logic' into 'december_candidate'
  refix the logic of direction in conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!144 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/144>`_
* refix the logic of direction in conveyor
* small correction clang
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
* correct nearly everything. Need to test
* post merge conveyor improvement
* fix crash when unplug conveyor
* worked with conveyor. no calibration needed
* add default id in conveyor state to check validity
* Merge branch 'clang_tidy' into conveyor_improvement
* first commit clang tidy
* Corrections for anormal error on conveyor deconnection
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* add similar template structure for can
* compiling
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
* start branch
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_led_ring/src/niryo_robot_led_ring/led_ring_commander.py
* Merge branch 'december_candidate' into fake_driver_config
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
* roslint
* Merge branch 'open_close_tool' into 'december_candidate'
  Open close tool
  See merge request `niryo/niryo-one-s/ned_ros_stack!116 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/116>`_
* Fix conveyor with new can driver
* Merge branch 'december_candidate' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into december_candidate
* Merge branch 'hardware_version_refacto' into 'december_candidate'
  fine tuning of simulation_mode
  See merge request `niryo/niryo-one-s/ned_ros_stack!114 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/114>`_
* fine tuning of simulation_mode
* Merge branch 'unit_tests_fix' into 'december_candidate'
  Unit tests fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!110 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/110>`_
* Unit tests fix
* Merge branch 'december_candidate' into can_manager_split
* Merge branch 'io_panel_w_new_HS' into 'december_candidate'
  IO Panel + EE Panel + Top button + Wifi Button
  See merge request `niryo/niryo-one-s/ned_ros_stack!109 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/109>`_
* IO Panel + EE Panel + Top button + Wifi Button
* Merge branch 'conveyor_handle_disconnection' into 'december_candidate'
  Fix bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!108 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/108>`_
* Fix bugs
* Merge branch 'package_standardization' into 'december_candidate'
  Package standardization
  See merge request `niryo/niryo-one-s/ned_ros_stack!107 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/107>`_
* Package standardization
* Merge branch 'fix_conveyor_ttl' into 'december_candidate'
  Fixed scan conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!102 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/102>`_
* Fixed scan conveyor
* fake_ned and fake_ned2 for conveyor
* Merge branch 'december_candidate' into calibration_refinement
* Merge branch 'conveyor_ttl' into december_candidate
* resolved unittest common + roslint
* resolve roslint, some points can't be solved
* modif accept 2 conveyors
* accept only 1 conveyor now
* update state for conveyor before added in can
* fix config conveyor
* update conveyor ttl, tested with fakeStepper
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'hw_stack_improve' into 'december_candidate'
  Hw stack improve
  See merge request `niryo/niryo-one-s/ned_ros_stack!96 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/96>`_
* Hw stack improve
* built
* Merge branch 'end_effector_driver_update' into december_candidate
* Move bus protocol inside states
  Add default ctor for states
  Remove bus protocol from to_motor_pos and to_rad_pos
  change addHardwareComponent into template
  add addHardwareDriver methode in ttl manager
  ttl manager should now have states has defined in the interface it was setup
* correction in progress for joints controller not loaded correctly
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'end_effector_package' into 'v3.2.0_with_HW_stack'
  End effector package
  See merge request `niryo/niryo-one-s/ned_ros_stack!69 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/69>`_
* corrections on conveyor conf
* Merge branch 'v3.2.0_with_HW_stack' into end_effector_package
* Merge branch 'common_unit_tests_additions_dev_thuc' into 'v3.2.0_with_HW_stack'
  tests run on hw
  See merge request `niryo/niryo-one-s/ned_ros_stack!66 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/66>`_
* tests run on hw
* add end_effector_state. temperature, voltage and error retrieved from ttl_interface_core
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* conveyor tests
* improvement of launch files. Begin work on EndEffectorInterfaceCore
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
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack_dev_thuc
* fix roslint
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack_dev_thuc
* merge changes
* update conveyor interface for ttl
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
* Merge branch 'catkin_lint_check' into 'v3.2.0'
  Fix all catkin_lint erros/warns/notices
  See merge request `niryo/niryo-one-s/ned_ros_stack!51 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/51>`_
* Fix all catkin_lint erros/warns/notices
* corrections for makint it compile
* Merge corrections for joints_interface
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* remove dynamic_cast for single cmd
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* simplify message if roslint not present
* retrieve architecture in CMakeLists
* correction on parameters for simulation launches
* Correction on all tests. Add tcp port as param for tcp server. Add protection to modbus server and tcp server (try catch)
* Fix getStates in can driver core
* first version make ttl driver and joint interface more compatible with stepper
* additions for tests. Works on dev machine but still failing on hw specific tests
* use parameter instead of attribute for starting services in nodes
* changed namespace to relative in all initParameters whenever possible
* Add velocity pid
* finish integration of changes from v3.2.0_with_hw_stack
* separate publishers into fake interface
* Last changes before merge
* add tools interface, ttl_driver, joints_interface
* repair conveyor and cpu
* add ros nodehandle to Core ctors
* add iinterfaceCore. Begin to adapt can_driver
* add tools interface confi
* restore docs changes (CMakeLists and dox)
* all nodes can launch separately on dev machine.
* add logging system in all py nodes
* modifications to be able to launch each node separately. Add debug logs for param loading in py files. Not finished yet
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* node handle modification on all nodes (access via relative path). Standardize init methods for interfaceCore nodes (add iinterface_core.hpp interface)
* small correction on conveyor node lanch
* standardize initialization methods
* add logs to conveyor interface
* correction on service name for conveyor tests
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
* merge HW stack into v3.2.0. A new branch has been defined for this purpose
* made the code compliant with catkin_make_isolated
* small correction
* correction on namespace naming
* correction on logging for tests. Add namespace into test launch files
* correction on conveyor
* switching to C++14
* correction on integration tests
* adding integration tests. Conveyor and tools integration test structure ok
* adding xsd link into launch files. Correcting tests for launch on dev machine
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* change stepper_driver to can_driver
* adding sizes for motor driver addresses in registers, adding draft for templatized driver
* change niryo_robot_debug into dxl_debug_tools
* update cpp unit tests
* adding configurations for ned V1 and V2
* standardize tool and conveyor interfaces
* corrected crash of stepper joints
* settup of the documentation generation using rosdoc_lite
* adding doc and tests building for dynamixel, stepper and common
* refactorize calibration
* add interface IDriverCore. Add queue to StepperDriver
* join StepperMotorEnum and DxlMotorEnum into MotorEnum; simplify jointInterface
* corrections for shared_ptr, unique_ptr, adding reallyAsync method in util, remove dependancy of jointInterface to drivers
* adding a common lib with model and utils subdirs. All classes refering to a State, a Cmd, an enum have been moved into model. Created a new enum structure, based on the CRTP design pattern
* bugs corrections on dynamixel driver
* adding logger configuration file in niryo_robot_bringup
* optimized states, begin work on stepper and conveyor
* add namespaces to interfaces, change DxlMotorType into DxlMotorType_t to include conversions from and to string
* adding const protection to getters methods of DxlMotorState
* use std::shared_ptr instead of boost::shared_ptr (needed for future ROS2 compatibility anyway)
* Contributors: AdminIT, Clément Cocquempot, Etienne Rey-Coquais, Justin, Minh Thuc, Thuc PHAM, Valentin Pitre, ValentinPitre, ccocquempot, f.dupuis, minhthuc

3.2.0 (2021-09-23)
------------------
* Merge branch 'develop' into 'master'
  v3.2.0
  See merge request `niryo/niryo-one-s/ned_ros_stack!113 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/113>`_
* Release September: v3.2.0
* Contributors: Ducatez Corentin

3.1.2 (2021-08-13)
------------------

3.1.1 (2021-06-21)
------------------

3.1.0 (2021-05-06)
------------------

3.0.0 (2021-01-25)
------------------
