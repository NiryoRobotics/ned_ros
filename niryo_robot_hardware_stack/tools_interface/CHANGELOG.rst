^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tools_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'test_dev_thuc' into ned2_devel_tests
* make run test more easier with cmake arg
* Merge branch 'post_tests_corrections' into 'ned2_devel'
  merge before tag
  See merge request `niryo/niryo-one-s/ned_ros_stack!188 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/188>`_
* merge before tag
* Merge remote-tracking branch 'origin/motor_limit_dev' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/motor_limit_dev' into ned2_vaml
* Merge branch 'fix_unstable_branch' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* Merge branch 'motor_limit_dev' into fix_unstable_branch
* config joints for ned + one
* fix reboot tool and protect joint by limit
* Merge branch 'clement_various_optims' into 'ned2_devel'
  Clement various optims
  See merge request `niryo/niryo-one-s/ned_ros_stack!178 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/178>`_
* Clement various optims
* Merge branch 'reboot_correction' into ned2_devel
* sync and single queue wait improvement
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* update reboot hardware for end effector
* Merge branch 'update_status' into 'ned2_devel'
  Update status
  See merge request `niryo/niryo-one-s/ned_ros_stack!174 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/174>`_
* Update status
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_vaml
* Merge branch 'fix_simulation_robot' into 'ned2_devel'
  fix simulation robot
  See merge request `niryo/niryo-one-s/ned_ros_stack!169 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/169>`_
* fix simulation robot
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'unittests-stack' into ned2_devel
* tool and ttl driver tests
* Merge branch 'sync_read_consec_bytes' into 'ned2_devel'
  Sync read consec bytes
  See merge request `niryo/niryo-one-s/ned_ros_stack!167 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/167>`_
* Sync read consec bytes
* add tool name in tools_interface config file
* add tool hw status in hardware_status topic
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* roslint
* Merge branch 'tools_for_ned_2' into 'ned2_devel'
  Tools for ned 2
  See merge request `niryo/niryo-one-s/ned_ros_stack!156 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/156>`_
* tools for ned 2, need to be tested on ned1
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
* Merge branch 'fw_changes_integration' into december_candidate
* Merge branch 'december_candidate' into fw_changes_integration
* Add velocity  in joint state publisher
* Merge branch 'hw_stack_rework' into 'december_candidate'
  Hw stack rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!146 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/146>`_
* Hw stack rework
* using unique pointer instead of shared pointer for cmds used
* Merge branch 'clang_only_almost_everything' into december_candidate
* roslint
* Corrected anything I could with clang tidy
* more clang tidy
* correct nearly everything. Need to test
* Merge branch 'rework_ros_timers' into 'december_candidate'
  add ros timer in all publishers except conveyor
  See merge request `niryo/niryo-one-s/ned_ros_stack!139 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/139>`_
* add ros timer in all publishers except conveyor
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
* Merge branch 'tests_simulation_rework' into 'december_candidate'
  Changes to make tests simulation rework
  See merge request `niryo/niryo-one-s/ned_ros_stack!121 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/121>`_
* Changes to make tests simulation rework
* Merge branch 'open_close_tool' into 'december_candidate'
  Open close tool
  See merge request `niryo/niryo-one-s/ned_ros_stack!116 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/116>`_
* Merge branch 'december_candidate' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into december_candidate
* Merge branch 'hardware_version_refacto' into 'december_candidate'
  fine tuning of simulation_mode
  See merge request `niryo/niryo-one-s/ned_ros_stack!114 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/114>`_
* fine tuning of simulation_mode
* merge ee refactor
* Merge branch 'unit_tests_fix' into 'december_candidate'
  Unit tests fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!110 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/110>`_
* Unit tests fix
* Merge branch 'december_candidate' into can_manager_split
* add digital io service in end effector
* Merge branch 'io_panel_w_new_HS' into 'december_candidate'
  IO Panel + EE Panel + Top button + Wifi Button
  See merge request `niryo/niryo-one-s/ned_ros_stack!109 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/109>`_
* IO Panel + EE Panel + Top button + Wifi Button
* Merge branch 'conveyor_handle_disconnection' into 'december_candidate'
  Fix bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!108 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/108>`_
* Fix bugs
* Merge branch 'notool_affichage' into 'december_candidate'
  Fix no tool selected not show correctly in niryo studio
  See merge request `niryo/niryo-one-s/ned_ros_stack!106 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/106>`_
* Fix no tool selected not show correctly in niryo studio
* Merge branch 'calibration_refinement' into 'december_candidate'
  Calibration refinement
  See merge request `niryo/niryo-one-s/ned_ros_stack!103 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/103>`_
* remove unused config
* Merge branch 'fake_ned_addition' into 'december_candidate'
  Fake ned addition
  See merge request `niryo/niryo-one-s/ned_ros_stack!98 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/98>`_
* Fake ned addition
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'hw_stack_improve' into 'december_candidate'
  Hw stack improve
  See merge request `niryo/niryo-one-s/ned_ros_stack!96 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/96>`_
* Hw stack improve
* remove log
* changes to protect state tool + end effector
* cover toolState
* built
* Merge branch 'improve_movement_ned2' into 'december_candidate'
  Fix crash when motor connection problem
  See merge request `niryo/niryo-one-s/ned_ros_stack!95 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/95>`_
* Fix crash when motor connection problem
* Merge branch 'end_effector_driver_update' into december_candidate
* Move bus protocol inside states
  Add default ctor for states
  Remove bus protocol from to_motor_pos and to_rad_pos
  change addHardwareComponent into template
  add addHardwareDriver methode in ttl manager
  ttl manager should now have states has defined in the interface it was setup
* Merge branch 'december_candidate_fix_fake_drivers' into december_candidate
* unittests for hw stack with fake_driver
* small additions
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* small correction on ROS_WARN %lu not valid
  correction for fake moveit with niryo one
  small corrections on launch files in niryo_robot_bringup
  correction on urdf for niryo one incorrect
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'end_effector_package' into 'v3.2.0_with_HW_stack'
  End effector package
  See merge request `niryo/niryo-one-s/ned_ros_stack!69 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/69>`_
* fake tool + fix segment fault in logic of set tool
* fake ttl dxl ran with bring up launch file
* updated end effector. Changed end_effectors.yaml into tools_description.yaml
* small corrections after merge
* Merge branch 'v3.2.0_with_HW_stack' into end_effector_package
* Improvement for EndEffector. Add commands for end effector, change buttons with array of 3 buttons
* Merge branch 'common_unit_tests_additions_dev_thuc' into 'v3.2.0_with_HW_stack'
  tests run on hw
  See merge request `niryo/niryo-one-s/ned_ros_stack!66 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/66>`_
* tests run on hw
* Fix fail in service test tools interface
* merge conflict resolved for tests
* correction roslint + run can tests only when hw is ned
* Add end effector control loop (retrieve button states et publisher)
* pull/push air tests
* tool_interface tests
* add end_effector_state. temperature, voltage and error retrieved from ttl_interface_core
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* improvement of launch files. Begin work on EndEffectorInterfaceCore
* Add end effector package
* Merge branch 'hw-stack-new-end-effector' into 'v3.2.0_with_HW_stack'
  adapt new end effector
  See merge request `niryo/niryo-one-s/ned_ros_stack!60 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/60>`_
* adapt new end effector
* Merge remote-tracking branch 'origin/v3.2.0' into v3.2.0_niryo_one
* correction on wrong cmakelists for installing doc
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
* Merge branch 'v3.2.0' into system_software_api
* Simplifying single and synchronize motor cmds
* Merge corrections for joints_interface
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
* additions for tests. Works on dev machine but still failing on hw specific tests
* use parameter instead of attribute for starting services in nodes
* make ttldriver less dependent on dxl motors
* changed namespace to relative in all initParameters whenever possible
* finish integration of changes from v3.2.0_with_hw_stack
* more additions
* add modifs from hw interface
* add tools interface, ttl_driver, joints_interface
* add ros nodehandle to Core ctors
* add iinterfaceCore. Begin to adapt can_driver
* add tools interface confi
* restore docs changes (CMakeLists and dox)
* add ned2 hardware for all impacted packages
* add logging system in all py nodes
* modifications to be able to launch each node separately. Add debug logs for param loading in py files. Not finished yet
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* node handle modification on all nodes (access via relative path). Standardize init methods for interfaceCore nodes (add iinterface_core.hpp interface)
* standardize initialization methods
* correction on integration tests
* correction on CMakeLists not installing some executable at the correct place. Add installation of tcp_server for niryo_robot_user_interface
* add missing config files in install in CMakeLists.txt files
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
* correction on tools_interface_core
* correction on merge - compiling
* merging last 5 commits
* merge HW stack into v3.2.0. A new branch has been defined for this purpose
* Merge branch 'reboot_tool' into 'v3.2.0'
  Reboot tool
  See merge request `niryo/niryo-one-s/ned_ros_stack!23 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/23>`_
* Reboot tool
* made the code compliant with catkin_make_isolated
* correction on namespace naming
* merge v3.2.0 in moveit_add_collision
* correction on logging for tests. Add namespace into test launch files
* correction on conveyor
* switching to C++14
* correction on integration tests
* adding integration tests. Conveyor and tools integration test structure ok
* adding xsd link into launch files. Correcting tests for launch on dev machine
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* correction on read_custom_dxl_value.py script
* changing OpenGripper.srv, adding open_max_torque value
* add open_max_torque as param for tools_interface::OpenGripper::Request
* change stepper_driver to can_driver
* changing dynamixel_driver package into ttl_driver package to prepare the passage of steppers in ttl
* adding sizes for motor driver addresses in registers, adding draft for templatized driver
* change niryo_robot_debug into dxl_debug_tools
* update cpp unit tests
* correction on v2 config files
* set default conf to ned v1
* adding configurations for ned V1 and V2
* stable version, calibration ok, tool ok, stepper and dxl drivers ok, motor report ok
* stable version, set tool ok, dxl and stepper ok
* move publish cmd of stepper into dedicated thread
* standardize tool and conveyor interfaces
* corrected crash of stepper joints
* settup of the documentation generation using rosdoc_lite
* begin modifications of tool interface
* regressions solved. Pb of overflow on the sync command queue to be solved
* adding AbstractMotorCmd and IObject interfaces
* add ff1 and ff2 gain. Set pid in jointInterface using directly the dynamixel driver
* join StepperMotorEnum and DxlMotorEnum into MotorEnum; simplify jointInterface
* corrections for shared_ptr, unique_ptr, adding reallyAsync method in util, remove dependancy of jointInterface to drivers
* adding a common lib with model and utils subdirs. All classes refering to a State, a Cmd, an enum have been moved into model. Created a new enum structure, based on the CRTP design pattern
* improve log messages, begin reformating of stepper driver (const getters, private methods)
* bugs corrections on dynamixel driver
* small corrections following hw tests
* adding logger configuration file in niryo_robot_bringup
* optimized states, begin work on stepper and conveyor
* add namespaces to interfaces, change DxlMotorType into DxlMotorType_t to include conversions from and to string
* add const to vectors passed as references in methods but not intended to be modified
* adding const protection to getters methods of DxlMotorState
* use std::shared_ptr instead of boost::shared_ptr (needed for future ROS2 compatibility anyway)
* report modifs from dxlDriverEnhancement
* corrections on xl330 driver. Working
* corrections on error messages
* introduction of xc430 and xl330 into files. Small improvement of code
* Contributors: AdminIT, Clément Cocquempot, Justin, Minh Thuc, Pauline Odet, Thuc PHAM, Valentin Pitre, ValentinPitre, ccocquempot, f.dupuis, minhthuc

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
