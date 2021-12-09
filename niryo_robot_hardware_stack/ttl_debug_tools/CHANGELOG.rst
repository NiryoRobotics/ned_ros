^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ttl_debug_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'post_tests_corrections' into 'ned2_devel'
  merge before tag
  See merge request `niryo/niryo-one-s/ned_ros_stack!188 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/188>`_
* merge before tag
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* Merge branch 'queue_optimization' into ned2_devel
* Merge branch 'queue_optimization' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into queue_optimization
* Merge branch 'te_fixDxlWrite' into ned2_devel
* Merge branch 'te_fixDxlWrite' into queue_optimization
* fix dxl write
* retrieve test file for debug tools
* Merge remote-tracking branch 'origin/december_candidate' into tools_for_ned_2
* Merge branch 'ned2_devel' into 'december_candidate'
  end effector improvement + write executor trajectory (built + test with...
  See merge request `niryo/niryo-one-s/ned_ros_stack!154 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/154>`_
* end effector improvement + write executor trajectory (built + test with...
* test for ttl debug tool
* Merge branch 'ned2_devel' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into ned2_devel
* Merge branch 'moveit_ned2_dev' into ned2_devel
* roslint
* Merge remote-tracking branch 'origin/moveit_ned2_dev' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/include/ttl_driver/end_effector_reg.hpp
  #	niryo_robot_sound/config/default.yaml
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
* using only position of calculate cmd by moveit
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
* fix checking architecture in ttl debug tool
* clang tidy
* Merge branch 'december_candidate' into fw_changes_integration
* rework port ttl debug tool
* fix port ttl debug tool
* Merge branch 'clang_only_almost_everything' into december_candidate
* more clang tidy
* correct nearly everything. Need to test
* post merge conveyor improvement
* set FakeTtlData as shared ptr to have common list of ids
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* add linking to pthread. Not sure it is usefull
* Merge branch 'firmware_update' into 'december_candidate'
  Firmware update
  See merge request `niryo/niryo-one-s/ned_ros_stack!122 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/122>`_
* Firmware update
* Changes to make tests simulation rework
* roslint
* Merge branch 'ttl_debug_tool_for_sync' into december_candidate
* small corrections in cast
* sync rw worked
* add sync write and read
* Merge branch 'december_candidate' into calibration_refinement
* Merge branch 'conveyor_ttl' into december_candidate
* resolved unittest common + roslint
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'end_effector_driver_update' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into end_effector_driver_update
* Merge branch 'december_candidate' into 'end_effector_driver_update'
  December candidate
  See merge request `niryo/niryo-one-s/ned_ros_stack!93 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/93>`_
* December candidate
* Merge branch 'fix_bug_hw_december_candidate' into 'december_candidate'
  Fix some bugs hw stack december candidate
  See merge request `niryo/niryo-one-s/ned_ros_stack!92 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/92>`_
* Fix some bugs hw stack december candidate
* Merge branch 'ned2_proto_work' into 'december_candidate'
  Ned2 proto work
  See merge request `niryo/niryo-one-s/ned_ros_stack!90 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/90>`_
* Ned2 proto work
* Merge branch 'new-stepper-ttl-dev' into december_candidate
* post merge corrections
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
* updated end effector. Changed end_effectors.yaml into tools_description.yaml
* correction on ttl_debug_tools any_cast error
* correction on wrong cmakelists for installing doc
* Merge branch 'v3.2.0_with_HW_stack_upgrade_cicd' into 'v3.2.0_with_HW_stack'
  Update CICD + various fixes related to CICD testing
  See merge request `niryo/niryo-one-s/ned_ros_stack!55 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/55>`_
* Update CICD + various fixes related to CICD testing
  Fix catkin_lint errors + missing controller for simulation launches
* Merge branch 'v3.2.0_with_HW_stack_dev_thuc' into 'v3.2.0_with_HW_stack'
  Ajout du driver stepper TTL, generalisation des drivers et des commandes
  See merge request `niryo/niryo-one-s/ned_ros_stack!57 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/57>`_
* Change dxl_debug_tools into ttl_debug_tools
* Contributors: AdminIT, Clément Cocquempot, Minh Thuc, Thuc PHAM, Valentin Pitre, ccocquempot, clement cocquempot, f.dupuis, minh thuc, minhthuc, te

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
