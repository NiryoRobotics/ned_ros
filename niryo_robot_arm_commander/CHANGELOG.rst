^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package niryo_robot_arm_commander
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'ned2_ralenti' into 'ned2_devel'
  Ned2 ralenti
  See merge request `niryo/niryo-one-s/ned_ros_stack!187 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/187>`_
* Ned2 ralenti
* merge corrections from thuc
* Merge branch 'standardize_drivers' into 'ned2_devel'
  standardize drivers, compile, launch and calib on ned, ned2 and one and simu
  See merge request `niryo/niryo-one-s/ned_ros_stack!182 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/182>`_
* standardize drivers, compile, launch and calib on ned, ned2 and one and simu
* fix gazebo simu
* compute and play traj
* video shoot move
* Merge remote-tracking branch 'origin/ned2_vaml' into ned2_vaml
* Merge branch 'draw_a_circle' into 'ned2_vaml'
  Draw a circle
  See merge request `niryo/niryo-one-s/ned_ros_stack!180 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/180>`_
* Draw a circle
* Merge remote-tracking branch 'origin/motor_limit_dev' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/motor_limit_dev' into ned2_vaml
* Merge branch 'fix_unstable_branch' into ned2_devel
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* oups
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* do not merge, test ameliorations
* Merge branch 'motor_limit_dev' into fix_unstable_branch
* remove delay in start node
* stop robot if a collision detected by moveit
* Merge branch 'clement_various_optims' into 'ned2_devel'
  Clement various optims
  See merge request `niryo/niryo-one-s/ned_ros_stack!178 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/178>`_
* Clement various optims
* test prod
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/december_candidate' into tools_for_ned_2
* Merge branch 'ned2_devel' into 'december_candidate'
  end effector improvement + write executor trajectory (built + test with...
  See merge request `niryo/niryo-one-s/ned_ros_stack!154 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/154>`_
* end effector improvement + write executor trajectory (built + test with...
* Merge branch 'ned2_devel' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into ned2_devel
* Merge branch 'moveit_ned2_dev' into ned2_devel
* roslint
* Merge branch 'moveit_ned2_dev' into 'ned2_devel'
  fix somes bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!153 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/153>`_
* fix somes bugs
* Merge branch 'ned2_devel' into moveit_ned2_dev
* resolv missing package when cross compile + using try catch to avoid if when read EE
* change steppers_config.cfg to steppers.cfg
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into moveit_ned2_dev
* Merge remote-tracking branch 'origin/sound_led_minor_improvements' into etienne_debug
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
* tcp speed
* Merge branch 'simu_ned_bug_fix' into 'december_candidate'
  Simu ned bug fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!149 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/149>`_
* Simu ned bug fix
* fix my bouze
* pass some functions in ros wrapper
* tcp velocity
* Merge branch 'Learning_mode_ned2' into sound_led_minor_improvements
* Rework learning mode for ned 2
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* Merge remote-tracking branch 'origin/december_candidate' into december_candidate
* Merge branch 'stack_corrections' into 'december_candidate'
  Stack corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!126 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/126>`_
* Stack corrections
* Merge branch 'ned_v2-sound' into 'december_candidate'
  Ned sound
  See merge request `niryo/niryo-one-s/ned_ros_stack!125 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/125>`_
* Ned sound
* Merge branch 'master' into december_candidate
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
* Merge branch 'conveyor_handle_disconnection' into 'december_candidate'
  Fix bugs
  See merge request `niryo/niryo-one-s/ned_ros_stack!108 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/108>`_
* Fix bugs
* Merge branch 'package_standardization' into 'december_candidate'
  Package standardization
  See merge request `niryo/niryo-one-s/ned_ros_stack!107 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/107>`_
* Package standardization
* Merge branch 'bringup_conrrection' into 'december_candidate'
  Bringup conrrection
  See merge request `niryo/niryo-one-s/ned_ros_stack!105 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/105>`_
* Bringup conrrection
* Merge branch 'december_candidate' into calibration_refinement
* Merge branch 'led_ring_w_new_HS' into 'december_candidate'
  Led Ring
  See merge request `niryo/niryo-one-s/ned_ros_stack!100 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/100>`_
* Led Ring
* Merge branch 'fake_ned_addition' into 'december_candidate'
  Fake ned addition
  See merge request `niryo/niryo-one-s/ned_ros_stack!98 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/98>`_
* Fake ned addition
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'release_septembre' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into release_septembre
* move error tolerances into specific hw directory
* Upgrade versions to v3.2.0
* Merge branch 'simu_gripper_dev' into 'december_candidate'
  simu gripper
  See merge request `niryo/niryo-one-s/ned_ros_stack!88 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/88>`_
* simu gripper
* post merge corrections
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'fix_release_septembre' into 'release_septembre'
  Fix release septembre
  See merge request `niryo/niryo-one-s/ned_ros_stack!80 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/80>`_
* Fix release septembre
* small correction for ned
* using simple controller for fake driver
* correction in progress for joints controller not loaded correctly
* Remove Fake_interface
* Merge branch 'v3.2.0_with_HW_stack' into 'december_candidate'
  V3.2.0 with hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!77 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/77>`_
* V3.2.0 with hw stack
* Merge branch 'release_septembre' into 'v3.2.0_with_HW_stack'
  Release septembre
  See merge request `niryo/niryo-one-s/ned_ros_stack!76 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/76>`_
* Release septembre
* small correction on ROS_WARN %lu not valid
  correction for fake moveit with niryo one
  small corrections on launch files in niryo_robot_bringup
  correction on urdf for niryo one incorrect
* Change collision detection tolerance
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'release_septembre' into v3.2.0_with_HW_stack
* Merge branch 'release_septembre' into v3.2.0_with_HW_stack
* Fix and clean jog controller
* Merge branch 'end_effector_package' into 'v3.2.0_with_HW_stack'
  End effector package
  See merge request `niryo/niryo-one-s/ned_ros_stack!69 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/69>`_
* Merge remote-tracking branch 'origin/v3.2.0' into release_septembre
* fix Nico's shit
* Fix package.xml name (oups merge)
* Merge branch develop
* correction post merge
* correction post merge
* Merge branch 'v3.2.0_niryo_one' into december_candidate
* Merge branch 'v3.2.0' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0
* correction for wrong config loaded
* Merge branch 'v3.2.0_with_HW_stack' into december_candidate
* Merge branch 'v3.2.0' into v3.2.0_with_HW_stack
* Merge branch 'ned_2' into iot_ned2
* Merge branch 'v3.2.0' into ned_2
* correct yaml tolerance
* put tolerance collision + jog_joint in param
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* Add end effector package
* catkin lint
* Merge remote-tracking branch 'origin/v3.2.0' into v3.2.0_niryo_one
* correction on wrong cmakelists for installing doc
* Merge branch 'v3.2.0_with_HW_stack_upgrade_cicd' into 'v3.2.0_with_HW_stack'
  Update CICD + various fixes related to CICD testing
  See merge request `niryo/niryo-one-s/ned_ros_stack!55 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/55>`_
* Update CICD + various fixes related to CICD testing
  Fix catkin_lint errors + missing controller for simulation launches
* merge changes
* Merge branch 'v3.2.0_with_HW_stack' into 'v3.2.0_with_HW_stack_dev_thuc'
  retrieve last V3.2.0 with hw stack changes
  See merge request `niryo/niryo-one-s/ned_ros_stack!56 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/56>`_
* retrieve last V3.2.0 with hw stack changes
* Post merge changes
* Merge branch 'v3.2.0_with_HW_stack' into v3.2.0_with_HW_stack_dev_thuc
* manual calib
* Merge branch 'catkin_lint_check' into 'v3.2.0'
  Fix all catkin_lint erros/warns/notices
  See merge request `niryo/niryo-one-s/ned_ros_stack!51 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/51>`_
* Fix all catkin_lint erros/warns/notices
* one compatible
* Merge branch 'v3.2.0' into system_software_api
* fix xacro collision for kinetic
* fix xacro imports
* Niryo One config
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* Merge branch 'v3.2.0' into v3.2.0_with_HW_stack
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* simplify message if roslint not present
* correction on install for commanders
* correction on parameters for simulation launches
* Merge branch 'v3.2.0' into clean_iot
* put CommandJog message in niryo_robot_msgs
* fix display traj
* changed namespace to relative in all initParameters whenever possible
* post merge corrections
* Merge branch 'resolve_roslint' into 'v3.2.0_with_HW_stack'
  Resolve roslint
  See merge request `niryo/niryo-one-s/ned_ros_stack!41 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/41>`_
* Resolve roslint
* Merge branch 'v3.2.0' into v3.2.0_with_HW_stack
* little fix
* remove print
* change name of variable
* fix several jogged joints
* fix jog joint
* fix jog joints
* fic acc/vel factor
* can driver standard init
* retrieve changes from all packages except hw stack
* small corrections
* missing hardware_version in launch files
* merged v3.2.0 into v3.2.0_with_HW_stack
* add ned2 hardware for all impacted packages
* all nodes can launch separately on dev machine.
* add logging system in all py nodes
* modifications to be able to launch each node separately. Add debug logs for param loading in py files. Not finished yet
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* check collision in jog just for jog joints
* Merge branch 'jog_joints_ns' into 'v3.2.0'
  Jog joints from NS
  See merge request `niryo/niryo-one-s/ned_ros_stack!34 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/34>`_
* Jog joints from NS
* Merge branch 'tcp_manager' into 'v3.2.0'
  Tcp manager
  See merge request `niryo/niryo-one-s/ned_ros_stack!18 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/18>`_
* Tcp manager
* Merge branch 'v3.2.0_with_HW_stack' of gitlab.com:niryo/niryo-one-s/ned_ros_stack into v3.2.0_with_HW_stack
* correction on roslint
* Fix rosdep for niryo_arm_commander (pkg robot_tools was renamed)
* Merge branch 'v3.2.0' into 'v3.2.0_with_HW_stack'
  jog tcp bug fix merge
  See merge request `niryo/niryo-one-s/ned_ros_stack!30 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/30>`_
* Merge branch 'cmakelist_additions_branch' into 'v3.2.0_with_HW_stack'
  merge into v3.2.0 with hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!29 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/29>`_
* small correction on doc installation
* Merge branch 'apply_roslint_branch' into 'cmakelist_additions_branch'
  merge rolint correction in cmake addition branch
  See merge request `niryo/niryo-one-s/ned_ros_stack!28 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/28>`_
* correction for python roslint
* Merge branch 'jogTCP-improvement' into 'v3.2.0'
  Jog tcp improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!25 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/25>`_
* bug fix pitch jog
* correction on doc install
* add documentation installation
* remove doc directory from python packages
* add template doc for each package. Add install operation in cmakelists.txt files
* update niryo_robot_tools_commander
* merging last 5 commits
* add learning mode -> button pressed jogTCP
* Merge branch 'refacto_tool_commander' into 'v3.2.0'
  Refacto tool commander
  See merge request `niryo/niryo-one-s/ned_ros_stack!22 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/22>`_
* Refacto tool commander
* Contributors: AdminIT, Clément Cocquempot, Corentin Ducatez, Ducatez Corentin, Etienne Rey-Coquais, Justin, Minh Thuc, Nicolas Guy, NicolasG_Niryo, Pauline Odet, Salomé Fournier, Thuc PHAM, Valentin Pitre, ValentinPitre, ccocquempot, f.dupuis, minhthuc

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
