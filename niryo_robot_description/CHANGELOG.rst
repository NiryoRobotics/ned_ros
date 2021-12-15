^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package niryo_robot_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix urdf + end effector panel python
* fix without mesh link collisions
* fix link collisions + roslaunch
* Merge branch 'urdf_ned2' into 'ned2_devel'
  Urdf ned2
  See merge request `niryo/niryo-one-s/ned_ros_stack!183 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/183>`_
* Urdf ned2
* Merge branch 'standardize_drivers' into 'ned2_devel'
  standardize drivers, compile, launch and calib on ned, ned2 and one and simu
  See merge request `niryo/niryo-one-s/ned_ros_stack!182 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/182>`_
* standardize drivers, compile, launch and calib on ned, ned2 and one and simu
* fix gazebo simu
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* Fix vision
* Merge branch 'ned2_devel' into pid_dev_frequencies
* enhance joints limits on j5
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* Merge branch 'queue_optimization' into ned2_devel
* Merge branch 'queue_optimization' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into queue_optimization
* add velocity in urdf
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* Merge branch 'master' into december_candidate
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
* Upgrade versions to v3.2.0
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* Merge branch 'release_septembre' into december_candidate
* working !
* Merge branch 'fix_release_septembre' into 'release_septembre'
  Fix release septembre
  See merge request `niryo/niryo-one-s/ned_ros_stack!80 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/80>`_
* Fix release septembre
* revert urdf names to niryo\_$(hardware_version)
* correction in progress for joints controller not loaded correctly
* Remove Fake_interface
* change use_fake_driver to hardware_version:=fake
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
* Merge branch 'release_septembre' into v3.2.0_with_HW_stack
* Merge branch develop
* correction post merge
* correction post merge
* Merge branch 'v3.2.0_niryo_one' into december_candidate
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* Merge remote-tracking branch 'origin/v3.2.0' into v3.2.0_niryo_one
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
* Increasing the limits of the joint5 for the vision kit
* Merge branch 'catkin_lint_check' into 'v3.2.0'
  Fix all catkin_lint erros/warns/notices
  See merge request `niryo/niryo-one-s/ned_ros_stack!51 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/51>`_
* Fix all catkin_lint erros/warns/notices
* Merge branch 'v3.2.0' into system_software_api
* fix xacro collision for kinetic
* fix xacro imports
* Niryo One config
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* add empty roslint target in needed ros node
* Add empty roslint target in all packages so that catkin build --make-args roslint works
* retrieve more elements
* retrieve changes from all packages except hw stack
* small corrections
* merged v3.2.0 into v3.2.0_with_HW_stack
* add ned2 hardware for all impacted packages
* all nodes can launch separately on dev machine.
* add logging system in all py nodes
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
* Merge branch 'tcp_manager' into 'v3.2.0'
  Tcp manager
  See merge request `niryo/niryo-one-s/ned_ros_stack!18 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/18>`_
* Tcp manager
* Merge branch 'cmakelist_additions_branch' into 'v3.2.0_with_HW_stack'
  merge into v3.2.0 with hw stack
  See merge request `niryo/niryo-one-s/ned_ros_stack!29 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/29>`_
* Merge branch 'apply_roslint_branch' into 'cmakelist_additions_branch'
  merge rolint correction in cmake addition branch
  See merge request `niryo/niryo-one-s/ned_ros_stack!28 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/28>`_
* roslint done for cpp
* correction on doc install
* add documentation installation
* remove doc directory from python packages
* add template doc for each package. Add install operation in cmakelists.txt files
* merging last 5 commits
* merge HW stack into v3.2.0. A new branch has been defined for this purpose
* Merge branch 'refacto_tool_commander' into 'v3.2.0'
  Refacto tool commander
  See merge request `niryo/niryo-one-s/ned_ros_stack!22 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/22>`_
* Refacto tool commander
* merge v3.2.0 in moveit_add_collision
* adding xsd link into launch files. Correcting tests for launch on dev machine
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* hide tf frame of collision motor at launch in rviz
* update box size to be bigger than motor 5
* clean urdf
* modify the urdf without mesh for the real robot
* add collision for joint 5 motor
* Contributors: AdminIT, Clément Cocquempot, Corentin Ducatez, Ducatez Corentin, Justin, Minh Thuc, Pauline Odet, Salomé Fournier, Valentin Pitre, ValentinPitre, ccocquempot, clement cocquempot, f.dupuis

3.2.0 (2021-09-23)
------------------
* Merge branch 'develop' into 'master'
  v3.2.0
  See merge request `niryo/niryo-one-s/ned_ros_stack!113 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/113>`_
* Release September: v3.2.0
* Merge branch 'release_language_versions' into 'develop'
  Release language versions
  See merge request `niryo/niryo-one-s/ned_ros_stack!62 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/62>`_
* Release language versions
* Merge branch 'Bug_fix_gripper3' into 'develop'
  Bug fix gripper3
  See merge request `niryo/niryo-one-s/ned_ros_stack!26 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/26>`_
* v3.1.1: Bug fix gripper3
* Change versions in package.xml to 3.1.0
* Contributors: Ducatez Corentin, corentin ducatez

3.1.2 (2021-08-13)
------------------
* Merge develop branch, see MR \`!63 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/63>`_: languages + versions for documentation + gitlab cicd
* Contributors: Corentin Ducatez

3.1.1 (2021-06-21)
------------------
* v3.1.1: Fix grip offset for gripper3 (vision pick)
* Merge branch 'develop' into 'master'
  Release v3.1.0
  See merge request `niryo/niryo-one-s/ned_ros_stack!9 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/9>`_
* Release v3.1.0
* Contributors: Corentin Ducatez, Ducatez Corentin

3.1.0 (2021-05-06)
------------------
* Change versions in package.xml to 3.1.0
* Contributors: corentin ducatez

3.0.0 (2021-01-25)
------------------
