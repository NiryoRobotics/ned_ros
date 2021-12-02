^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package niryo_moveit_config_standalone
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'ned2_ralenti' into 'ned2_devel'
  Ned2 ralenti
  See merge request `niryo/niryo-one-s/ned_ros_stack!187 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/187>`_
* Ned2 ralenti
* fix link collisions + roslaunch
* Merge branch 'urdf_ned2' into 'ned2_devel'
  Urdf ned2
  See merge request `niryo/niryo-one-s/ned_ros_stack!183 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/183>`_
* Urdf ned2
* new profiles based on new calculus
* Merge branch 'clement_compat_ned_one' into ned2_devel
* better config for smooth movements
* add new moveit config
* pyniryo
* Merge remote-tracking branch 'origin/ned2_vaml' into pid_dev_frequencies
* motor velocity and acceleratio
* do not merge, test ameliorations
* ajustement asservissement vitesse acceleration moteurs
* Merge branch 'ned2_devel' into pid_dev_frequencies
* Merge remote-tracking branch 'origin/ned2_devel' into ned2_devel
* Merge branch 'optimize_delay_ttl_bus' into 'ned2_devel'
  update hot fix conveyor id + delay if read ttl failed + ticket no message if a motor disconnected + best config velocity now
  See merge request `niryo/niryo-one-s/ned_ros_stack!164 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/164>`_
* update hot fix conveyor id + delay if read ttl failed + ticket no message if a motor disconnected + best config velocity now
* Merge branch 'ned2_devel' into 'december_candidate'
  Ned2 devel
  See merge request `niryo/niryo-one-s/ned_ros_stack!163 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/163>`_
* Ned2 devel
* some changes for improve freq r/w position and velocity profile
* Merge branch 'ned2_devel' into 'december_candidate'
  stable version with last corrections
  See merge request `niryo/niryo-one-s/ned_ros_stack!159 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/159>`_
* stable version with last corrections
* Merge branch 'queue_optimization' into ned2_devel
* Merge branch 'queue_optimization' of https://gitlab.com/niryo/niryo-one-s/ned_ros_stack into queue_optimization
* optimize limit and pid
* Merge branch 'ned2_devel' into queue_optimization
* Merge branch 'improvement_movement' into 'ned2_devel'
  config for speed stepper
  See merge request `niryo/niryo-one-s/ned_ros_stack!155 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/155>`_
* Merge remote-tracking branch 'origin/improvement_movement' into tools_for_ned_2
* config for speed stepper
* add velocity in urdf
* add mutex to addsinglecmdtoqueue
* remove moveit limits for debug
* correction on simulation for ned2
* add sync read for N blockes of bytes
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
  # Conflicts:
  #	niryo_robot_hardware_stack/ttl_driver/src/ttl_interface_core.cpp
* Merge branch 'simu_ned_bug_fix' into 'december_candidate'
  Simu ned bug fix
  See merge request `niryo/niryo-one-s/ned_ros_stack!149 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/149>`_
* Simu ned bug fix
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
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
* Merge branch 'fake_ned_addition' into 'december_candidate'
  Fake ned addition
  See merge request `niryo/niryo-one-s/ned_ros_stack!98 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/98>`_
* Fake ned addition
* Merge branch 'release_septembre' into december_candidate
* Merge branch 'fake_drivers' into december_candidate
  Be carefull, lots of changes
* working !
* revert urdf names to niryo\_$(hardware_version)
* using simple controller for fake driver
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
* correction post merge
* correction post merge
* Merge branch 'v3.2.0_niryo_one' into december_candidate
* Contributors: Clément Cocquempot, Minh Thuc, Thuc PHAM, Valentin Pitre, ccocquempot, clement cocquempot, f.dupuis, minhthuc

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
