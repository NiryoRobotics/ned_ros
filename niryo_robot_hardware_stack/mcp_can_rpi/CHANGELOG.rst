^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mcp_can_rpi
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
* adapt interfaces
* post merge conveyor improvement
* Merge branch 'clang_tidy' into conveyor_improvement
* Clang-only questionable constructs ok
* Merge remote-tracking branch 'origin/december_candidate' into sound_led_minor_improvements
* Merge branch 'december_candidate' into conveyor_improvement
* Merge branch 'ttl_service_improvment' into 'december_candidate'
  Ttl service improvment
  See merge request `niryo/niryo-one-s/ned_ros_stack!133 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/133>`_
* Ttl service improvment
* Merge branch 'clean_iot' into iot_ned2
* Merge branch 'v3.2.0' into clean_iot
* Merge branch 'v3.2.0' into system_software_api
* Merge remote-tracking branch 'origin/v3.2.0' into v3.2.0_niryo_one
* correction on wrong cmakelists for installing doc
* Merge branch 'v3.2.0_with_HW_stack_upgrade_cicd' into 'v3.2.0_with_HW_stack'
  Update CICD + various fixes related to CICD testing
  See merge request `niryo/niryo-one-s/ned_ros_stack!55 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/55>`_
* Update CICD + various fixes related to CICD testing
  Fix catkin_lint errors + missing controller for simulation launches
* Merge branch 'v3.2.0_with_HW_stack' into v3.2.0_with_HW_stack_dev_thuc
* Merge branch 'catkin_lint_check' into 'v3.2.0'
  Fix all catkin_lint erros/warns/notices
  See merge request `niryo/niryo-one-s/ned_ros_stack!51 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/51>`_
* Fix all catkin_lint erros/warns/notices
* Fix merge conflict
* Merge branch 'catkin_lint_clean' into 'v3.2.0_with_HW_stack'
  Catkin lint clean
  See merge request `niryo/niryo-one-s/ned_ros_stack!50 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/50>`_
* Catkin lint clean
* add empty roslint target in needed ros node
* Add empty roslint target in all packages so that catkin build --make-args roslint works
* Merge branch 'relative_namespaces_branch' into 'v3.2.0_with_HW_stack'
  merging namespace and tests improvement
  See merge request `niryo/niryo-one-s/ned_ros_stack!46 <https://gitlab.com/niryo/niryo-one-s/ned_ros_stack/-/merge_requests/46>`_
* retrieve architecture in CMakeLists
* restore docs changes (CMakeLists and dox)
* add documentation generation for python using epydoc. Clean CMakeLists.txt files
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
* switching to C++14
* adding xsd ref in package.xml files. Changing to setuptools instead of distutils.core, changing packages to format 3, set cmake min version to 3.0.2
* change niryo_robot_debug into dxl_debug_tools
* adding configurations for ned V1 and V2
* small corrections
* corrected crash of stepper joints
* settup of the documentation generation using rosdoc_lite
* adding logger configuration file in niryo_robot_bringup
* Contributors: AdminIT, Clément Cocquempot, Justin, Minh Thuc, Valentin Pitre, ValentinPitre, ccocquempot, minhthuc

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
