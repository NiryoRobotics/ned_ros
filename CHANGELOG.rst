^^^^^^^^^^^^^^^^^^^^^^^
Ned ROS stack Changelog
^^^^^^^^^^^^^^^^^^^^^^^

forthcoming
-----------

* fix(doc): use the link to the public github page in the README
* fix(niryo_robot_vision): stream handling improvements
* fix(niryo_robot_vision): fix concurrency issues between the post-processing pipeline and the external image accessors
* fix(niryo_robot_programs_manager_v2): ensure the output is fully consumed before quitting
* fix(niryo_robot_vision): use correct image to find the object's main color when looking for any color
* fix(niryo_robot_vision): apply the blur before converting to HSV
* fix(niryo_robot_vision): blue thresholds fine-tuning to reduce noise
* fix(niryo_robot_vision): check if the found shape matches the requirements before returning
* fix(niryo_robot_vision): increase nb_contours_max from 3 to 10 to avoid missing circles when there is more than 3 squares on the workspace

v5.6.1
-----------

* fix(vision_node): (SN-900) add back camera_intrinsics topic

v5.6.0
-----------

* feat: add new hosted documentation
* feat: Display program names with the program player
* feat: Add program player lock/unlock buttons mechanism by pressing up + custom/down + custom during 3 seconds
* feat: Implement program player custom button
* feat(vision): add debug topics which can be activated by calling the service /niryo_robot_vision/debug/activate:

  * /niryo_robot_vision/debug/active_topics: list of active debug topics
  * /niryo_robot_vision/debug/red_channel: red channel image
  * /niryo_robot_vision/debug/green_channel: green channel image
  * /niryo_robot_vision/debug/blue_channel: blue channel image
  * /niryo_robot_vision/debug/any_channel: RGB channels image
  * /niryo_robot_vision/debug/markers: workspaces markers image

* feat: Async ros wrapper move functions
* feat(vision): increased video stream frame rate (from 5 to 15 fps)
* feat(vision): the vision node can now get the video stream from any external client. Use the service /niryo_robot_vision/set_source to set the source to `client` then publish the stream on `/niryo_robot_vision/client_stream`
* refactor(vision): improved image processing pipeline speed
* refactor: remove TCPv2 from the software as it brings more problems than it solves
* fix: Wait for joint_states and rpi node before building modbus registers
* fix: sendCustomCommand instant return success if motor is fake
* fix(conveyor_interface): instanciate a fake stepper if in simulation mode
* fix(robot state publisher): skip the publish if robot state is None
* fix(ros_wrapper): fix move_relative method
* fix(programs_manager): don't wait for database node in programs manager anymore
* chore(vision): cleanup of dead code
* chore(gripper): increase position range (open/close) for gripper1
* fix: gripper and conveyor are not simulated by default anymore

v5.5.5
------

* fix: Fixed the conveyors modbus registers crashing
* fix: publish on program_list topic when service update_program is called

v5.5.4
------

**Features**

* feat: add ned3pro simulation launchfile
* feat: add updated ROS wrapper examples

**Improvements**

* Check saved trajectory validity before executing it and output an error message if a waypoint exceeds the soft limits
* No wait_for in nodes constructors. Use callbacks / lazy loading instead
* Expose /niryo_robot_led_ring/led_ring_status topic though the led ring ros wrapper with 3 new properties:

  * is_autonomous
  * animation_mode
  * color

* Expose the service /niryo_robot_led_ring/set_user_animation though the led ring ros wrapper with the function set_user_animation. All the higher animation functions use this one under the hood
* New end of production test script

  * Seperated from the demo program
  * Do the same tests than the previous one but with a clearer and easily maintenable code
  * Is unique for all the robots
  * Delete the test script once it is successful and replace it with the demo script

* New demo program, which do the same actions than the previous one but in a more readable fashion

  * the demo program is also unique for all the robots

* Add missing functions to JointsPosition object to make it setattr compatible
* Add the module end_of_production_test to the niryo_robot_utils package

  * LedRingManager handle the led ring during the EOP test
  * TestReport run a tests playbook, generate the report and send it to RFM
  * utils contains some utility functions and the BaseTest class which is the base class for all the tests run during the EOP test

* refacto: use temp files to handle tts files
* refacto: make of sentry a soft dependency
* style: cleanup CHANGELOG
* refactor: remove old documentation
* refactor: remove empty config file
* refactor: remove several unused parameters
* docs: correct get_joints() docstring
* ci: delete all doc related jobs

**Bugfixes**

* Fixed a typo in arm commander stop command service clients which prevented the service from being called during robot shutdown and when using the ROS wrapper
* Fixed program player unit tests so that they reflect the entire finite state machine
* Fixed Ned3pro calibration tip transform
* Fixed a bug in the robot_state publisher which issued a warning when trying to divide by 0
* Resolved multiple bugs that were preventing the simulated robot stack from functioning properly

  * Sentry is not initialized if the API keys are not set (typical simulation scenario)
  * Added missing tool parameters in simulation
  * Modbus ROS package is not started anymore in simulation mode
  * Fixed the control loop which is not releasing its lock if the loop frequency is too fast
  * Fake stepper motors does not require to be calibrated anymore in simulation

* fix: check if a sound file is empty before loading it
* (reports) use set_serial_number and set_rasp_id instead of set_identifier
* (reports): return True instead of ping returns (None)
* (reports): set correct endpoint for reports ping
* fix: wrong os.getenv call in sentry_init
* The end effector panel still publish the buttons state while calibration is in progress
* Fixed a bug which caused the cloud api to not update it's S/N and rasp_id values when they are updated
* Fixed a bug causing the check_connection service to ping the wrong endpoints for the reports micro services
* fix: add missing tool_id comment for vacuum pump v2
* fix: use the correct field to qualify a Pin in the GetDigitalIORequest
* fix: add missing led.yaml file for ned3pro simulation
* fix: remove install instruction for non-existing config directory in niryo_robot_programs_manager_v2


v5.5.3
------

**Features**

**Improvements**

* Improve /niryo_robot/conveyor/feedback reactivity when a conveyor is added or removed
* Added convenience function NiryoRosWrapper.init_with_node which init a ros node and returns a NiryoRosWrapper instance
* Deprecate NiryoRosWrapper.move_relative
* Added a param normalize=True to NiryoRosWrapper.msg_from_pose
* Add security checks to NiryoRosWrapper.unset_conveyor and NiryoRosWrapper.control_conveyor
* NiryoRosWrapper.get_conveyors_feedback now returns a list of dict instead of a list of ConveyorFeedback messages
* Added the following functions to the TCP server:

  * GET_POSE_V2
  * FORWARD_KINEMATICS_V2
  * INVERSE_KINEMATICS_V2
  * GET_TCP
  * GET_CONVEYORS_FEEDBACK

**Bugfixes**

* Fixed wrong camera and gripper transforms in Ned3pro and Ned2's URDF for simulation
* Fixed a bug which prevented the jog feature to work
* Use lazy loading from database when getting the joints home position settings to prevent a bug where the robot was unable to access the database at boot time
* Ned3pro digital inputs are now not inverted
* Fixed the jog features
* Fixed NiryoRosWrapper.__get_obj_from_args which didn't work with JointsPosition
* Fixed NiryoRosWrapper.forward_kinematics which returned v2 metadata despite being a v1 pose
* Fixed NiryoRosWrapper.inverse_kinematics which returned was converting poses to v2 before doing the inverse kinematics
* Fixed a bug in Pose which was allocating the same instance of PoseMetadata when using the constructor's default metadata value
* Fixed a bug where memory was allocated and not freed each time the program player's node tried to connect to an inexistant device
* Fixed a bug in Tool ROS wrapper which was not using the correct service type when calling tool reboot service
* use correct message type for ros subscriber in arm_state

v5.5.2
------

**Features**

**Improvements**

* perf: add send_buffer_limit rosarg in foxglove spawner

**Bugfixes**

* fix: use forward_kinematics_v2 in __ros_poses_from_robot_positions
* fix: DXL' PIDs adjustments

v5.5.1
------

**Features**

**Improvements**

* Increase error threshold for joints out of bounds detection in order to match hardware limits
* Added topic '/niryo_robot/max_acceleration_scaling_factor' to check current acceleration percentage
* Refacto of the wifi button handling: the names reflects its real job, which is managing solely the hotspot
* Disabled torque of steppers not being calibrated during factory calibration

**Bugfixes**

* Fixed a bug while setting acceleration from the '/niryo_robot_arm_commander/set_acceleration_factor' service which always set it to 40%
* Fixed the conveyors direction which was inverted in the modbus server
* Fixed PoseMetada.from_dict which wasn't passing the 'frame' argument when using a PoseMetadata.v1
* Fixed collision thresholds settings which was not correctly calling the setting of the 2nd threshold and with the wrong datatype
* Fixed a bug in __hotspot_state_callback which still processed the API response even if the request failed
* Fixed tools transforms for Ned3 pro
* Fix a missing offset in the URDF for Ned3 pro
* Fixed a goal tolerance too low for the joint 1


v5.5.0
------

**Features**

* Add sentry for python nodes
* Add Ned3 demo/production program

**Improvements**

* Add program player single point of control safety,

  * Program player is running a program and it's disconnected program stops
  * Program running without program player and one is connected program stops

* Add PID and vel/acc profiles configurations for the grippers and configure them on initialization
* Display stop message for player one when a program is stopped
* Rename ned3 to ned3pro

**Bugfixes**

* Add retrocompability get pose
* Add retrocompability get list of trajectories
* Add missing inverse_kinematics_v2() function in ROS wrapper and tag forward_kinematics() and inverse_kinematics() as deprecated
* Velocity and acceleration motor profiles are scaled when scaling the robot velocity and synced with moveit scaling
* Add save pose compatibility with legacy poses
* Ill-formatted trajectories, poses, workspaces and dynamic frames won't make the stack crash
* Add TCP server support for vacuum pump v2
* Remove a collision checking in niryo_robot_arm_commander which was a duplicate of ROS control's joint trajectory controller
* Created get_target_pose_v2 service in order to keep get_target_pose original behaviors
* Created a foxglove spawner to start the bridge once the robot has finished booting. This prevents some errors that occurs with the clients
* Remove move spiral demo Ned3
* Reset home position service called uninitialized default_home_position vector, now it's well initialized


v5.4.0
------

**Features**

* Implemented NED3 hardware changes into niryo_robot_rpi package
* Add Ned3 Calibration support
* Add support for Conveyor with ned3 stepper
* Add support for vacuum pump v2
* Add velocity and acceleration profile for vacuum pump v2
* Change the robot's URDF in order to follow the Denavit-Hartenberg convention.
* New Pose and JointsPosition
* The `/niryo_robot_arm_commander/robot_action` action server can handle old and new TCP versions using ``tcp_version`` in ``ArmMoveCommand.msg``
* Edit niryo_robot_poses_handlers' grip files according to the new TCP orientation
* Added ``pose_version`` and ``tcp_version`` to ``NiryoPose.msg``
* New generic classes designed to be the universal classes to represent the data:

  * JointsPosition
  * JointsPositionMetadata
  * Pose
  * PoseMetadata

* New niryo_robot_poses_handlers/transform_functions functions: ``convert_legacy_rpy_to_dh_convention()`` and ``convert_dh_convention_to_legacy_rpy()``
* New ros_wrapper.NiryoRosWrapper functions which can't take either Pose or JointsPosition objects:

  * move replace move_joints, move_pose and move_linear_pose.
  * jog_shift, replace jog_joints_shift and jog_pose_shift.
  * pick, replace pick_from_pose.
  * place, replace place_from_pose.
  * execute_trajectory, replace execute_trajectory_from_poses and execute_trajectory_from_poses_and_joints.
  * compute_trajectory, replace compute_trajectory_from_poses and compute_trajectory_from_poses_and_joints.

* New TCP server commands: GET_COLLISION_DETECTED, CLEAR_COLLISION_DETECTED, HANDSHAKE, MOVE, JOG, PICK, PLACE, EXECUTE_TRAJECTORY
* New tools translation transforms according to the new TCP orientation
* Add Hardware ID to the conveyors, only for v2 and v3
* The new robot pose is published on /niryo_robot/robot_state_v2 in order to keep compatibility with older NiryoStudios
* The new robot relative pose is published on /niryo_robot_poses_handlers/relative_pose_v2 in order to keep compatibility with older NiryoStudios
* Add a service to get conveyor hardware ID to be able to differentiate them (conveyor v2 and v3), service name: `/niryo_robot/conveyor/get_hardware_id`
* Add services to get forward and inverse kinematics using the new TCP convention (Z axis pointing forward)
* Add a ROS wrapper function to get the forward kinematics using the new TCP convention (Z axis pointing forward)
* Add a ROS topic `/niryo_robot_tools_commander/tcp_v2` which publishes the transform from hand_link to the TCP following the new TCP convention
* Change robot's pose limits for Ned3 to match its reachability

**Improvements**

* Use the I/O panel version instead of the robot hardware version to differentiate the implementations
* Created a set of classes gpio_expander_adapters. Theses classes are adapters in order to handle GPIOs the same way MCP IOs are handled
* Update of the DACx0501 driver
* NiryoRosWrapper.vision_pick now can take an optional observation pose ``obs_pose``
* Add speed limit pourcentage for the conveyor
* When a tool stops moving, a position-holding command is sent instead of resending the command with the max position, drastically reducing temperature rise
* Update dynamixels PID (Axis 4 to 6)

**Bugfixes**

* Stopping a program now send SIGTERM and then SIGKILL after 3 seconds if the program didn't exit gracefully
* Grasp and release actions now use a feedback to check if they finished their motion instead of stopping after a fixed time
* Debounce emergency stop resume to avoid to resume on small 12v spikes
* Renaming a dynamic frame also rename its name in the transform
* NiryoRosWrapper.get_workspace_list no longer return an error
* TCP server ``__send_answer_with_payload`` encode the payload only if it's not already encoded
* Fix intermitent delay in the control loop caused by a ROS spinOnce call in the control loop


v5.3.3
------

**Features**

**Improvements**

* Increase threshold for end effector collision detection

**Bugfixes**


v5.3.2
------

**Features**

**Improvements**

**Bugfixes**

* Stopping a program now send SIGTERM instead of SIGKILL in order to let the program handle its exit

v5.3.1
------

**Features**

**Improvements**

**Bugfixes**

* Fixed a bug which didn't handled when a goal was timed out in NiryoActionClient
* Fixed a bug which could lead to the tool commander's action server to always be locked in active mode
* Fixed a bug which prevented the tool commander action server to accept new goals once a goal with a future date was published

v5.3.0
------

**Features**

**Improvements**

* Brand new modbus server
* New ros_wrapper functions: get_current_tool_state, get_tcp, get_digital_io_mode, get_available_disk_size, get_ros_logs_size, control_video_stream
* New system_api_client endpoint: get_system_version_current

**Bugfixes**

* in ros_wrapper, __conveyor_id_to_conveyor_number no longer rely on the currently attached conveyors
* fixed some incorrect ros_wrapper's docstring
* fixed a bug which prevented to do vision picks with a TCP transformation enabled
* fixed a bug which caused an executed program's process to not totally stop

v5.2.2
------

**Features**

**Improvements**

* The service /niryo_robot/kinematics/forward now returns status and message in its response
* The service /niryo_robot/kinematics/inverse now returns status and message in its response

**Bugfixes**

* Fixed the service /niryo_robot/kinematics/forward which sometimes crashed because of transform extrapolation

v5.2.1
------

**Features**

**Improvements**

**Bugfixes**

* the service set_program_autorun wasn't taking the mode into account when passing "DISABLE"

v5.2.0
------

**Features**

* Added a foxglove bridge server
* new messages: BasicObject and BasicObjectList
* New topics:

  * /niryo_robot_arm_commander/trajectory_list (BasicObjectArray)
  * /niryo_robot_poses_handler/pose_list (BasicObjectArray)
  * /niryo_robot_poses_handler/dynamic_frame_list (BasicObjectArray)

* New topic: /niryo_robot_poses_handlers/relative_pose

  * This topic publish the TCP pose relative to a dynamic frame

* New service: /niryo_robot_poses_handlers/set_relative_transform_frame

  * Use this service to set the dynamic frame which should be used for the relative pose

* New service: /niryo_robot_database/get_db_file_path

  * Use this service to retrieve the database file path

**Improvements**

* Refacto of the programs manager

  * It now uses programs ids to handle the programs
  * A program is now defined as a python program which can have a blockly program attached
  * An action server is used to execute a program rather than a service
  * The autorun and the programs properties are stored in the database
  * Named programs_manager_v2 in order to keep the old programs manager for NS1

* The service GetNameDescription takes an array of BasicObject (for future compatibility)
* remove ros_wrapper_2

* modified service type:

  * /niryo_robot/tools/reboot: std_srvs/Trigger -> niryo_robot_msgs/Trigger
  * /niryo_robot_vision/debug_markers: Added "status" and "message" to service response
  * /niryo_robot_vision/debug_colors: Added "status" and "message" to service response
  * /niryo_robot_vision/visualization: Added "message" to service response

**Bugfixes**

* Removed double assignment of the const REBOOT_MOTOR in RobotStatus.msg

v5.1.3
------

**Features**

**Improvements**

* Added a field "saved_at" in the service /niryo_robot_programs_manager/get_program response

**Bugfixes**


v5.1.2
------

**Features**

**Improvements**

* The daily reports can now send metrics about the robot. Currently, the total lifetime of the robot is sent

**Bugfixes**


v5.1.1
------

**Features**

**Improvements**

* Revamped WiFi button functionalities: Brief press (< 2s) toggles hotspot; Extended press (> 2s) enables/disables WiFi client; Long press (> 7s) restores network settings. To abort, press for over 10 seconds.
* WiFi button press triggers LED ring indication for forthcoming action.
* The database node use the system software HTTP API to get and set the settings in the database. This is to ensure there is only one access point to the database.
* Reduced the grippers open / close torque to slow down the motor's heating

**Bugfixes**

* Resolved an issue where the hotspot failed to initiate concurrently with the WiFi client.
* Fixed a bug that consistently disregarded the "purge ros logs on startup" command.
* Fixed a problem that sometimes stopped the camera from recognizing colored shapes.
* Fixed an issue that permitted editing of dynamic frames belonging to workspaces.
* Rectified dynamic frame editing by implementing quaternion normalization.
* Fixed a bug that caused the vision picks to catch objects by their corners.


v5.1.0
------

**Features**

* The topic ``/niryo_robot_tools_commander/tcp`` now return the tcp position either if it is enabled or not
* Every part of a dynamic frame can now be edited using the service ``/niryo_robot_poses_handlers/manage_dynamic_frame``
* Manage a file in ``~/.ros/logs`` which store the date corresponding to the ros run ids

**Improvements**

**Bugfixes**

v5.0.1
------

**Features**

**Improvements**

**Bugfixes**
* Fixed a bug preventing the downloaded update to be applied successfully

v4.0.0 (2021-12-16)
-------------------

**Requirements**

sudo apt-get install sqlite3
End effector driver fw 1.0.7

**Bug corrections**


**Features**

* add VERSION file at root 
* add CHANGELOG.rst in every package (using catkin_generate_changelog tool)
* add this changelog
* fix calibration for Ned and One
* add documentation strucutre (sphinx doc)
* update PID values for DXL (ned2)
* update joints_interface and ttl_driver read and write frequencies

**Limitations**


* Contributors: AdminIT, Clément Cocquempot, Corentin Ducatez, Minh Thuc, Mottier Justin, Thuc PHAM, Valentin Pitre, f.dupuis

ned2_v0.6 (2021-12-02)
----------------------
**Requirements**

* steppers driver with fw 1.0.16
* gtts (pip install gTTS)

**Bug corrections**

* slower movements
* wrong conveyor feedback
* drivers optimised (using more precise velocity for steppers profiles)
* error connection better handled (resolve pb of read blocked for end effector mainly)
* new urdf for ned2
* missing firmware version sometimes corrected
* wrong conversion for axis 6 for Ned
* end effector input
* warn end effector at startup
* rework of bringup to simplify it

**Features**

* Compatibility Ned and One
* Calibration ok
* Retrieve firmware
* last PID in place
* last velocity profiles for steppers
* voltage and temperature for all hw
* documentation structure set up for the stack
* option simu_gripper and simu_conveyor added for simulation
* remove service GetFrequencies and SetFrequencies
* script prod
* script videoshoot
* urdf + collada + stl ned2
* circular trajectories
* spirals trajectories
* Save and Replay waypointed trajectories
* Google Text to speech

**Limitations**

* You need to update your steppers drivers to version 1.16
* control mode bug if the stack is launch with a tool connected
* no blinking of the led ring when there is a motor error or an error message in topic hardware_status (to avoid blinking for nothing)
* manual calibration not working

Tests done (will be updated)
Non tested (will be updated)

ned2_v0.5 (2021-11-25)
----------------------
**Bug corrections**

* less jerky mouvments
* stabilised reboot
* security on moveit
* pb out of bond corrected

**Features**

* Compatibility Ned and One
* last PID in place
* last velocity and acceleration profiles

**Limitations**

* connection loss from time to time, linked to bus pb (corrected in new cards, stand by for now)

**Tests done**

* compilation
* calibration Ned2

**Non tested (will be updated)**

*  all accessories, grippers and conveyors
*  blockly
*  Ned, one simu

ned2_v0.4 (2021-11-17)
----------------------
**Features**

* Calibration reworked, more stable, slower
* Initializations rework
* Queues protection
* Sync read fail corrected on End Effector
* Error messages enhanced
* Version, Temperature and voltage addition on Tools and Conveyors
* Calibration status kept if motors are not shutdown

**Limitations**

* Not tested with conveyor
* Control mode fail on gripper
* Reboot fail (unexpected movements during reboot)
* Joints limits to update
* Broken simulation
* Ned and one incompatible

ned2_v0.3 (2021-11-08)
----------------------
**Bug Corrections**

* correction in joints_limit.yaml
* remove end effector read status when robot moving (check if collision is still read...)
* smoother movement
* read firmware version only at init

ned2_v0.2 (2021-11-04)
----------------------

**Bug corrections**

* smoother movements
* conveyor fixes (to be tested)
* last additions from valentin
* cross compilation rpi4 pk

**Limitations**

* Carefull : ned and one compatibility broken


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

ned2_v0.1 (2021-09-21)
----------------------
* Stable version usable for Ned 2 testing
* Be carefull, usable only with niryo studio tag ned2_v0.1

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
