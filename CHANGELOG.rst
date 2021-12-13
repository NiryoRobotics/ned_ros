^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for ned_ros_stack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
----------------
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
----------------
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
----------------
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
-----------------
**Bug Corrections**

* correction in joints_limit.yaml
* remove end effector read status when robot moving (check if collision is still read...)
* smoother movement
* read firmware version only at init

ned2_v0.2 (2021-11-04)
------------------

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
------------------
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
