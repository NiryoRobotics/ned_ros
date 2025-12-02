Quick start
###########

Welcome to the robot quick start.
Here you will learn the essential features of the robot to get you started.

Connect to your robot
*********************

There are 4 ways to connect your computer to your Ned robot:

Hotspot
-------

* **Type:** Wi-Fi
* **Difficulty:** :green:`Easy`
* **Description:** The robot provides its own Wi-Fi network. In this mode, you can connect to the robot like any other Wi-Fi network. The network name is in the format of **Niryo Hotspot <rasp_id>** and the default password is **niryorobot**.
* **More informations:** `Using Ned in Hotspot mode <https://docs.niryo.com/niryostudio/connecting-to-your-robot/connecting-to-your-robot/#section-header-two-2ojrn>`_ .
* **Advantage:** Easy, no cable required.
* **Disadvantage:** Ethernet connection needed on the computer to have access to the internet.
* **IP address:** 10.10.10.10

Connected mode
--------------

* **Type:** Wi-Fi
* **Difficulty:** :orange:`Medium`
* **Description:** The robot is connected to an existing Wi-Fi network.
* **More informations:** `Wi-Fi settings <https://docs.niryo.com/niryostudio/connecting-to-your-robot/connecting-to-your-robot/#section-header-two-9c4ls>`_.
* **Advantage:** Easy, no cable required.
* **Disadvantage:** Stable Wi-Fi connection required.
* **IP address:** Dependant of your network.

Direct ethernet
---------------

* **Type:** Ethernet
* **Difficulty:** :orange:`Medium`
* **Description:** The robot is connected directly to the computer via an ethernet cable.
* **More informations:** `Using Ned with an Ethernet cable <https://docs.niryo.com/niryostudio/connecting-to-your-robot/connecting-to-your-robot/#section-header-two-2bkg7>`_ .
* **Advantage:** The computer can have access to the internet through Wi-Fi. Safer and better communication with the robot.
* **Disadvantage:** Cable required. The robot has no access to the internet and cannot update itself.
* **IP address:** 169.254.200.200

Ethernet through network
------------------------

* **Type:** Ethernet
* **Difficulty:** :orange:`Medium`
* **Description:** The robot is connected to the network via an ethernet cable, and the computer is connected to the network via an ethernet cable or via Wi-Fi.
* **More informations:** `Ethernet settings <https://docs.niryo.com/niryostudio/connecting-to-your-robot/connecting-to-your-robot/#section-header-two-e1ilt>`_.
* **Advantage:** The robot and the computer can have access to the internet. Better communication with the robot.
* **Disadvantage:** Cable required.
* **IP address:** Dependant of your network.

Robot programming
*****************

There are 6 ways to program Niryo's robots:

.. list-table:: Ways to program the Niryo robots
   :header-rows: 1
   :widths: auto
   :stub-columns: 1
   :align: center

   *  - Name
      - Language
      - Difficulty
      - Documentation
      - Description

   *  - Niryo Studio
      - Blockly
      - :green:`Beginner`
      - :blockly:`Blockly <>`
      - | Simplified block programming.
        | Program your use cases as quickly as possible.

   *  - PyNiryo
      - Python
      - :orange:`Intermediate`
      - :pyniyro:`Pyniryo <>`
      - Program your robot remotely via a Python API 2.7 and 3.X .

   *  - Python ROS wrapper
      - Python
      - :orange:`Intermediate`
      - :doc:`Python ROS Wrapper <../packages/ros_wrapper>`
      - | Program and run your Python code directly in the robot.
        | No software or setup required except Niryo Studio or an ssh terminal.

   *  - ROS
      - Python, C++
      - :red:`Advanced`
      - :doc:`Niryo ROS <../packages/overview>`
      - | Program and run your ROS node directly on the robot,
        | or remotely through ROS Multimachine.

   *  - MODBUS
      - Any
      - :red:`Advanced`
      - :modbus:`MODBUS <>`
      - | Programs can communicate through network MODBUS
        | with the robots in any language available.

   *  - TCP Server
      - Any
      - :red:`Advanced`
      - :doc:`TCP server <../to_go_further/tcp_server>`
      - | Programs can communicate through network TCP
        | with the robots in any language available.

Tips
****

Program your first move in 30 seconds
-------------------------------------

The fastest way to program the robot is via :blockly:`Blockly <>`.
When you are on the Blockly page and logged into the robot, switch to learning mode via the toggle.
You can then press the button on top of the robot's base once to bring up a block with the robot's current position.
Thus, move your robot by hand, press the button and connect the blocks. Congratulations you have programmed a robot at lightning speed!

At the top right of the Blockly window, you can choose to save the positions in either **Joints** or **Pose** mode.

Joints & Poses, what's the difference?
--------------------------------------

The joints are the different joints of the robot. In joint mode, you give the robot a command on each of the robot motors.
The default unit used is the radian. 6.28318530718 radian is 2π and corresponds to 360°. On Niryo Studio you can switch to degrees for more simplicity.

The Pose corresponds to the x, y, z coordinates and the roll, pitch, yaw orientation (respecting the rotation around the x, y, z axes) of the extremity of the robot.
The z-axis is directed to the front of the robot, and the y-axis to the left of the robot. A positive z-coordinate will move the robot forward.
A positive y-coordinate will move the robot to the left, and negative y will move the robot to the right.

Sometimes there can be several axis configurations of the robot that correspond to the same coordinates.
This is why it is recommended to use the **Joints** commands instead.
The **Pose** is however easier and more intuitive to use to ask the robot to go for example 10cm higher, or 10 to the right.

Use a tool
----------

To use a tool, remember to use the :niryo_studio_scan_equipment:`scan <>` functionality to detect the connected tool.
You can then use the grippers, the Vacuum Pump or the Electromagnet as you wish.

Remember to add the scan function at the beginning of each of your programs to avoid any surprises.

Our different tools are intelligent, so the robot will be able to adapt its movements according to the selected tool for a pick and place with vision.
Also, you can program your movements with **Pose**.


Standard, linear, waypointed moves, what's the difference?
----------------------------------------------------------

There are many different types of movement possible for robot arms.
The 3 most used are the following:

* **Standard movements:** Also called PTP (Point To Point). This is the simplest movement.
  In this type of movement, the duration of the movement is minimized, each joint reaches the final position at the same time.
  The robot draws a kind of arc of a circle according to the initial and final positions.

* **Linear movements:** The robot draws a straight line between the start and end position
  However, a linear movement is not always possible between two points depending on the constraints of the robot.
  Make sure that the movement is feasible. If not, the robot will return an error.

* **Smoothed movements by waypoints:** This is where we ask the robot to make a movement to an end point by passing through intermediate points.
  The robot draws linear paths, or PTP if linear motion is not possible, between each waypoint without stopping.
  It is also possible to record blend radius to smooth the movement and to draw curves between the points.
  This path is ideal for dodging obstacles.


.. figure:: /.static/images/waypointed_trajectory.png
   :alt: Waypointed trajectory with blend radius
   :width: 400px
   :align: center

   `Waypointed trajectory with blend radius <https://ros-planning.github.io/moveit_tutorials/doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html#user-interface-sequence-capability>`_


Start, Pause, Cancel a program execution
----------------------------------------

You may not know it, but the button on the top of the base of the robot also allows you to start, pause and stop a program.

When a program is running:
    * 1 press pauses the program
    * 2 presses will pause the programme and activate the learning mode

When a program is paused:
    * 1 press resumes the program
    * 2 presses stop the program
    * If there is no intervention for 30 seconds, the programme stops automatically

When the program is paused, the LED at the back flashes white.

When no program is running you can also start a program by pressing the same button once.