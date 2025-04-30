Examples: Basics
################

In this file, two short programs are implemented & commented in order to
help you understand the philosophy behind the Python ROS Wrapper.

.. danger::
    If you are using the real robot, make sure the environment around is clear.


Your first move joint
*********************

The following example shows a first use case.
It's a simple MoveJ. 

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:

Code details - First MoveJ
^^^^^^^^^^^^^^^^^^^^^^^^^^
First of all, we indicate to the shell that we are running a Python Script:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:
  :lines: 1

Then, we import the required class/functions to be able to access functions:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:
  :lines: 3

We start a :class:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper` and initialize a ROS Node in order to communicate with ROS master:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:
  :lines: 5-6

Once the connection is done, we calibrate the robot (Only for Ned2) using its
:meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.calibrate_auto` function:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:
  :lines: 8-9

As the robot is now calibrated, we can do a Move Joints by giving the 6 axis positions
in radians! To do so, we use :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move`:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/move_joints.py
  :language: python
  :linenos:
  :lines: 11-12


Your first pick and place
*************************
For our second example, we are going to develop script to make a pick and place.

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/pick_and_place.py
  :language: python
  :linenos:

Code details - first pick and place
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First of all, we import the ROS wrapper:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/pick_and_place.py
  :language: python
  :lines: 1
  :linenos:

Then, instantiate a :class:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper` instance
& calibrate the robot if needed:

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/pick_and_place.py
  :language: python
  :start-at: # Instantiate the ROS wrapper and initialize the ROS node
  :end-at: robot.calibrate_auto()
  :linenos:

Then, we setup the connected tool
with :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.update_tool`

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/pick_and_place.py
  :language: python
  :lines: 13-14
  :linenos:

Now that our initialization is done, we can open the Gripper (or push air from the Vacuum pump)
with :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.release_with_tool`,
move to the picking pose via :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.move`
& catch an object
with :meth:`~.niryo_robot_python_ros_wrapper.ros_wrapper.NiryoRosWrapper.grasp_with_tool`!

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/pick_and_place.py
  :language: python
  :start-at: # Open the tool
  :end-at: robot.grasp_with_tool()
  :linenos:

We can now move to the next pose, and place the object.

.. literalinclude:: /../niryo_robot_python_ros_wrapper/examples/pick_and_place.py
  :language: python
  :start-at: # Move to place pose
  :end-at: robot.release_with_tool()
  :linenos: