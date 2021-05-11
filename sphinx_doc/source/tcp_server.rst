Use Ned TCP Server
========================================

Ned is permanently running a TCP Server to acquire requests.
This server is built on top of the :doc:`Ned Python ROS Wrapper <ros_wrapper>`

It offers a simple way for developers to create programs for robot
to control them via remote communication on a computer, on a mobile
or any device with network facilities.

Programs can communicate through network TCP with the robots
in any language available.


Connection
-------------
To access the server, you will have to use to Robot's IP Adress and communicate 
via the **port 40001**

Communication
-----------------
Only one client can communicate with the server (reconnection effective but no multi clients).

The server answer only after the command is done, so it cannot deal with multiple commands at the same time.

Packet Convention
-----------------------

General format
^^^^^^^^^^^^^^^^^^

For easier usage and easier debugging, the communication is based on JSON format.

Every package have this following shape : ``<json_packet_size>{<json_content>}<payload>``

The JSON packet size is an unsigned short coded on 2 bytes

The JSON contains command's name & params

Payload contains *heavy* data like an image

Request
^^^^^^^^^^^^^^^^^^

Format - Request
""""""""""""""""""""""""""""

As no function request a payload in input, requests have the following

format : ``<json_packet_size>{'param_list': [<param_1>, <param_2>, ....], 'command': <command_str>}``

Examples - Request
""""""""""""""""""""""""""""

Calibrate auto : ``{'param_list': ['AUTO'], 'command': 'CALIBRATE'}``

Move joints : ``{'param_list': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 'command': 'MOVE_JOINTS'}``

Answer
^^^^^^^^^^^^^^^^^^

Format - Answer
""""""""""""""""""""""""""""

Firstly, answers indicate to the user if its command has been well executed.
This is indicated in the JSON by the parameter "status"

A successful answer will have the format :

``{'status': 'OK', 'list_ret_param': [<param_1>, <param_2>, ....], 'payload_size': <payload_size_int>, 'command': <command_str>}<payload_str>``

An unsuccessful answer will have the format :
``{'status': 'KO', 'message': <message_str>}``

Examples - Answer
""""""""""""""""""""""
Calibrate Auto :
``{'status': 'OK', 'list_ret_param': [], 'payload_size': 0, 'command': 'CALIBRATE'}``

Get Pose:
``{'status': 'OK', 'list_ret_param': [0.2, 0.15, 0.35, 0.5, -0.6, 0.1], 'payload_size': 0, 'command': 'GET_POSE'}``



Commands enum for TCP Server
-------------------------------
.. automodule:: niryo_robot_python_ros_wrapper.ros_wrapper_enums
    :members: CommandEnum
    :undoc-members:
    :exclude-members:
    :member-order: bysource
