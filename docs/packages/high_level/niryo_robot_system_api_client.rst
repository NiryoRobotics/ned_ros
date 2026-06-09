Niryo robot System API Client
#############################

This package is wraps a python client to the System API. It also expose some ROS services for the CPP packages to use the API.

It belongs to the ROS namespace: |namespace_emphasize|.

System API Client API functions
**********************

System API Client
-----------------

The library has been designed in order to be ROS agnostic, so it can be used in any python code.
For instance, a script getting the robot name via Python ROS Wrapper will looks like:

.. code:: python

    from niryo_robot_system_api_client import system_api_client

    response = system_api_client.get_setting('robot_name')
    robot_name = response.data['robot_name']

System API Client ROS wrapper
--------------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script getting the robot name via Python ROS Wrapper will looks like:

.. code:: python

    from niryo_robot_system_api_client.api import SystemAPIRosWrapper
    
    client = SystemAPIRosWrapper()
    robot_name = client.get_setting('robot_name')

API list
--------

.. automodule:: niryo_robot_system_api_client.api.system_api_client_ros_wrapper
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Package Documentation
*********************

.. rosdoc:: /niryo_robot_system_api_client
    :description_file: packages/descriptions.yaml
    :patch_file: packages/rosdoc_patches/niryo_robot_system_api_client_patch.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_system_api_client

.. |namespace_emphasize| replace:: ``/niryo_robot_system_api_client``