Niryo robot database
####################

This package is charge of managing the robot's database.
It is used to store and retrieve information about the robot's settings, versions, ...

It belongs to the ROS namespace: |namespace_emphasize|.

Database API functions
**********************

Database ROS wrapper
--------------------

In order to control the robot more easily than calling each topics & services one by one,
a Python ROS Wrapper has been built on top of ROS.

For instance, a script getting the robot name from the database via Python ROS Wrapper will looks like: 

.. code:: python

    from niryo_robot_database.api import DatabaseRosWrapper
    
    database = DatabaseRosWrapper()
    robot_version = database.get_setting('robot_name')

API list
--------

.. automodule:: niryo_robot_database.api.database_ros_wrapper
   :members:
   :undoc-members:
   :show-inheritance:
   :noindex:

Package Documentation
*********************

.. rosdoc:: /niryo_robot_database
    :description_file: packages/descriptions.yaml
    :patch_file: packages/rosdoc_patches/niryo_robot_database_patch.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_database

.. |namespace_emphasize| replace:: ``/niryo_robot_database``