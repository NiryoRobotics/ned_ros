Niryo robot description
#######################

This package contains URDF files and meshes (collada + stl) for Ned.

To display Ned on Rviz:

.. code:: bash

 roslaunch niryo_robot_description display.launch

To display other Niryo robots on Rviz:

.. code:: bash

 roslaunch niryo_robot_description display.launch hardware_version:=ned2  # ned2, ned3pro


**Note** : 3D visualization is not available on Ned Raspberry Pi4 image.
To use the following commands, you must have setup Ned ROS Stack on your computer.