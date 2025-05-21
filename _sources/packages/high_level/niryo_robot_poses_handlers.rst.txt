Niryo robot poses handlers
##########################

This package is in charge of dealing with transforms, workspace, grips and
trajectories.

It belongs to the ROS namespace: |namespace_emphasize|.

Workspaces
**********

A workspace is defined by 4 markers that form a rectangle. With the help of the robot's calibration
tip, the marker positions are learned. The camera returns poses (x, y, yaw) relative to the workspace.
We can then infer the absolute object pose in robot coordinates.

Grips
*****

When we know the object pose in robot coordinates, we can't directly send this pose to the robot because we specify the target pose of the 
tool_link and not of the actual TCP (tool center point).
Therefore we introduced the notion of grip. Each end effector has its own grip that specifies where to place the robot with respect to the object.

Currently, the notion of grip is not part of the python/tcp/blockly interface 
because it would add an extra layer of complexity that is not really necessary for the moment.

Therefore we have a default grip for all tools that is selected automatically based on the current tool id. However, 
everything is ready if you want to define custom grips, e.g. for custom tools or for custom grip positions.

The vision pick loop
********************

1. The camera detects objects relative to markers and sends   x\ :sub:`rel`\, y\ :sub:`rel`\, yaw\ :sub:`rel`\ .
2. The object is placed on the workspace, revealing the object pose in robot coordinates x, y, z, roll, pitch, yaw.
3. The grip is applied on the absolute object pose and gives the pose the robot should move to.

Dynamic frames
**************

.. note::
    This part is still in progress.


Poses & trajectories
********************

.. note::
    This part is still in progress.


Package Documentation
*********************

.. rosdoc:: /niryo_robot_poses_handlers
    :description_file: packages/descriptions.yaml
    :package_path_for_rosdoc_lite: ../niryo_robot_poses_handlers
    :can_be_simulated:
    :launchfile_path: ../niryo_robot_poses_handlers/launch/poses_handlers.launch

.. |namespace_emphasize| replace:: ``/niryo_robot_poses_handlers``

