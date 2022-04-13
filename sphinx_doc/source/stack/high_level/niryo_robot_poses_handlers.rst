Niryo_robot_poses_handlers
======================================================

This package is in charge of dealing with transforms, workspace, grips and
trajectories.


Poses handlers node
--------------------------

Description - Poses handlers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ROS Node is made of several services to deal with transforms, workspace, grips and
trajectories.

It belongs to the ROS namespace: |namespace_emphasize|.

Workspaces
"""""""""""""

A workspace is defined by 4 markers that form a rectangle. With the help of the robot's calibration
tip, the marker positions are learned. The camera returns poses (x, y, yaw) relative to the workspace.
We can then infer the absolute object pose in robot coordinates.

Grips
""""""""""""""""""""""""""""""

When we know the object pose in robot coordinates, we can't directly send this pose to the robot because we specify the target pose of the 
tool_link and not of the actual TCP (tool center point).
Therefore we introduced the notion of grip. Each end effector has its own grip that specifies where to place the robot with respect to the object.

Currently, the notion of grip is not part of the python/tcp/blockly interface 
because it would add an extra layer of complexity that is not really necessary for the moment.

Therefore we have a default grip for all tools that is selected automatically based on the current tool id. However, 
everything is ready if you want to define custom grips, e.g. for custom tools or for custom grip positions.


The vision pick loop
""""""""""""""""""""""""""""""

1. The camera detects objects relative to markers and sends   x\ :sub:`rel`\, y\ :sub:`rel`\, yaw\ :sub:`rel`\ .
2. The object is placed on the workspace, revealing the object pose in robot coordinates x, y, z, roll, pitch, yaw.
3. The grip is applied on the absolute object pose and gives the pose the robot should move to.

Poses & trajectories
""""""""""""""""""""""""""""""""""

List of poses

Parameters - Poses handlers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. list-table:: Poses Handlers' Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``workspace_dir``
      -  Path to the Workspace storage mother folder
   *  -  ``grip_dir``
      -  Path to the Grip storage mother folder
   *  -  ``poses_dir``
      -  Path to the Poses storage mother folder
   *  -  ``dynamic_frame_dir``
      -  Path to the dynamic frames storage mother folder


Services - Poses handlers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Poses Handlers' Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description

   *  -  ``manage_workspace``
      -  :ref:`ManageWorkspace<source/stack/high_level/niryo_robot_poses_handlers:ManageWorkspace (Service)>`
      -  Save/Delete a workspace
   *  -  ``get_workspace_ratio``
      -  :ref:`GetWorkspaceRatio<source/stack/high_level/niryo_robot_poses_handlers:GetWorkspaceRatio (Service)>`
      -  Get ratio of a workspace
   *  -  ``get_workspace_list``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:GetNameDescriptionList`
      -  Get list of workspaces name & description
   *  -  ``get_workspace_poses``
      -  :ref:`GetWorkspaceRobotPoses<source/stack/high_level/niryo_robot_poses_handlers:GetWorkspaceRobotPoses (Service)>`
      -  Get workspace's robot poses
   *  -  ``get_workspace_points``
      -  :ref:`GetWorkspacePoints<source/stack/high_level/niryo_robot_poses_handlers:GetWorkspacePoints (Service)>`
      -  Get workspace's robot points
   *  -  ``get_workspace_matrix_poses``
      -  :ref:`GetWorkspaceMatrixPoses<source/stack/high_level/niryo_robot_poses_handlers:GetWorkspaceMatrixPoses (Service)>`
      -  Get workspace's robot matrix poses

   *  -  ``get_target_pose``
      -  :ref:`GetTargetPose<source/stack/high_level/niryo_robot_poses_handlers:GetTargetPose (Service)>`
      -  Get saved programs name

   *  -  ``manage_pose``
      -  :ref:`ManagePose<source/stack/high_level/niryo_robot_poses_handlers:ManagePose (Service)>`
      -  Save/Delete a Pose
   *  -  ``get_pose``
      -  :ref:`GetPose<source/stack/high_level/niryo_robot_poses_handlers:GetPose (Service)>`
      -  Get Pose
   *  -  ``get_pose_list``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:GetNameDescriptionList`
      -  Get list of poses name & description

   *  -  ``manage_dynamic_frame``
      -  :ref:`ManageDynamicFrame<source/stack/high_level/niryo_robot_poses_handlers:ManageDynamicFrame (Service)>`
      -  Save/Edit/Delete a dynamic frame
   *  -  ``get_dynamic_frame_list``
      -  :ref:`source/stack/high_level/niryo_robot_msgs:GetNameDescriptionList`
      -  Get list of dynamic frame
   *  -  ``get_dynamic_frame``
      -  :ref:`GetDynamicFrame<source/stack/high_level/niryo_robot_poses_handlers:GetDynamicFrame (Service)>`
      -  Get dynamic frame
   *  -  ``get_transform_pose``
      -  :ref:`GetTransformPose<source/stack/high_level/niryo_robot_poses_handlers:GetTransformPose (Service)>`
      -  Get transform between two frames


All these services are available as soon as the node is started.

Dependencies - Poses handlers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- :msgs_index:`geometry_msgs`
- :msgs_index:`moveit_msgs`
- :doc:`niryo_robot_msgs`
- :wiki_ros:`tf`


Services & messages files - Poses handlers
----------------------------------------------

GetDynamicFrame (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetDynamicFrame.srv
   :language: rostype


GetPose (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetPose.srv
   :language: rostype


GetTargetPose (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetTargetPose.srv
   :language: rostype


GetTransformPose (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetTransformPose.srv
   :language: rostype


GetWorkspaceMatrixPoses (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetWorkspaceMatrixPoses.srv
   :language: rostype


GetWorkspacePoints (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetWorkspacePoints.srv
   :language: rostype


GetWorkspaceRatio (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetWorkspaceRatio.srv
   :language: rostype


GetWorkspaceRobotPoses (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/GetWorkspaceRobotPoses.srv
   :language: rostype


ManageDynamicFrame (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/ManageDynamicFrame.srv
   :language: rostype


ManagePose (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/ManagePose.srv
   :language: rostype


ManageWorkspace (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/srv/ManageWorkspace.srv
   :language: rostype


NiryoPose (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/msg/NiryoPose.msg
   :language: rostype


Workspace (Message)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../../niryo_robot_poses_handlers/msg/Workspace.msg
   :language: rostype



.. |namespace| replace:: /niryo_robot_poses_handlers/
.. |namespace_emphasize| replace:: ``/niryo_robot_poses_handlers/``
.. |package_path| replace:: ../../../../niryo_robot_poses_handlers
