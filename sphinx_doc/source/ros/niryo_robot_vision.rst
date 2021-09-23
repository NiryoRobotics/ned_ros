Niryo_robot_vision
======================================

This package is the one dealing with all vision related stuff.


Vision Node
--------------------------
The ROS Node is made of several services to deal with video streaming, object detection...
The node is working exactly the same way if you chose to use it on simulation or reality.

This node can be launched locally in a standalone mode via the command: ::

 roslaunch niryo_robot_vision vision_node_local.launch

Configuration (Frame Per Second, Camera Port, Video Resolution) can be
edited in the config file:

 - For "standard" Node: *niryo_robot_vision/config/video_server_setup.yaml*
 - For local Node: *niryo_robot_vision/config/video_server_setup_local.yaml*

The namespace used is: |namespace_emphasize|

Parameters - Vision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Vision Package's Parameters
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``frame_rate``
      -  Streams frame rate
   *  -  ``simulation_mode``
      -  | Sets to true if you are using the gazebo simulation.
         | It will adapt how the node get its video stream
   *  -  ``debug_compression_quality``
      -  Debugs Stream compression quality
   *  -  ``stream_compression_quality``
      -  Streams compression quality
   *  -  ``subsampling``
      -  Streams subsampling factor


Publisher - Vision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Vision Package's Publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``compressed_video_stream``
      -  :sensor_msgs:`CompressedImage`
      -  Publishes the last image read as a compressed image
   *  -  ``video_stream_parameters``
      -  :ref:`ImageParameters<source/ros/niryo_robot_vision:ImageParameters (Topic)>`
      -  Publishes the brightness, contrast and saturation settings of the video stream

Services - Vision
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Programs manager Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``debug_colors``
      -  :ref:`DebugColorDetection<source/ros/niryo_robot_vision:DebugColorDetection (Service)>`
      -  Returns an annotated image to emphasize what happened with color detection
   *  -  ``debug_markers``
      -  :ref:`DebugMarkers<source/ros/niryo_robot_vision:DebugMarkers (Service)>`
      -  Returns an annotated image to emphasize what happened with markers detection
   *  -  ``obj_detection_rel``
      -  :ref:`ObjDetection<source/ros/niryo_robot_vision:ObjDetection (Service)>`
      -  Object detection service
   *  -  ``start_stop_video_streaming``
      -  :ref:`source/ros/niryo_robot_msgs:SetBool`
      -  Starts or stops video streaming
   *  -  ``take_picture``
      -  :ref:`TakePicture<source/ros/niryo_robot_vision:TakePicture (Service)>`
      -  Saves a picture in the specified folder
   *  -  ``set_brightness``
      -  :ref:`SetImageParameter<source/ros/niryo_robot_vision:SetImageParameter (Service)>`
      -  Sets the brightness of the video stream
   *  -  ``set_contrast``
      -  :ref:`SetImageParameter<source/ros/niryo_robot_vision:SetImageParameter (Service)>`
      -  Sets the contrast of the video stream
   *  -  ``set_saturation``
      -  :ref:`SetImageParameter<source/ros/niryo_robot_vision:SetImageParameter (Service)>`
      -  Sets the saturation of the video stream


All these services are available as soon as the node is started.


Dependencies - Vision
^^^^^^^^^^^^^^^^^^^^^^^^^^^
* :doc:`niryo_robot_msgs`
* :msgs_index:`sensor_msgs`


Topics files - Vision
--------------------------

ImageParameters (Topic)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_vision/msg/ImageParameters.msg
   :language: rostype



Services files - Vision
--------------------------

DebugColorDetection (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_vision/srv/DebugColorDetection.srv
   :language: rostype


DebugMarkers (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_vision/srv/DebugMarkers.srv
   :language: rostype


ObjDetection (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_vision/srv/ObjDetection.srv
   :language: rostype


TakePicture (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_vision/srv/TakePicture.srv
   :language: rostype

SetImageParameter (Service)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../../niryo_robot_vision/srv/SetImageParameter.srv
   :language: rostype


.. |namespace| replace:: /niryo_robot_vision/
.. |namespace_emphasize| replace:: ``/niryo_robot_vision/``
.. |package_path| replace:: ../../../niryo_robot_vision
