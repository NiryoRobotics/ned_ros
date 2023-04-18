Niryo_robot_system_api_client
========================================

This packages contains a small HTTP Client that the packages can import in order to contact the flask server.


Publisher - System API Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: System API Client Package's Publishers
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/wifi/status``
      -  :ref:`WifiStatus<source/stack/high_level/niryo_robot_system_api_client:WifiStatus (Message)>`
      -  Publish the current wifi and hotspot status

Messages files - System API Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

WifiStatus (Message)
---------------------------

.. literalinclude:: ../../../../niryo_robot_system_api_client/msg/WifiStatus.msg
   :language: rostype

.. |namespace| replace:: /niryo_robot_system_api_client/
.. |namespace_emphasize| replace:: ``/niryo_robot_system_api_client/``