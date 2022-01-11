Niryo_robot_system_api_client
========================================

This packages handle the flask server requests to manage:
 - Robot name
 - Wifi settings
 - Ethernet settings


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
      -  Publish the current wifi status



Services - System API Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: System API Client Services
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Message Type
      -  Description
   *  -  ``/niryo_robot/wifi/set_robot_name``
      -  :ref:`SetString<source/stack/high_level/niryo_robot_msgs:SetString>`
      -  Change the robot name
   *  -  ``/niryo_robot/wifi/manage``
      -  :ref:`ManageWifi<source/stack/high_level/niryo_robot_system_api_client:ManageWifi (Service)>`
      -  Change the wifi hotspot mode
   *  -  ``/niryo_robot/ethernet/manage``
      -  :ref:`ManageEthernet<source/stack/high_level/niryo_robot_system_api_client:ManageEthernet (Service)>`
      -  Change the ethernet setup (ip address, netmask, gateway, dhcp) based on nmcli interface.


Services files -  System API Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ManageEthernet (Service)
---------------------------

.. literalinclude:: ../../../../niryo_robot_system_api_client/srv/ManageEthernet.srv
   :language: rostype

ManageWifi (Service)
---------------------------

.. literalinclude:: ../../../../niryo_robot_system_api_client/srv/ManageWifi.srv
   :language: rostype

Messages files - System API Client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

WifiStatus (Message)
---------------------------

.. literalinclude:: ../../../../niryo_robot_system_api_client/msg/WifiStatus.msg
   :language: rostype

.. |namespace| replace:: /niryo_robot_system_api_client/
.. |namespace_emphasize| replace:: ``/niryo_robot_system_api_client/``