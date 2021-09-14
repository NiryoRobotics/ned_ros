CPU_interface
=================================

| This package handles CPU states.

CPU interface node
--------------------------
The ROS Node is made to:
 - monitor CPU temperature.

Parameters - CPU interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: CPU Interface's Parameters 
   :header-rows: 1
   :widths: auto
   :stub-columns: 0
   :align: center

   *  -  Name
      -  Description
   *  -  ``read_rpi_diagnostics_frequency``
      -  | Publishes rate for CPU temperature
         | Default: '0.25'
   *  -  ``temperature_warn_threshold``
      -  | CPU temperature [celsius] threshold before a warn message
         | Default: '75'
   *  -  ``temperature_shutdown_threshold``
      -  | CPU temperature [celsius] threshold before shutdown the robot
         | Default: '85'
