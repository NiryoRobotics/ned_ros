Modbus
============

.. image:: ../images/modbus_logo.jpg
         :alt: Modbus logo
         :width: 400px
         :align: center

In this document, we will focus on the Modbus/TCP server.

Ned is permanently running a Modbus TCP Server that enables Ned to communicate with a PLC, or another computer in the same network.

The Modbus/TCP server is running on **port 5020** by default.
It has been built on top of the :pymodbus:`pymodbus<>` library.
This enables you to make Ned communicates with a PLC, or another computer on the same network.


.. toctree::
   :maxdepth: 2
   :caption: Modbus

   modbus/setup
   modbus/api_documentation
   modbus/modbus_examples