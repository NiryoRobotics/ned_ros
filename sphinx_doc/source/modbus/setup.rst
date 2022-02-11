Modbus Python library installation
=========================================

To use the Modbus Python library, your workspace must have a Python interpreter with **Python 3** (3.6 or greater) or **Python 2** (2.7 or greater).

.. note:: 
  Download Python on the official :python_website:`Python website<>` and find more information about the installation on :python_installation:`this website<>`.


This installation requires the use of :pip_website:`pip<>`, the package manager included in Python.


Start with the installation of numpy: ::

    pip install numpy

To use the **Modbus API**, you also need to install Modbus python library :pymodbus:`pymodbus<>`: ::

    pip install pymodbus

.. attention::

    - Pip can require administrator authorizations to install packages. In this case, add ::
        
        sudo 

      before your command lines on Linux.

    - If pip is not automatically installed with Python, please visit the following website:
      :pip_website:`pip installation<>`.
