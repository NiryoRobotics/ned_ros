.. _install_for_ubuntu_20:

Ubuntu 20.04 installation
=========================

This guide will explain the steps needed to install the Niryo Robot Stack on an Ubuntu 20.04 computer.
You can apply these steps to set up a working simulation environment on any development computer, or to set up a working robot stack
on a Raspberry Pi.

.. note::
    All terminal command listed are for Ubuntu users.

Installation index:

.. contents::
   :local:
   :depth: 1

Prerequisites
*************

The Niryo ROS Stack runs on top of ROS Noetic. This version of ROS is strongly dependent of Ubuntu 20.04 version,
thus, this OS is currently the only official supported OS.

Be sure to have an up to date system before continuing:

.. code::

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get dist-upgrade

Install ROS
***********

You need to install ROS Noetic. To do so, follow the ROS official tutorial
`here <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ and chose the
**Desktop-Full Install**.


Set up your workspace
*********************

The Ned ROS Stack is a set of ROS packages that need to be compiled in a catkin workspace.
Place yourself in the folder of your choice and create a folder
**catkin_ws_niryo** as well as a sub-folder **src**:

.. code:: bash

    mkdir -p ~/catkin_ws_niryo/src

Then go to the **src/** folder then clone the robot repository and init its submodules.

.. code:: bash

    cd ~/catkin_ws_niryo/src/
    git clone https://github.com/NiryoRobotics/ned_ros.git
    cd ned_ros
    git submodule update --init ros-foxglove-bridge



Install dependencies
********************
To ensure the functioning of all Ned's packages, you need to
install several more packages:

* build-essential
* sqlite3
* ffmpeg

.. code:: bash

    sudo apt install sqlite3 ffmpeg build-essential -y

.. note::
    These packages are mostly useful on a real robot, 
    but as the code is identical between simulation and real functioning, a lack of these packages on a simulation can lead to unstabilities.

For each Ned ROS package, we have referenced all the dependencies in their respective
*package.xml* file, which allows to install each dependency via *rosdep* command.
We also provide a *requirements.txt* file which lists all the required dependencies for python code which can be installed via *pip*.

.. code:: bash

    cd ~/catkin_ws_niryo
    pip install -r src/ned_ros/requirements.txt
    rosdep update 
    rosdep install --from-paths src --ignore-src -r -y

Build the packages
******************

Go to the root of your workspace and perform the **build** of all packages:

.. code:: bash

    cd ~/catkin_ws_niryo
    catkin_make install

If no errors occurred during the **build** phase, the setup
of your environment is almost complete!


Then, in order to run the built ROS nodes, it is necessary to source the configuration file to add them to the environment.
To do so, run the command:

.. code:: bash

    source ~/catkin_ws_niryo/install/setup.bash

It is necessary to run this command each time you launch a new terminal.
If you want to make this sourcing happen automatically for all terminals,
you can add it to your **.bashrc** file:

.. code:: bash

    echo "source ~/catkin_ws_niryo/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Installation is now finished!
