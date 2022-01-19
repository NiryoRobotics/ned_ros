Ubuntu 18 Installation
=========================================

This guide will explain the steps needed to install the Niryo Robot Stack on an Ubuntu 18 OS.
You can apply these steps to set up a working simulation environment on any development computer, or to set up a working robot stack
on a Raspberry Pi.

Installation index:

.. contents::
   :local:
   :depth: 1

Prerequisites
-------------------------

The Niryo ROS Stack runs on top of ROS Melodic or Kinetic (deprecated). This version of ROS is strongly dependent of Ubuntu 18.04 version,
thus, this OS is currently the only official supported OS.

Be sure to have an up to date system before continuing

.. code::

    sudo apt-get update
    sudo apt-get upgrade
    sudo apt-get dist-upgrade

Ubuntu packages
************************************************

The Niryo ROS Stack needs the following packages in order to run correctly: 

* build-essential
* sqlite3
* ffmpeg

.. note::
    These packages are mostly useful on a real robot, 
    but as the code is identical between simulation and real functioning, a lack of these packages on a simulation can lead to unstabilities.


Python environment
************************************************

The Python environment is installed using the requirements_ned2.txt file

.. code::

    pip2 install -r src/requirements_ned2.txt

.. note::
    ROS Melodic is still using Python2 internally so are our Python scripts to keep version alignment.
    You thus need to install the requirements using Python2 pip2 tool

ROS set up
************************************************

.. note::
    All terminal command listed are for Ubuntu users.


Place yourself in the folder of your choice and create a folder
**catkin_ws_niryo_ned** as well as a sub-folder **src**: ::

    mkdir -p catkin_ws_niryo_ned/src

Then go to the folder **catkin_ws_niryo_ned** and
clone Ned repository in the folder **src**.
For the future operations, be sure to stay in the **catkin_ws_niryo_ned** folder: ::

    cd catkin_ws_niryo_ned
    git clone https://github.com/NiryoRobotics/ned_ros src


Install ROS dependencies
------------------------------------

Install ROS
************************************************

You need to install ROS Melodic. To do so, follow the ROS official tutorial
`here <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ and chose the
**Desktop-Full Install**.

Install additional packages
************************************************
To ensure the functioning of all Ned's packages, you need to
install several more packages:

Method 1: Quick installation via ROSDep
"""""""""""""""""""""""""""""""""""""""""""""""""
For each package, we have referenced all the dependencies in their respective
*package.xml* file, which allows to install each dependency via *rosdep* command: ::

 rosdep update
 rosdep install --from-paths src --ignore-src --default-yes --rosdistro melodic --skip-keys "python-rpi.gpio"


Method 2: Full installation
"""""""""""""""""""""""""""""""""""""""""""""""""

ROS packages needed are:

* catkin
* python-catkin-pkg
* python-pymodbus
* python-rosdistro
* python-rospkg
* python-rosdep-modules
* python-rosinstall python-rosinstall-generator
* python-wstool

To install a package on Ubuntu: ::

    sudo apt install <package_name>


Melodic specific packages needed are:

* moveit
* control
* controllers
* tf2-web-republisher
* rosbridge-server
* joint-state-publisher-gui

To install a ROS Melodic's package on Ubuntu: ::

    sudo apt install ros-melodic-<package_name>


Setup Ned ROS environment
--------------------------------

.. note::
    Be sure to be still placed in the **catkin_ws_niryo_ned** folder.

Then perform the **make** of Ned's ROS Stack via the command: ::

    catkin_make

If no errors occurred during the **make** phase, the setup
of your environment is almost complete!

It is necessary to source the configuration file to add all Ned
packages to ROS environment. To do so, run the command: ::

    source devel/setup.bash

It is necessary to run this command each time you launch a new terminal.
If you want to make this sourcing appends for all your future terminals,
you can add it to your **bashrc** file: ::

    echo "source $(pwd)/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Installation is now finished!
