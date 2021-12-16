Ubuntu 18 Installation
=========================================


To allow the simulation to run on your computer, you will need to install ROS and some
packages.

Installation index:

.. contents::
   :local:
   :depth: 1

Prerequisites
-------------------------
.. note::
    All terminal command listed are for Ubuntu users.

Place yourself in the folder of your choice and create a folder
**catkin_ws_niryo_ned** as well as a sub-folder **src**: ::

    mkdir -p catkin_ws_niryo_ned/src

Then go to the folder **catkin_ws_niryo_ned** and
clone Ned repository in the folder **src**.
For the future operation, be sure to stay in the **catkin_ws_niryo_ned** folder: ::

    cd catkin_ws_niryo_ned
    git clone https://github.com/NiryoRobotics/ned_ros src


Install ROS dependencies
------------------------------------

Install ROS
^^^^^^^^^^^^^

You firstly need to install ROS Melodic. To do so, follow the ROS official tutorial
`here <http://wiki.ros.org/melodic/Installation/Ubuntu>`_ and chose the
**Desktop-Full Install**.

Install additional packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To ensure the functioning of all Ned's packages, you need to
install several more packages:

Method 1: Quick installation via ROSDep
"""""""""""""""""""""""""""""""""""""""""""""""""
For each packages, we have referenced all the dependencies in their respective
*package.xml* file, which allow to install each dependency via *rosdep* command: ::

 rosdep update
 rosdep install --from-paths src --ignore-src --default-yes --rosdistro melodic --skip-keys "python-rpi.gpio"


Method 2: Full installation
"""""""""""""""""""""""""""""""""""""""""""""""""

ROS packages needed are:

* build-essential
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
