/*
    hardware_interface_node.cpp
    Copyright (C) 2020 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

// ros
#include <ros/ros.h>

// niryo
#include "niryo_robot_hardware_interface/hardware_interface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interface_node");

    ROS_DEBUG("Launching hardware_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh("~");

    niryo_robot_hardware_interface::HardwareInterface nd(nh);
    ros::waitForShutdown();

    ROS_INFO("Hardware Interface - Shutdown node");
}
