/*
    can_driver_node.cpp
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
#include "can_driver/can_interface_core.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_driver_node");

    ROS_DEBUG("Launching can_driver_node");

    ros::NodeHandle nodeHandle("~");

    can_driver::CanInterfaceCore can_node(nodeHandle);

    ros::spin();
    return 0;
}
