/*
    tools_interface_node.cpp
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

// stdio
#include <memory>

// ros
#include <ros/ros.h>

// niryo
#include "tools_interface/tools_interface_core.hpp"
#include "ttl_driver/ttl_interface_core.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tools_interface_node");

    ROS_DEBUG("Launching tools_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh_ttl("ttl_driver");
    ros::NodeHandle nh("~");

    auto ttl_driver = std::make_shared<ttl_driver::TtlInterfaceCore>(nh_ttl);
    ros::Duration(1).sleep();

    auto tool = std::make_shared<tools_interface::ToolsInterfaceCore>(nh, ttl_driver);
    ros::Duration(1).sleep();

    ros::waitForShutdown();

    ROS_INFO("Tools Interface - Shutdown node");
}
