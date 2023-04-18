/*
    joints_interface_node.cpp
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
#include <ros/console.h>
#include <ros/ros.h>
#include <string>

// niryo
#include "can_driver/can_interface_core.hpp"
#include "joints_interface/joints_interface_core.hpp"
#include "ttl_driver/ttl_interface_core.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joints_interface_node");

    ROS_DEBUG("Launching joints_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    ros::NodeHandle nh_private("~");
    std::string hw_version;
    nh_private.getParam("hardware_version", hw_version);

    ros::NodeHandle nh_ttl("ttl_driver");
    auto ttl_driver = std::make_shared<ttl_driver::TtlInterfaceCore>(nh_ttl);
    ros::Duration(1).sleep();

    std::shared_ptr<can_driver::CanInterfaceCore> can_driver;
    if (hw_version == "ned")
    {
        ros::NodeHandle nh_can("can_driver");
        can_driver = std::make_shared<can_driver::CanInterfaceCore>(nh_can);
        ros::Duration(1).sleep();
    }

    auto joints = std::make_shared<joints_interface::JointsInterfaceCore>(nh, nh_private, ttl_driver, can_driver);
    ros::waitForShutdown();

    ROS_INFO("Joints Interface - Shutdown node");
}
