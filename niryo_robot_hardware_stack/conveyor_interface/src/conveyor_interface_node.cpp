/*
    conveyor_interface_node.cpp
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
#include <memory>
#include <ros/ros.h>
#include <string>

// niryo
#include "can_driver/can_interface_core.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ttl_driver/ttl_interface_core.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conveyor_interface_node");

    ROS_DEBUG("Launching conveyor_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh("~");
    ros::NodeHandle nh_conveyor("conveyor");

    std::string hardware_version;

    nh.getParam("hardware_version", hardware_version);

    std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_driver;
    std::shared_ptr<can_driver::CanInterfaceCore> can_driver;

    if (hardware_version == "ned2" || hardware_version == "ned3pro")
    {
        ros::NodeHandle nh_ttl("ttl_driver");
        ttl_driver = std::make_shared<ttl_driver::TtlInterfaceCore>(nh_ttl);
        ros::Duration(0.25).sleep();
    }

    if (hardware_version == "ned" || hardware_version == "one")
    {
        ros::NodeHandle nh_can("can_driver");
        can_driver = std::make_shared<can_driver::CanInterfaceCore>(nh_can);
        ros::Duration(0.25).sleep();
    }

    auto conveyor_interface = std::make_shared<conveyor_interface::ConveyorInterfaceCore>(nh_conveyor, ttl_driver, can_driver);
    ros::Duration(0.25).sleep();

    ros::waitForShutdown();
    return 0;
}
