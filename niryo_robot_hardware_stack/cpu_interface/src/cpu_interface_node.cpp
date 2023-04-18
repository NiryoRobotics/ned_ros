/*
    cpu_interface_node.cpp
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

// std
#include <memory>

// ros
#include <ros/ros.h>

// niryo
#include "cpu_interface/cpu_interface_core.hpp"

/**
 * @brief readTemperature
 * @param cpu
 */
void readTemperature(const std::shared_ptr<cpu_interface::CpuInterfaceCore> &cpu)
{
    ros::Rate read_rpi_diagnostics_rate = ros::Rate(1);

    while (ros::ok())
    {
        int temperature = cpu->getCpuTemperature();
        ROS_INFO("cpu temperature = %d", temperature);
        read_rpi_diagnostics_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cpu_interface_node");

    ROS_DEBUG("Launching cpu_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh("~");

    auto cpu = std::make_shared<cpu_interface::CpuInterfaceCore>(nh);

    std::thread readTempThread(readTemperature, cpu);
    readTempThread.join();
    ros::waitForShutdown();

    ROS_INFO("shutdown node");
}
