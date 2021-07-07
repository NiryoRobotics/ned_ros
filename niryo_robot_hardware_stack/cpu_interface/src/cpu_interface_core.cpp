/*
    cpu_interface_core.cpp
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

#include "cpu_interface/cpu_interface_core.hpp"
#include <functional>

namespace cpu_interface
{

/**
 * @brief CpuInterfaceCore::CpuInterfaceCore
 */
CpuInterfaceCore::CpuInterfaceCore(ros::NodeHandle& nh)
{
    initParams();
    startReadingData();
    ROS_DEBUG("CPU Interface Core - ctor");

    ROS_INFO("CPU Interface - Started");
}

/**
 * @brief CpuInterfaceCore::~CpuInterfaceCore
 */
CpuInterfaceCore::~CpuInterfaceCore()
{
    if (_read_hardware_data_thread.joinable())
        _read_hardware_data_thread.join();
}

/**
 * @brief CpuInterfaceCore::initParams
 */
void CpuInterfaceCore::initParams()
{
    if (ros::param::has("read_rpi_diagnostics_frequency/read_rpi_diagnostics_frequency"))
    {
        ros::param::get("read_rpi_diagnostics_frequency/read_rpi_diagnostics_frequency", _read_cpu_frequency);
    }
    else
    {
        ROS_INFO("CPU Interface - No params found. Init with default value");
        _nh.getParam("/niryo_robot_hardware_interface/read_rpi_diagnostics_frequency", _read_cpu_frequency);
        _nh.getParam("/niryo_robot_hardware_interface/temperature_warn_threshold", _temperature_warn_threshold);
        _nh.getParam("/niryo_robot_hardware_interface/temperature_shutdown_threshold", _temperature_shutdown_threshold);
    }
    ROS_DEBUG("CPU Interface - Read temperature frequency %f", _read_cpu_frequency);
}

/**
 * @brief CpuInterfaceCore::_readCpuTemperature
 */
void CpuInterfaceCore::_readCpuTemperature()
{
#if defined __arm__ || defined __aarch64__
    std::fstream cpu_temp_file("/sys/class/thermal/thermal_zone0/temp", std::ios_base::in);

    int read_temp;
    cpu_temp_file >> read_temp;
    if (read_temp > 0)
    {
        _cpu_temperature = read_temp / 1000;
    }
#endif
}

/**
 * @brief CpuInterfaceCore::startReadingData
 */
void CpuInterfaceCore::startReadingData()
{
    _read_hardware_data_thread = std::thread(&CpuInterfaceCore::_readHardwareDataLoop, this);
}

/**
 * @brief CpuInterfaceCore::_readHardwareDataLoop
 */
void CpuInterfaceCore::_readHardwareDataLoop()
{
    ros::Rate read_rpi_diagnostics_rate = ros::Rate(_read_cpu_frequency);

    while (ros::ok())
    {
        _readCpuTemperature();

        // check if Rpi is too hot
        if (_cpu_temperature > _temperature_warn_threshold)
        {
            ROS_WARN("CPU Interface - Rpi temperature is really high !");
        }
        if (_cpu_temperature > _temperature_shutdown_threshold)
        {
            ROS_ERROR("CPU Interface - Rpi is too hot, shutdown to avoid any damage");
            int ret = std::system("sudo shutdown now");
            ROS_INFO("Shutdown now: %d", ret);
        }

        read_rpi_diagnostics_rate.sleep();
    }
}
}  // namespace cpu_interface
