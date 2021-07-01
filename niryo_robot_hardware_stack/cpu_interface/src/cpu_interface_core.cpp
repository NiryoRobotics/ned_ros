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
    ROS_DEBUG("CPU Interface Core - ctor");

    init(nh);

    startReadingData();

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
 * @brief CpuInterfaceCore::init
 */
bool CpuInterfaceCore::init(ros::NodeHandle& nh)
{
    initParameters(nh);

    ROS_DEBUG("CpuInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("CpuInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    ROS_DEBUG("CpuInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    return true;
}

/**
 * @brief CpuInterfaceCore::initParameters
 */
void CpuInterfaceCore::initParameters(ros::NodeHandle& nh)
{
    if (nh.hasParam("read_rpi_diagnostics_frequency/read_rpi_diagnostics_frequency"))
    {
        nh.getParam("read_rpi_diagnostics_frequency/read_rpi_diagnostics_frequency", _read_cpu_frequency);
    }
    else
    {
        ROS_INFO("CPU Interface::initParameters - No params found. Init with default value");
        nh.getParam("read_rpi_diagnostics_frequency", _read_cpu_frequency);
        nh.getParam("temperature_warn_threshold", _temperature_warn_threshold);
        nh.getParam("temperature_shutdown_threshold", _temperature_shutdown_threshold);
    }

    ROS_DEBUG("CPU Interface::initParameters - Read temperature frequency %f", _read_cpu_frequency);
    ROS_DEBUG("CPU Interface::initParameters - Temperature warn threshold %d", _temperature_warn_threshold);
    ROS_DEBUG("CPU Interface::initParameters - Temperature shutdown threshold %d", _temperature_shutdown_threshold);
}

/**
 * @brief CpuInterfaceCore::startServices
 */
void CpuInterfaceCore::startServices(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("CpuInterfaceCore::startServices - no services to start");
}

/**
 * @brief CpuInterfaceCore::startSubscribers
 */
void CpuInterfaceCore::startSubscribers(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("CpuInterfaceCore::startSubscribers - no subscribers to start");
}

/**
 * @brief CpuInterfaceCore::startPublishers
 */
void CpuInterfaceCore::startPublishers(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("CpuInterfaceCore::startPublishers - no publishers to start");
}

/**
 * @brief CpuInterfaceCore::_readCpuTemperature
 */
void CpuInterfaceCore::_readCpuTemperature()
{
    std::fstream cpu_temp_file("/sys/class/thermal/thermal_zone0/temp", std::fstream::in);
    if (cpu_temp_file.good())
    {
        int read_temp = 0;
        cpu_temp_file >> read_temp;
        if (read_temp > 0)
        {
            _cpu_temperature = read_temp / 1000;
        }

        cpu_temp_file.close();
    }
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
