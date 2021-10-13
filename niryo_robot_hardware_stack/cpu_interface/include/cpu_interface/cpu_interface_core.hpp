/*
cpu_interface_interface_core.hpp
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

#ifndef CPU_INTERFACE_CORE_HPP
#define CPU_INTERFACE_CORE_HPP

#include <memory>
#include <fstream>
#include <ros/ros.h>
#include <vector>
#include <thread>
#include <string>

#include "common/util/i_interface_core.hpp"

namespace cpu_interface
{

/**
 * @brief The CpuInterfaceCore class
 */
class CpuInterfaceCore : public common::util::IInterfaceCore
{
    public:
        CpuInterfaceCore(ros::NodeHandle& nh);
        ~CpuInterfaceCore() override;

        // non copyable class
        CpuInterfaceCore( const CpuInterfaceCore& ) = delete;
        CpuInterfaceCore( CpuInterfaceCore&& ) = delete;

        CpuInterfaceCore& operator= ( CpuInterfaceCore && ) = delete;
        CpuInterfaceCore& operator= ( const CpuInterfaceCore& ) = delete;

        bool init(ros::NodeHandle& nh) override;

        void startReadingData();
        int getCpuTemperature() const;

    private:
        void initParameters(ros::NodeHandle& nh) override;
        void startServices(ros::NodeHandle& nh) override;
        void startPublishers(ros::NodeHandle& nh) override;
        void startSubscribers(ros::NodeHandle& nh) override;

        void _readCpuTemperature();
        void _readHardwareDataLoop();

    private:
        std::thread _read_hardware_data_thread;

        bool _simulation_mode{false};
        int _cpu_temperature{0};

        double _read_cpu_frequency{0.0};
        int _temperature_warn_threshold{0};
        int _temperature_shutdown_threshold{0};
};

/**
 * @brief CpuInterfaceCore::getCpuTemperature
 * @return
 */
inline
int CpuInterfaceCore::getCpuTemperature() const
{
    return _cpu_temperature;
}

} // CpuInterface

#endif
