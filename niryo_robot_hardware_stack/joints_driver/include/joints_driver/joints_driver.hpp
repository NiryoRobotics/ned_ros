/*
    joint_driver.hpp
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

#ifndef _JOINTS_DRIVER_HPP
#define _JOINTS_DRIVER_HPP

#include <ros/ros.h>
#include <ttl_driver/ttl_interface_core.hpp>
#include <can_driver/can_interface_core.hpp>

#include <memory>
#include <map>
#include <string>

namespace joint_driver
{
    class JointDriver {
    public:
        JointDriver(ros::NodeHandle &nh);
        ~JointDriver();

        bool haveCan() const;
        bool haveTtl() const;

        // getters
        std::shared_ptr<ttl_driver::TtlInterfaceCore> getTtlInterfaceCore() const;
        std::shared_ptr<can_driver::CanInterfaceCore> getCanInterfaceCore() const;
        std::shared_ptr<common::model::IDriverCore> getProtocolOfMotor(std::string name) const;
    private:
        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttlInterfaceCore;
        std::shared_ptr<can_driver::CanInterfaceCore> _canInterfaceCore;

        std::map<std::string, std::shared_ptr<common::model::IDriverCore>> _m_name_proto; 
        
        bool _haveCan;
        bool _haveTtl;
        void init(ros::NodeHandle &nh);
    };
}   // joint_driver

#endif // _JOINTS_DRIVER_HPP