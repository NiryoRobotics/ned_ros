/*
    end_effector_interface_core.cpp
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

// c++
#include <functional>
#include <string>
#include <vector>

// ros

// niryo
#include "end_effector_interface/end_effector_interface_core.hpp"
#include "ttl_driver/ttl_manager.hpp"

namespace end_effector_interface
{

/**
 * @brief EndEffectorInterfaceCore::EndEffectorInterfaceCore
 * @param nh
 * @param ttl_interface
 */
EndEffectorInterfaceCore::EndEffectorInterfaceCore(ros::NodeHandle& nh,
                                       std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface):
    _ttl_interface(ttl_interface)
{
    ROS_DEBUG("EndEffectorInterfaceCore::ctor");

    init(nh);
}

/**
 * @brief EndEffectorInterfaceCore::~EndEffectorInterfaceCore
 */
EndEffectorInterfaceCore::~EndEffectorInterfaceCore()
{
   
}

/**
 * @brief EndEffectorInterfaceCore::init
 * @param nh
 * @return
 */
bool EndEffectorInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("EndEffectorInterfaceCore::init - Initializing parameters...");
    initParameters(nh);

    ROS_DEBUG("EndEffectorInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("EndEffectorInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("EndEffectorInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief EndEffectorInterfaceCore::initParameters
 * @param nh
 */
void EndEffectorInterfaceCore::initParameters(ros::NodeHandle& nh)
{
        ROS_DEBUG("No parameters to init");

}

/**
 * @brief EndEffectorInterfaceCore::startServices
 * @param nh
 */
void EndEffectorInterfaceCore::startServices(ros::NodeHandle& nh)
{
    ROS_DEBUG("No services to start");
}

/**
 * @brief EndEffectorInterfaceCore::startPublishers
 * @param nh
 */
void EndEffectorInterfaceCore::startPublishers(ros::NodeHandle& nh)
{
    ROS_DEBUG("No publishers to start");
}

/**
 * @brief EndEffectorInterfaceCore::startSubscribers
 * @param nh
 */
void EndEffectorInterfaceCore::startSubscribers(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("No subscribers to start");
}

}  // namespace end_effector_interface
