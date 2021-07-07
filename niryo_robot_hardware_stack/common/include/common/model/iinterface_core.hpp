/*
idriver.hpp
Copyright (C) 2017 Niryo
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

#ifndef IINTERFACE_CORE_H
#define IINTERFACE_CORE_H

#include <stdint.h>
#include <string>
#include <vector>
#include "joint_state.hpp"

#include "ros/node_handle.h"

namespace common
{
namespace model
{

/**
 * @brief The IInterfaceCore class
 */
class IInterfaceCore
{
    public:
        virtual ~IInterfaceCore() = 0;
        virtual bool init(ros::NodeHandle& nh) = 0;

    private:
        virtual void initParameters(ros::NodeHandle& nh) = 0;
        virtual void startServices(ros::NodeHandle& nh) = 0;
        virtual void startSubscribers(ros::NodeHandle& nh) = 0;
        virtual void startPublishers(ros::NodeHandle& nh) = 0;
};

/**
 * @brief IInterfaceCore::~IInterfaceCore
 */
inline
IInterfaceCore::~IInterfaceCore()
{

}

} // namespace model
} // namespace common

#endif // IINTERFACE_CORE
