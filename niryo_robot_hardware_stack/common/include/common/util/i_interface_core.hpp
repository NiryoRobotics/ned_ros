/*
i_interface_core.hpp
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

#ifndef I_INTERFACE_CORE_H
#define I_INTERFACE_CORE_H

#include <cstdint>
#include <string>
#include <vector>

#include "ros/node_handle.h"

namespace common
{
namespace util
{

/**
 * @brief The IInterfaceCore class is an interface intended to be used as a polymorphic base class
 */
class IInterfaceCore
{
public:
    virtual ~IInterfaceCore() = default;
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    IInterfaceCore( const IInterfaceCore& ) = delete;
    IInterfaceCore( IInterfaceCore&& ) = delete;
    IInterfaceCore& operator= ( IInterfaceCore && ) = delete;
    IInterfaceCore& operator= ( const IInterfaceCore& ) = delete;

    virtual bool init(ros::NodeHandle& nh) = 0;

protected:
    IInterfaceCore() = default;

private:
    virtual void initParameters(ros::NodeHandle& nh) = 0;
    virtual void startServices(ros::NodeHandle& nh) = 0;
    virtual void startPublishers(ros::NodeHandle& nh) = 0;
    virtual void startSubscribers(ros::NodeHandle& nh) = 0;
};

} // namespace util
} // namespace common

#endif // I_INTERFACE_CORE
