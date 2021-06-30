/*
idriver_core.hpp
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

#ifndef IDriverCore_H
#define IDriverCore_H

// C++
#include <string>
#include <thread>
#include <mutex>

// ros
#include <ros/node_handle.h>

// niryo
#include "niryo_robot_msgs/BusState.h"
#include "motor_type_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The IDriverCore class
 */
class IDriverCore
{
    public:
        virtual ~IDriverCore() = 0;

        virtual void startControlLoop() = 0;
        virtual bool isConnectionOk() const = 0;

        virtual void activeDebugMode(bool mode) = 0;

        virtual int launchMotorsReport() = 0;
        virtual niryo_robot_msgs::BusState getBusState() const = 0;

    private:
        virtual void resetHardwareControlLoopRates() = 0;
        virtual void controlLoop() = 0;
        virtual void _executeCommand() = 0;

    protected:

};

/**
 * @brief IDriverCore::~IDriverCore
 */
inline
IDriverCore::~IDriverCore()
{

}

} // namespace model
} // namespace common

#endif
