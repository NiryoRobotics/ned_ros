/*
isynchronize_motor_cmd.hpp
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

#ifndef _ISYNCHRONIZE_MOTOR_CMD_H
#define _ISYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <sstream>
#include <typeinfo>

#include "common/model/iobject.hpp"

#include "common/model/motor_type_enum.hpp"
#include "common/model/joint_state.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SynchronizeMotorCmd class
 */

class ISynchronizeMotorCmd : public IObject
{
    public:
        virtual ~ISynchronizeMotorCmd() override = 0;

        // setters
        virtual void addMotorParam(EMotorType type, uint8_t id, uint32_t param) = 0;

        // getters
        virtual std::vector<uint8_t> getMotorsId(EMotorType type) const = 0;
        virtual std::vector<uint32_t> getParams(EMotorType type) const = 0;
        virtual std::set<EMotorType> getMotorTypes() const = 0;

        // This method help get type of a command through SynchronizeMotorCmd interface
        virtual int getType() const = 0;

        virtual bool isCmdStepper() const = 0;
        virtual bool isCmdDxl() const = 0;

    // IObject interface
    public:
        virtual void reset() override = 0;
        virtual std::string str() const override = 0;
        virtual bool isValid() const override = 0;
};

/**
 * @brief ISynchronizeMotorCmd::~ISynchronizeMotorCmd
 */
inline
ISynchronizeMotorCmd::~ISynchronizeMotorCmd()
{
}

} // namespace model
} // namespace common

#endif // _ISYNCHRONIZE_MOTOR_CMD_H
