/*
isingle_motor_cmd.hpp
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

#ifndef _ISINGLE_MOTOR_CMD_H
#define _ISINGLE_MOTOR_CMD_H

#include <string>
#include <vector>

#include "common/model/iobject.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SingleMotorCmd class
 */
class ISingleMotorCmd : public IObject
{
    public:
        virtual ~ISingleMotorCmd() override = 0;

        // setters
        virtual void setId(uint8_t id) = 0;
        virtual void setParam(uint32_t param) = 0;
        virtual void setParams(std::vector<int32_t> params) = 0;

        // getters
        virtual uint8_t getId() const = 0;
        virtual uint32_t getParam() const = 0;
        // using in case steppers
        virtual std::vector<int32_t> getParams() const = 0;

        virtual bool isCmdStepper() const = 0;
        virtual bool isCmdDxl() const = 0;

        virtual int getTypeCmd() const = 0;

    // IObject interface
    public:
        virtual void reset() override = 0;
        virtual std::string str() const override = 0;
        virtual bool isValid() const override = 0;
};

/**
 * @brief ISingleMotorCmd::~ISingleMotorCmd
 */
inline
ISingleMotorCmd::~ISingleMotorCmd()
{

}

} // namespace model
} // namespace common

#endif // _ISINGLE_MOTOR_CMD_H
