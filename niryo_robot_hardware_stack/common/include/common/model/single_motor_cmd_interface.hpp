/*
single_motor_cmd_interface.hpp
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

#ifndef _SINGLE_MOTOR_CMD_INTERFACE_H
#define _SINGLE_MOTOR_CMD_INTERFACE_H

#include <string>
#include <vector>

namespace common
{
namespace model
{

/**
 * @brief The SingleMotorCmd class
 */
class SingleMotorCmdI
{
    public:
        SingleMotorCmdI() {}
        SingleMotorCmdI(uint8_t motor_id,
                       uint32_t param = 0);

        virtual ~SingleMotorCmdI() {};

        // setters
        void setId(uint8_t id);
        void setParam(uint32_t param);
        void setParams(std::vector<int32_t> params); 

        // getters
        uint8_t getId() const;
        uint32_t getParam() const;
        // using in case steppers
        virtual std::vector<int32_t> getParams() const;

        // AbstractMotorCmd interface
        virtual bool isCmdStepper() const;
        virtual bool isCmdDxl() const;
        virtual bool isValid() const;
        virtual std::string str() const;
        // This method help get type of a command through SingleMotorCmd interface
        virtual int getTypeCmd() const;
    protected:
        uint8_t _id;
        uint32_t _param;
};

/**
 * @brief SingleMotorCmdI::SingleMotorCmdI
 * @return
 */

inline SingleMotorCmdI::SingleMotorCmdI(uint8_t motor_id, uint32_t param) :
                _id(motor_id),
                _param(param)
{}

/**
 * @brief SingleMotorCmdI::getId
 * @return
 */
inline
uint8_t SingleMotorCmdI::getId() const
{
    return _id;
}

/**
 * @brief SingleMotorCmdI::getParam
 * @return
 */
inline
uint32_t SingleMotorCmdI::getParam() const
{
    return _param;
}

/**
 * @brief SingleMotorCmdI::getParam
 * @return
 */
inline
std::vector<int32_t> SingleMotorCmdI::getParams() const
{
    return {(int32_t)_param};
}

/**
 * @brief SingleMotorCmdI::setId
 * @param id
 */
inline
void SingleMotorCmdI::setId(uint8_t id)
{
    _id = id;
}

/**
 * @brief SingleMotorCmdI::setParam
 * @param param
 */
inline
void SingleMotorCmdI::setParam(uint32_t param)
{
    _param = param;
}

/**
 * @brief SingleMotorCmdI::setParam
 * @param param
 */
inline
void SingleMotorCmdI::setParams(std::vector<int32_t> params) 
{
    _param = params[0];
}

/**
 * @brief SingleMotorCmdI::isCmdStepper
*/
inline bool SingleMotorCmdI::isCmdStepper() const
{
    return false;
}

/**
 * @brief SingleMotorCmdI::isCmdDxl
*/
inline bool SingleMotorCmdI::isCmdDxl() const
{
    return false;
}

/**
 * @brief SingleMotorCmdI::isValid
*/
inline bool SingleMotorCmdI::isValid() const
{
    return false;
}

/**
 * @brief SingleMotorCmdI::str
*/
inline std::string SingleMotorCmdI::str() const
{
    return "";
}

/**
 * @brief SingleMotorCmdI::getType()
*/
inline int SingleMotorCmdI::getTypeCmd() const
{
    return 0;
}

} // namespace model
} // namespace common

#endif
