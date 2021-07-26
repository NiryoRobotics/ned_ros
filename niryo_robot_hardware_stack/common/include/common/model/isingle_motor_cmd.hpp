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
class ISingleMotorCmd
{
    public:
        ISingleMotorCmd() {}

        ISingleMotorCmd(uint8_t motor_id,
                       uint32_t param = 0);

        ISingleMotorCmd(uint8_t motor_id, std::vector<int32_t> params = std::vector<int32_t>());

        virtual ~ISingleMotorCmd() {};

        // setters
        void setId(uint8_t id);
        void setParam(uint32_t param);
        void setParams(std::vector<int32_t> params); 

        // getters
        uint8_t getId() const;
        uint32_t getParam() const;
        // using in case steppers
        virtual std::vector<int32_t> getParams() const;

        virtual bool isCmdStepper() const;
        virtual bool isCmdDxl() const;
        virtual bool isValid() const;
        virtual std::string str() const;
        // This method help get type of a command through SingleMotorCmd interface
        virtual int getTypeCmd() const;
    protected:
        uint8_t _id;
        uint32_t _param;
        std::vector<int32_t> _param_list;
};

/**
 * @brief ISingleMotorCmd::ISingleMotorCmd
 * @return
 */

inline ISingleMotorCmd::ISingleMotorCmd(uint8_t motor_id, uint32_t param) :
                _id(motor_id),
                _param(param)
{}

/**
 * @brief ISingleMotorCmd::ISingleMotorCmd
 * @param type
 * @param motor_id
 * @param params
 */
inline 
ISingleMotorCmd::ISingleMotorCmd(uint8_t motor_id,
                                 std::vector<int32_t> params) :
    _id(motor_id),
    _param_list(params)
{
}

/**
 * @brief ISingleMotorCmd::getId
 * @return
 */
inline
uint8_t ISingleMotorCmd::getId() const
{
    return _id;
}

/**
 * @brief ISingleMotorCmd::getParam
 * @return
 */
inline
uint32_t ISingleMotorCmd::getParam() const
{
    return _param;
}

/**
 * @brief ISingleMotorCmd::getParam
 * @return
 */
inline
std::vector<int32_t> ISingleMotorCmd::getParams() const
{
    return _param_list;
}

/**
 * @brief ISingleMotorCmd::setId
 * @param id
 */
inline
void ISingleMotorCmd::setId(uint8_t id)
{
    _id = id;
}

/**
 * @brief ISingleMotorCmd::setParam
 * @param param
 */
inline
void ISingleMotorCmd::setParam(uint32_t param)
{
    _param = param;
}

/**
 * @brief ISingleMotorCmd::setParam
 * @param param
 */
inline
void ISingleMotorCmd::setParams(std::vector<int32_t> params) 
{
    _param_list = params;
}

/**
 * @brief ISingleMotorCmd::isCmdStepper
*/
inline bool ISingleMotorCmd::isCmdStepper() const
{
    return false;
}

/**
 * @brief ISingleMotorCmd::isCmdDxl
*/
inline bool ISingleMotorCmd::isCmdDxl() const
{
    return false;
}

/**
 * @brief ISingleMotorCmd::isValid
*/
inline bool ISingleMotorCmd::isValid() const
{
    return false;
}

/**
 * @brief ISingleMotorCmd::str
*/
inline std::string ISingleMotorCmd::str() const
{
    return "";
}

/**
 * @brief ISingleMotorCmd::getType()
*/
inline int ISingleMotorCmd::getTypeCmd() const
{
    return 0;
}

} // namespace model
} // namespace common

#endif
