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

#ifndef ABSTRACT_SINGLE_MOTOR_CMD_H
#define ABSTRACT_SINGLE_MOTOR_CMD_H

#include <string>
#include <vector>

#include "common/model/i_object.hpp"

namespace common
{
namespace model
{

class ISingleMotorCmd : public IObject
{
public:
    virtual int getCmdType() const = 0;

    //tests
    virtual bool isStepperCmd() const = 0;
    virtual bool isDxlCmd() const = 0;
};

/**
 * @brief The AbstractSingleMotorCmd class
 */
template<typename ParamType>
class AbstractSingleMotorCmd : public ISingleMotorCmd
{
    public:
        AbstractSingleMotorCmd(uint8_t id);
        virtual ~AbstractSingleMotorCmd() = 0;

        // setters
        void clear();

        void setId(uint8_t id);
        void setParam(ParamType param);
        void setParams(std::vector<ParamType> params);

        // getters
        uint8_t getId() const;
        ParamType getParam() const;
        std::vector<ParamType> getParams() const;

    protected:
        uint8_t _id;
        std::vector<ParamType> _param_list;

        AbstractSingleMotorCmd() = delete;
};

/**
 * @brief SingleMotorCmd::SingleMotorCmd
 * @param type
 */
template<typename ParamType>
AbstractSingleMotorCmd<ParamType>::AbstractSingleMotorCmd(uint8_t id) :
    _id(id)
{
    clear();
}

/**
 * @brief SingleMotorCmd::SingleMotorCmd
 * @param type
 */
template<typename ParamType>
AbstractSingleMotorCmd<ParamType>::~AbstractSingleMotorCmd()
{}

/**
 * @brief SingleMotorCmd::clear
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::clear()
{
    _id = 0;
    _param_list.clear();
}

// ***********************
//  AbstractSingleMotorCmd intf
// ***********************

/**
 * @brief SingleMotorCmd::setId
 * @param id
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::setId(uint8_t id)
{
    _id = id;
}

/**
 * @brief SingleMotorCmd::setParam
 * @param param
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::setParam(ParamType param)
{
    _param_list.clear();
    _param_list.emplace_back(param);
}

/**
 * @brief SingleMotorCmd::setParam
 * @param param
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::setParams(std::vector<ParamType> params)
{
    _param_list = params;
}

/**
 * @brief SingleMotorCmd::getId
 * @return
 */
template<typename ParamType>
uint8_t AbstractSingleMotorCmd<ParamType>::getId() const
{
    return _id;
}

/**
 * @brief SingleMotorCmd::getParam
 * @return
 */
template<typename ParamType>
ParamType AbstractSingleMotorCmd<ParamType>::getParam() const
{
    return _param_list.front();
}

/**
 * @brief SingleMotorCmd::getParams
 * @return
 */
template<typename ParamType>
std::vector<ParamType>
AbstractSingleMotorCmd<ParamType>::getParams() const
{
    return _param_list;
}

using AbstractTtlSingleMotorCmd = AbstractSingleMotorCmd<uint32_t>;
using AbstractCanSingleMotorCmd = AbstractSingleMotorCmd<int32_t>;

} // namespace model
} // namespace common

#endif // ABSTRACT_SINGLE_MOTOR_CMD_H
