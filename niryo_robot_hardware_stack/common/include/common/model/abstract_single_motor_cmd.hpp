/*
abstract_single_motor_cmd.hpp
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

#include "common/model/i_single_motor_cmd.hpp"

namespace common
{
namespace model
{

/**
 * @brief The AbstractSingleMotorCmd class
 */
template<typename ParamType>
class AbstractSingleMotorCmd : public ISingleMotorCmd
{
public:
    AbstractSingleMotorCmd() = delete;
    AbstractSingleMotorCmd(uint8_t id);

    ~AbstractSingleMotorCmd() override = default;

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
    std::vector<ParamType> _param_list;
    uint8_t _id{};

protected:
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    AbstractSingleMotorCmd( const AbstractSingleMotorCmd& ) = default;
    AbstractSingleMotorCmd( AbstractSingleMotorCmd&& ) noexcept = default;

    AbstractSingleMotorCmd& operator= ( AbstractSingleMotorCmd && ) noexcept = default;
    AbstractSingleMotorCmd& operator= ( const AbstractSingleMotorCmd& ) = default;
};


/**
 * @brief AbstractSingleMotorCmd<ParamType>::AbstractSingleMotorCmd
 * @param id
 */
template<typename ParamType>
AbstractSingleMotorCmd<ParamType>::AbstractSingleMotorCmd(uint8_t id) :
    _id(id)
{}

/**
 * @brief AbstractSingleMotorCmd<ParamType>::clear
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
 * @brief AbstractSingleMotorCmd<ParamType>::setId
 * @param id
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::setId(uint8_t id)
{
    _id = id;
}

/**
 * @brief AbstractSingleMotorCmd<ParamType>::setParam
 * @param param
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::setParam(ParamType param)
{
    _param_list.clear();
    _param_list.emplace_back(param);
}

/**
 * @brief AbstractSingleMotorCmd<ParamType>::setParams
 * @param params
 */
template<typename ParamType>
void AbstractSingleMotorCmd<ParamType>::setParams(std::vector<ParamType> params)
{
    _param_list = params;
}

/**
 * @brief AbstractSingleMotorCmd<ParamType>::getId
 * @return
 */
template<typename ParamType>
uint8_t AbstractSingleMotorCmd<ParamType>::getId() const
{
    return _id;
}

/**
 * @brief AbstractSingleMotorCmd<ParamType>::getParam
 * @return
 */
template<typename ParamType>
ParamType AbstractSingleMotorCmd<ParamType>::getParam() const
{
    return _param_list.front();
}

/**
 * @brief AbstractSingleMotorCmd<ParamType>::getParams
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
