/*
synchronize_motor_cmd.hpp
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

#ifndef DXL_SYNCHRONIZE_MOTOR_CMD_H
#define DXL_SYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <sstream>
#include <typeinfo>

#include "common/model/abstract_motor_cmd.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/motor_type_enum.hpp"
#include "common/model/joint_state.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SynchronizeMotorCmd class
 */
template<typename T, typename TE> 
class SynchronizeMotorCmd : public AbstractMotorCmd<T>
{
    struct MotorParam {
        MotorParam(uint8_t id, uint32_t param) {
            motors_id.emplace_back(id);
            params.emplace_back(param);
        }

        bool isValid() const {
            return !motors_id.empty() && motors_id.size() == params.size();
        }

        std::vector<uint8_t> motors_id;
        std::vector<uint32_t> params;
    };

    public:
        SynchronizeMotorCmd();
        SynchronizeMotorCmd(T type);

        // setters
        void addMotorParam(EMotorType type, uint8_t id, uint32_t param);

        // getters
        std::vector<uint8_t> getMotorsId(EMotorType type) const;
        std::vector<uint32_t> getParams(EMotorType type) const;
        std::set<EMotorType> getMotorTypes() const;

        // AbstractMotorCmd interface
        bool isCmdStepper() const override;
        bool isCmdDxl() const override;
        void reset() override;
        std::string str() const override;
        void clear() override;
        bool isValid() const override;

    private:
        std::set<EMotorType> _types;
        std::map<EMotorType, MotorParam > _motor_params_map;
};

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 */
template<typename T, typename TE>
SynchronizeMotorCmd<T, TE>::SynchronizeMotorCmd() :
    AbstractMotorCmd<T>(T::CMD_TYPE_UNKNOWN)

{
    reset();
}

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 * @param type
 */
template<typename T, typename TE>
SynchronizeMotorCmd<T, TE>::SynchronizeMotorCmd(T type) :
    AbstractMotorCmd<T>(type)
{
}

/**
 * @brief SynchronizeMotorCmd::addMotorParam
 * @param type
 * @param id
 * @param param
 */
template<typename T, typename TE>
void SynchronizeMotorCmd<T, TE>::addMotorParam(EMotorType type, uint8_t id, uint32_t param)
{
    // not yet in map
    if (!_motor_params_map.count(type))
    {
        _motor_params_map.insert(std::make_pair(type, MotorParam(id, param)));
        _types.insert(type);
    }
    else
    {
        _motor_params_map.at(type).motors_id.emplace_back(id);
        _motor_params_map.at(type).params.emplace_back(param);
    }
}

// ***********************
//  Getters
// ***********************

/**
 * @brief SynchronizeMotorCmd::getMotorsId
 * @param type
 * @return
 */
template<typename T, typename TE>
std::vector<uint8_t> SynchronizeMotorCmd<T, TE>::getMotorsId(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).motors_id;
}

/**
 * @brief SynchronizeMotorCmd::getParams
 * @param type
 * @return
 */
template<typename T, typename TE>
std::vector<uint32_t> SynchronizeMotorCmd<T, TE>::getParams(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).params;
}

/**
 * @brief SynchronizeMotorCmd::getMotorTypes
 * @return
 */
template<typename T, typename TE>
std::set<EMotorType> SynchronizeMotorCmd<T, TE>::getMotorTypes() const
{
    return _types;
}

// ***********************
//  AbstractMotorCmd intf
// ***********************

/**
 * @brief SynchronizeMotorCmd::reset
 */
template<typename T, typename TE>
void SynchronizeMotorCmd<T, TE>::reset()
{
    this->setType(T::CMD_TYPE_UNKNOWN);
    clear();
}

/**
 * @brief SynchronizeMotorCmd::str
 * @return
 */
template<typename T, typename TE>
std::string SynchronizeMotorCmd<T, TE>::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Sync motor cmd - ";
    ss << TE(this->getType()).toString();
    ss << ": ";

    if (!isValid())
    {
        ss << "Corrupted command : motors id list and params list size mismatch ";
        string_info = ss.str();
    }
    else
    {
        ss << "[";

        for (auto const& param : _motor_params_map)
        {
            ss << MotorTypeEnum(param.first).toString() << " => ";
            MotorParam p = param.second;
            for (size_t i = 0; i < p.motors_id.size() && i < p.params.size(); ++i)
                ss << "(" << static_cast<int>(p.motors_id.at(i)) << ", " << p.params.at(i) << ")" << ",";
        }

        string_info = ss.str();
        string_info.pop_back();

        string_info += "]";
    }

    return string_info;
}

/**
 * @brief SynchronizeMotorCmd::isValid
 * @return
 */
template<typename T, typename TE>
bool SynchronizeMotorCmd<T, TE>::isValid() const
{
    if (T::CMD_TYPE_UNKNOWN == this->getType() || _motor_params_map.empty())
        return false;

    for (auto const& it_map : _motor_params_map)
    {
        if (!it_map.second.isValid())
            return false;
    }

    return true;
}

/**
 * @brief SynchronizeMotorCmd::isCmdStepper
 * @param none
 * @return
 */
template<typename T, typename TE>
bool SynchronizeMotorCmd<T, TE>::isCmdStepper() const
{
    return typeid(T) == typeid(common::model::StepperCommandTypeEnum);
}

/**
 * @brief SynchronizeMotorCmd::isCmdDxl
 * @param none
 * @return
 */
template<typename T, typename TE>
bool SynchronizeMotorCmd<T, TE>::isCmdDxl() const
{
    return typeid(T) == typeid(common::model::DxlCommandTypeEnum);
}

/**
 * @brief SynchronizeMotorCmd::clear : clears the data (keep the cmd type)
 */
template<typename T, typename TE>
void SynchronizeMotorCmd<T, TE>::clear()
{
    _motor_params_map.clear();
    _types.clear();
}

} // namespace model
} // namespace common

#endif
