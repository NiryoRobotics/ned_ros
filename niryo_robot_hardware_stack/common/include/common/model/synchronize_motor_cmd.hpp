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
#include "common/model/isynchronize_motor_cmd.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SynchronizeMotorCmd class
 */
template<typename E> 
class SynchronizeMotorCmd : public AbstractMotorCmd<E>, public ISynchronizeMotorCmd
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
        SynchronizeMotorCmd(E type);

        // AbstractMotorCmd interface
        void clear() override;

        // ISynchronizeMotor interface

        // setters
        void addMotorParam(EMotorType type, uint8_t id, uint32_t param) override;

        // getters
        std::vector<uint8_t> getMotorsId(EMotorType type) const override;
        std::vector<uint32_t> getParams(EMotorType type) const override;
        std::set<EMotorType> getMotorTypes() const override;

        int getTypeCmd() const override;
        bool isCmdStepper() const override;
        bool isCmdDxl() const override;

        // IObject interface
        virtual void reset() override;
        virtual std::string str() const override;
        virtual bool isValid() const override;

    private:
        std::set<EMotorType> _types;
        std::map<EMotorType, MotorParam > _motor_params_map;
};

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 */
template<typename E>
SynchronizeMotorCmd<E>::SynchronizeMotorCmd() :
    AbstractMotorCmd<E>(E::CMD_TYPE_UNKNOWN)

{
    reset();
}

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 * @param type
 */
template<typename E>
SynchronizeMotorCmd<E>::SynchronizeMotorCmd(E type) :
    AbstractMotorCmd<E>(type)
{
}

// ***********************
//  ISynchronizeMotorCmd intf
// ***********************

/**
 * @brief SynchronizeMotorCmd::addMotorParam
 * @param type
 * @param id
 * @param param
 */
template<typename E>
void SynchronizeMotorCmd<E>::addMotorParam(EMotorType type, uint8_t id, uint32_t param)
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

/**
 * @brief SynchronizeMotorCmd::getMotorsId
 * @param type
 * @return
 */
template<typename E>
std::vector<uint8_t> 
SynchronizeMotorCmd<E>::getMotorsId(EMotorType type) const
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
template<typename E>
std::vector<uint32_t> 
SynchronizeMotorCmd<E>::getParams(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).params;
}

/**
 * @brief SynchronizeMotorCmd::getMotorTypes
 * @return
 */
template<typename E>
std::set<EMotorType> 
SynchronizeMotorCmd<E>::getMotorTypes() const
{
    return _types;
}

/**
 * @brief SingleMotorCmd::getTypeCmd
 * @return
 */
template<typename E>
int SynchronizeMotorCmd<E>::getTypeCmd() const
{
    return (int)this->getType();
}

/**
 * @brief SynchronizeMotorCmd::isValid
 * @return
 */
template<typename E>
bool SynchronizeMotorCmd<E>::isValid() const
{
    if (E::CMD_TYPE_UNKNOWN == this->getType() || _motor_params_map.empty())
    {
        return false;
    }

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
template<typename E>
bool SynchronizeMotorCmd<E>::isCmdStepper() const
{
    return typeid(E) == typeid(common::model::EStepperCommandType);
}

/**
 * @brief SynchronizeMotorCmd::isCmdDxl
 * @param none
 * @return
 */
template<typename E>
bool SynchronizeMotorCmd<E>::isCmdDxl() const
{
    return typeid(E) == typeid(common::model::EDxlCommandType);
}

/**
 * @brief SynchronizeMotorCmd::reset
 */
template<typename E>
void SynchronizeMotorCmd<E>::reset()
{
    this->setType(E::CMD_TYPE_UNKNOWN);
    clear();
}

/**
 * @brief SynchronizeMotorCmd::clear : clears the data (keep the cmd type)
 */
template<typename E>
void SynchronizeMotorCmd<E>::clear()
{
    _motor_params_map.clear();
    _types.clear();
}

//********************************
// specializations for dynamixel
//********************************

template<>
inline
std::string SynchronizeMotorCmd<common::model::EDxlCommandType>::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Sync motor cmd - ";
    ss << DxlCommandTypeEnum(this->getType()).toString();
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


//********************************
// specializations for steppers
//********************************

template<>
inline
std::string SynchronizeMotorCmd<common::model::EStepperCommandType>::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Sync motor cmd - ";
    ss << StepperCommandTypeEnum(this->getType()).toString();
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


using DxlSyncCmd = SynchronizeMotorCmd<EDxlCommandType>;
using StepperSyncCmd = SynchronizeMotorCmd<EStepperCommandType>;

} // namespace model
} // namespace common

#endif
