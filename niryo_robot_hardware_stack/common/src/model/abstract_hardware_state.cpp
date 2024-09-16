/*
    abstract_hardware_state.cpp
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

#include "common/model/abstract_hardware_state.hpp"

#include <sstream>
#include <string>
#include <utility>

using ::std::ostringstream;
using ::std::string;

namespace common
{
namespace model
{

/**
 * @brief AbstractHardwareState::AbstractHardwareState
 * @param type
 * @param component_type
 * @param bus_proto
 * @param id
 */
AbstractHardwareState::AbstractHardwareState(EHardwareType type, EComponentType component_type, EBusProtocol bus_proto, uint8_t id)
    : _hw_type(type), _component_type(component_type), _bus_proto(bus_proto), _id(id)
{
}

/**
 * @brief AbstractHardwareState::reset
 */
void AbstractHardwareState::reset()
{
    _id = 0;
    _temperature = 0;
    _voltage = 0.0;
    _hw_error = 0;
    _hw_error_message.clear();
}

/**
 * @brief AbstractHardwareState::operator ==
 * @param other
 * @return
 */
bool AbstractHardwareState::operator==(const AbstractHardwareState &other) { return (this->_id == other._id); }

/**
 * @brief AbstractHardwareState::str
 * @return
 */
string AbstractHardwareState::str() const
{
    ostringstream ss;

    ss << "AbstractHardwareState :\n"
       << "id: " << static_cast<int>(_id) << "\n";

    ss << "Hardware Type " << HardwareTypeEnum(_hw_type).toString() << "\n"
       << "Component Type " << ComponentTypeEnum(_component_type).toString() << "\n"
       << "Bus Protocol " << BusProtocolEnum(_bus_proto).toString() << "\n";

    ss << "temperature " << static_cast<int>(_temperature) << "\n"
       << "voltage " << _voltage << "\n"
       << "hw_error " << _hw_error << "\n"
       << "hw_error_message \"" << _hw_error_message << "\"";
    ss << "\n---\n";
    ss << "\n";

    return ss.str();
}

/**
 * @brief StepperMotorState::setFirmwareVersion
 * @param firmware_version
 */
void AbstractHardwareState::setFirmwareVersion(const std::string &firmware_version) { _firmware_version = firmware_version; }

/**
 * @brief AbstractHardwareState::setTemperature
 * @param temp
 */
void AbstractHardwareState::setTemperature(uint8_t temp) { _temperature = temp; }

/**
 * @brief AbstractHardwareState::setRawVoltage
 * @param raw_volt
 * TODO(CC) avoid using hardcoded values, only usable for Ned2 for now
 */
void AbstractHardwareState::setRawVoltage(double raw_volt)
{
    if (EHardwareType::STEPPER == _hw_type || EHardwareType::FAKE_STEPPER_MOTOR == _hw_type || EHardwareType::END_EFFECTOR == _hw_type ||
        EHardwareType::FAKE_END_EFFECTOR == _hw_type || EHardwareType::NED3PRO_STEPPER == _hw_type || EHardwareType:: NED3PRO_END_EFFECTOR == _hw_type)
        _voltage = raw_volt / 1000;
    else
        _voltage = raw_volt / 10;
}

/**
 * @brief AbstractHardwareState::setVoltage
 * @param volt
 */
void AbstractHardwareState::setVoltage(double volt) { _voltage = volt; }

/**
 * @brief AbstractHardwareState::setHardwareError
 * @param hw_error
 */
void AbstractHardwareState::setHardwareError(uint32_t hw_error) { _hw_error = hw_error; }

/**
 * @brief AbstractHardwareState::setConnectionStatus
 * @param connected
 */
void AbstractHardwareState::setConnectionStatus(bool connected)
{
    if (connected)
        _hw_error |= (1UL << 7);
    else
        _hw_error &= ~(1UL << 7);
}

/**
 * @brief AbstractHardwareState::setHardwareError
 * @param hw_error_msg
 */
void AbstractHardwareState::setHardwareError(std::string hw_error_msg) { _hw_error_message = std::move(hw_error_msg); }

void AbstractHardwareState::setStrictModelNumber(bool strict_model_number)
{
    _strict_model_number = strict_model_number;
}

}  // namespace model
}  // namespace common
