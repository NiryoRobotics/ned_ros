/*
abstract_hardware_state.h
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

#ifndef ABSTRACT_HARDWARE_STATE_H
#define ABSTRACT_HARDWARE_STATE_H

#include "i_object.hpp"
#include <string>

#include "common/model/hardware_type_enum.hpp"
#include "common/model/component_type_enum.hpp"
#include "common/model/bus_protocol_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The AbstractHardwareState class
 */
class AbstractHardwareState : public IObject
{
public:
    AbstractHardwareState() = default;
    AbstractHardwareState(EHardwareType type,
                          EComponentType component_type,
                          EBusProtocol bus_proto,
                          uint8_t id);

    ~AbstractHardwareState() override = default;

    // getters
    EHardwareType getHardwareType() const;
    EComponentType getComponentType() const;
    EBusProtocol getBusProtocol() const;

    uint8_t getId() const;

    std::string getFirmwareVersion() const;
    uint8_t getTemperature() const;
    double getVoltage() const;
    uint32_t getHardwareError() const;
    std::string getHardwareErrorMessage() const;
    bool getStrictModelNumber() const;

    // setters
    void setFirmwareVersion(const std::string &firmware_version);
    void setTemperature(uint8_t temp);
    void setVoltage(double volt);
    void setRawVoltage(double raw_volt);
    void setHardwareError(uint32_t hw_error);
    void setHardwareError(std::string hw_error_msg);
    void setConnectionStatus(bool connected);
    void setStrictModelNumber(bool strict_model_number);

    // operators
    virtual bool operator==(const AbstractHardwareState& other);

    // IObject interface
    void reset() override;
    std::string str() const override;
    bool isValid() const override = 0; // not reimplemented to keep this class abstract


protected:
    EHardwareType _hw_type{EHardwareType::UNKNOWN};
    EComponentType _component_type{EComponentType::UNKNOWN};
    EBusProtocol _bus_proto{EBusProtocol::UNKNOWN};

    // read variables
    std::string _firmware_version{};

    uint8_t _temperature{0};
    double _voltage{0.0};
    uint32_t _hw_error{0};
    std::string _hw_error_message{};

    uint8_t _id{0};
    bool _strict_model_number{false};

protected:
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    AbstractHardwareState( const AbstractHardwareState& ) = delete;
    AbstractHardwareState( AbstractHardwareState&& ) = delete;

    AbstractHardwareState& operator= ( AbstractHardwareState && ) = delete;
    AbstractHardwareState& operator= ( const AbstractHardwareState& ) = delete;
};

/**
 * @brief AbstractHardwareState::getHardwareType
 * @return
 */
inline
EHardwareType AbstractHardwareState::getHardwareType() const
{
    return _hw_type;
}

/**
 * @brief AbstractHardwareState::getComponentType
 * @return
 */
inline
EComponentType AbstractHardwareState::getComponentType() const
{
  return _component_type;
}

/**
 * @brief AbstractHardwareState::getBusProtocol
 * @return
 */
inline
EBusProtocol AbstractHardwareState::getBusProtocol() const
{
    return _bus_proto;
}

/**
 * @brief AbstractHardwareState::getId
 * @return
 */
inline
uint8_t AbstractHardwareState::getId() const
{
    return _id;
}


/**
 * @brief StepperMotorState::getFirmwareVersion
 * @return
 */
inline
std::string AbstractHardwareState::getFirmwareVersion() const
{
    return _firmware_version;
}

/**
 * @brief AbstractHardwareState::getTemperatureState
 * @return
 */
inline
uint8_t AbstractHardwareState::getTemperature() const
{
    return _temperature;
}

/**
 * @brief AbstractHardwareState::getVoltageState
 * @return
 */
inline
double AbstractHardwareState::getVoltage() const
{
    return _voltage;
}

/**
 * @brief AbstractHardwareState::getHardwareErrorState
 * @return
 */
inline
uint32_t AbstractHardwareState::getHardwareError() const
{
    return _hw_error;
}

/**
 * @brief AbstractHardwareState::getHardwareErrorMessageState
 * @return
 */
inline
std::string AbstractHardwareState::getHardwareErrorMessage() const
{
    return _hw_error_message;
}

inline
bool AbstractHardwareState::getStrictModelNumber() const
{
    return _strict_model_number;
}


} // model
} // common

#endif // ABSTRACT_HARDWARE_STATE_H
