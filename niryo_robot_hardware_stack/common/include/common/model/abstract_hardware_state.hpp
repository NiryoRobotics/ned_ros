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

        AbstractHardwareState();
        AbstractHardwareState(EHardwareType type, EBusProtocol bus_proto, uint8_t id);
        virtual ~AbstractHardwareState() override;

        // getters
        EHardwareType getType() const;
        EBusProtocol getBusProtocol() const;
        uint8_t getId() const;

        int getTemperatureState() const;
        int getVoltageState() const;
        int getHardwareErrorState() const;
        std::string getHardwareErrorMessageState() const;

        // setters
        void setTemperatureState(int temp);
        void setVoltageState(int volt);
        void setHardwareError(int hw_error);
        void setHardwareError(std::string hw_error_msg);

        // operators
        virtual bool operator==(const AbstractHardwareState& other);

        // IObject interface
        virtual void reset() override;
        virtual std::string str() const override;
        virtual bool isValid() const override = 0; // not reimplemented to keep this class abstract

    protected:
        EHardwareType _type;
        EBusProtocol _bus_proto;

        uint8_t _id;

        // read variables
        int _temperature_state;
        int _voltage_state;
        int _hw_error_state;
        std::string _hw_error_message_state;
};

/**
 * @brief AbstractHardwareState::getType
 * @return
 */
inline
EHardwareType AbstractHardwareState::getType() const
{
    return _type;
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
 * @brief AbstractHardwareState::getTemperatureState
 * @return
 */
inline
int AbstractHardwareState::getTemperatureState() const
{
    return _temperature_state;
}

/**
 * @brief AbstractHardwareState::getVoltageState
 * @return
 */
inline
int AbstractHardwareState::getVoltageState() const
{
    return _voltage_state;
}

/**
 * @brief AbstractHardwareState::getHardwareErrorState
 * @return
 */
inline
int AbstractHardwareState::getHardwareErrorState() const
{
    return _hw_error_state;
}

/**
 * @brief AbstractHardwareState::getHardwareErrorMessageState
 * @return
 */
inline
std::string AbstractHardwareState::getHardwareErrorMessageState() const
{
    return _hw_error_message_state;
}

} // model
} // common

#endif // ABSTRACT_HARDWARE_STATE_H
