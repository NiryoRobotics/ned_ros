/*
    xdriver.hpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MOTORDRIVER_HPP
#define MOTORDRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "xdriver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common_defs.hpp"
#include "model/motor_type_enum.hpp"


namespace DynamixelDriver
{
    class XC430Reg;
    class XL430Reg;
    class XL330Reg;
    class XL320Reg;

    /**
     * @brief The TTLMotorDriver class
     */
    template<typename reg_type>
    class MotorDriver : public XDriver
    {

    public:
        MotorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                       std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        ~MotorDriver() override;

        //specifics to reg type
        std::string str() const override;
        std::string interpreteErrorState(uint32_t hw_state) override;
        int checkModelNumber(uint8_t id) override;

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id ) override
        {
            return write(reg_type::ADDR_ID, reg_type::SIZE_ID, id, new_id);
        }

        int changeBaudRate(uint8_t id, uint32_t new_baudrate ) override
        {
            return write(reg_type::ADDR_BAUDRATE, reg_type::SIZE_BAUDRATE, id, new_baudrate);
        }

        int setReturnDelayTime(uint8_t id, uint32_t return_delay_time ) override
        {
            return write(reg_type::ADDR_RETURN_DELAY_TIME, reg_type::SIZE_RETURN_DELAY_TIME, id, return_delay_time);
        }

        int setLimitTemperature(uint8_t id, uint32_t temperature ) override
        {
            return write(reg_type::ADDR_TEMPERATURE_LIMIT, reg_type::SIZE_TEMPERATURE_LIMIT, id, temperature);
        }
        int setMaxTorque(uint8_t id, uint32_t torque ) override
        {
            return write(reg_type::ADDR_MAX_TORQUE, reg_type::SIZE_MAX_TORQUE, id, torque);
        }
        int setReturnLevel(uint8_t id, uint32_t return_level ) override
        {
            return write(reg_type::ADDR_RETURN_LEVEL, reg_type::SIZE_RETURN_LEVEL, id, return_level);
        }
        int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown ) override
        {
            return write(reg_type::ADDR_ALARM_SHUTDOWN, reg_type::SIZE_ALARM_SHUTDOWN, id, alarm_shutdown);
        }

        // eeprom read
        int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time ) override
        {
            return read(reg_type::ADDR_RETURN_DELAY_TIME, reg_type::SIZE_RETURN_DELAY_TIME, id, return_delay_time);
        }
        int readLimitTemperature(uint8_t id, uint32_t *limit_temperature ) override
        {
            return read(reg_type::ADDR_TEMPERATURE_LIMIT, reg_type::SIZE_TEMPERATURE_LIMIT, id, limit_temperature);
        }
        int readMaxTorque(uint8_t id, uint32_t *max_torque ) override
        {
            return read(reg_type::ADDR_MAX_TORQUE, reg_type::SIZE_MAX_TORQUE, id, max_torque);
        }
        int readReturnLevel(uint8_t id, uint32_t *return_level ) override
        {
            return read(reg_type::ADDR_RETURN_LEVEL, reg_type::SIZE_RETURN_LEVEL, id, return_level);
        }
        int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown ) override
        {
            return read(reg_type::ADDR_ALARM_SHUTDOWN, reg_type::SIZE_ALARM_SHUTDOWN, id, alarm_shutdown);
        }

        // ram write
        int setTorqueEnable(uint8_t id, uint32_t torque_enable ) override
        {
            return write(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id, torque_enable);
        }
        int setLed(uint8_t id, uint32_t led_value ) override
        {
            return write(reg_type::ADDR_LED, reg_type::SIZE_LED, id, led_value);
        }
        int setGoalPosition(uint8_t id, uint32_t position ) override
        {
            return write(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id, position);
        }
        int setGoalVelocity(uint8_t id, uint32_t velocity ) override
        {
            return write(reg_type::ADDR_GOAL_SPEED, reg_type::SIZE_GOAL_SPEED, id, velocity);
        }

        int setGoalTorque(uint8_t id, uint32_t torque ) override
        {
            return write(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id, torque);
        }

        int setPGain(uint8_t id, uint32_t gain ) override
        {
            return write(reg_type::ADDR_POSITION_P_GAIN, reg_type::SIZE_POSITION_P_GAIN, id, gain);
        }
        int setIGain(uint8_t id, uint32_t gain ) override
        {
            return write(reg_type::ADDR_POSITION_I_GAIN, reg_type::SIZE_POSITION_I_GAIN, id, gain);
        }
        int setDGain(uint8_t id, uint32_t gain ) override
        {
            return write(reg_type::ADDR_POSITION_D_GAIN, reg_type::SIZE_POSITION_D_GAIN, id, gain);
        }
        int setff1Gain(uint8_t id, uint32_t gain ) override
        {
            return write(reg_type::ADDR_FF1_GAIN, reg_type::SIZE_FF1_GAIN, id, gain);
        }
        int setff2Gain(uint8_t id, uint32_t gain ) override
        {
            return write(reg_type::ADDR_FF2_GAIN, reg_type::SIZE_FF2_GAIN, id, gain);
        }

        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list ) override
        {
            return syncWrite(reg_type::ADDR_LED, reg_type::SIZE_LED, id_list, led_list);
        }
        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list ) override
        {
            return syncWrite(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id_list, torque_enable_list);
        }
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list ) override
        {
            return syncWrite(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id_list, position_list);
        }
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list ) override
        {
            return syncWrite(reg_type::ADDR_GOAL_SPEED, reg_type::SIZE_GOAL_SPEED, id_list, velocity_list);
        }
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list ) override
        {
            return syncWrite(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id_list, torque_list);
        }

        // ram read
        int readPosition(uint8_t id, uint32_t *present_position ) override
        {
            return read(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id, present_position);
        }
        int readVelocity(uint8_t id, uint32_t *present_velocity ) override
        {
            return read(reg_type::ADDR_PRESENT_SPEED, reg_type::SIZE_PRESENT_SPEED, id, present_velocity);
        }
        int readLoad(uint8_t id, uint32_t *present_load ) override
        {
            return read(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id, present_load);
        }

        int readTemperature(uint8_t id, uint32_t *temperature ) override
        {
            return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
        }
        int readVoltage(uint8_t id, uint32_t *voltage ) override
        {
            return read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage);
        }
        int readHardwareStatus(uint8_t id, uint32_t *hardware_status ) override
        {
            return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
        }

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list ) override
        {
            return syncRead(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id_list, position_list);
        }
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list ) override
        {
            return syncRead(reg_type::ADDR_PRESENT_SPEED, reg_type::SIZE_PRESENT_SPEED, id_list, velocity_list);
        }
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list ) override
        {
            return syncRead(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id_list, load_list);
        }
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list ) override
        {
            return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
        }
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list ) override
        {
            return syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, voltage_list);
        }
        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list ) override
        {
            return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
        }

        // XDriver interface
    public:
    };

    // definition of methods

    /**
     * @brief TTLMotorDriver<reg_type>::TTLMotorDriver
     * @param type
     * @param portHandler
     * @param packetHandler
     */
    template<typename reg_type>
    MotorDriver<reg_type>::MotorDriver(
                     std::shared_ptr<dynamixel::PortHandler> portHandler,
                     std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
        XDriver(reg_type::motor_type,
                portHandler,
                packetHandler)
    {
    }

    /**
     * @brief TTLMotorDriver<reg_type>::~TTLMotorDriver
     */
    template<typename reg_type>
    MotorDriver<reg_type>::~MotorDriver()
    {

    }

    template<typename reg_type>
    std::string MotorDriver<reg_type>::str() const
    {
        std::ostringstream ss;

        ss << "\n"
           << "Driver - type:"
           << common::model::MotorTypeEnum(reg_type::motor_type).toString();

        return ss.str() + XDriver::str();
    }

    template<typename reg_type>
    std::string MotorDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/)
    {
        return "";
    }

    template<typename reg_type>
    int MotorDriver<reg_type>::checkModelNumber(uint8_t id)
    {
        uint16_t model_number;
        int ping_result = getModelNumber(id, &model_number);

        if (ping_result == COMM_SUCCESS)
        {
            if (model_number && model_number != reg_type::MODEL_NUMBER)
            {
                return PING_WRONG_MODEL_NUMBER;
            }
        }

        return ping_result;
    }

    /*
     *  -----------------   Read Write operations   --------------------
     */

    // define specializations

    //XL320

    template<>
    std::string MotorDriver<XL320Reg>::interpreteErrorState(uint32_t hw_state)
    {
        std::string hardware_message;

        if (hw_state & 1<<0)    //0b00000001
        {
            hardware_message += "Overload";
        }
        if (hw_state & 1<<1)    //0b00000010
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1<<2)    //0b00000100
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Input voltage out of range";
        }

        return hardware_message;
    }

    template<>
    int MotorDriver<XL320Reg>::setff1Gain(uint8_t /*id*/, uint32_t /*gain*/)
    {
        std::cout << "setff1Gain not available for motor XL320" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XL320Reg>::setff2Gain(uint8_t /*id*/, uint32_t /*gain*/)
    {
        std::cout << "setff2Gain not available for motor XL320" << std::endl;
        return COMM_TX_ERROR;
    }


    //XL430

    template<>
    std::string MotorDriver<XL430Reg>::interpreteErrorState(uint32_t hw_state)
    {
        std::string hardware_message;

        if (hw_state & 1<<0)    //0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1<<2)    //0b00000100
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1<<3)    //0b00001000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1<<4)    //0b00010000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1<<5)    //0b00100000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hardware_message != "")
            hardware_message += " Error";

        return hardware_message;
    }


    template<>
    int MotorDriver<XL430Reg>::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setGoalTorque not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XL430Reg>::setMaxTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setMaxTorque not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XL430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &/*id_list*/, const std::vector<uint32_t> &/*torque_list*/)
    {
        std::cout << "syncWriteTorqueGoal not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XL430Reg>::readMaxTorque(uint8_t /*id*/, uint32_t* /*max_torque*/)
    {
        std::cout << "readMaxTorque not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    //XC430

    template<>
    std::string MotorDriver<XC430Reg>::interpreteErrorState(uint32_t hw_state)
    {
        std::string hardware_message;

        if (hw_state & 1<<0)    //0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1<<2)    //0b00000100
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1<<3)    //0b00001000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1<<4)    //0b00010000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1<<5)    //0b00100000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hardware_message != "")
            hardware_message += " Error";

        return hardware_message;
    }

    template<>
    int MotorDriver<XC430Reg>::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setGoalTorque not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XC430Reg>::setMaxTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setMaxTorque not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XC430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &/*id_list*/, const std::vector<uint32_t> &/*torque_list*/)
    {
        std::cout << "syncWriteTorqueGoal not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int MotorDriver<XC430Reg>::readMaxTorque(uint8_t /*id*/, uint32_t* /*max_torque*/)
    {
        std::cout << "readMaxTorque not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    //XL330

    template<>
    std::string MotorDriver<XL330Reg>::interpreteErrorState(uint32_t hw_state)
    {
        std::string hardware_message;

        if (hw_state & 1<<0)    //0b00000001
        {
            hardware_message += "Input Voltage";
        }
        if (hw_state & 1<<2)    //0b00000100
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 1<<3)    //0b00001000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Motor Encoder";
        }
        if (hw_state & 1<<4)    //0b00010000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Electrical Shock";
        }
        if (hw_state & 1<<5)    //0b00100000
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "Overload";
        }
        if (hardware_message != "")
            hardware_message += " Error";

        return hardware_message;
    }

} //DynamixelDriver

#endif
