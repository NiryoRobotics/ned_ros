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

#ifndef XDRIVER_HPP
#define XDRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "abstract_motor_driver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common_defs.hpp"
#include "model/motor_type_enum.hpp"

#include "stepper_reg.hpp"
#include "xc430_reg.hpp"
#include "xl430_reg.hpp"
#include "xl330_reg.hpp"
#include "xl320_reg.hpp"

namespace TtlDriver
{

    /**
     * @brief The XDriver class
     */
    template<typename reg_type>
    class XDriver : public AbstractMotorDriver
    {

    public:
        XDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                       std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        ~XDriver() override;

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
            return write(reg_type::ADDR_STATUS_RETURN_LEVEL, reg_type::SIZE_STATUS_RETURN_LEVEL, id, return_level);
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
            return read(reg_type::ADDR_STATUS_RETURN_LEVEL, reg_type::SIZE_STATUS_RETURN_LEVEL, id, return_level);
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
            return write(reg_type::ADDR_GOAL_VELOCITY, reg_type::SIZE_GOAL_VELOCITY, id, velocity);
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
            return syncWrite(reg_type::ADDR_GOAL_VELOCITY, reg_type::SIZE_GOAL_VELOCITY, id_list, velocity_list);
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
            return read(reg_type::ADDR_PRESENT_VELOCITY, reg_type::SIZE_PRESENT_VELOCITY, id, present_velocity);
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
            return syncRead(reg_type::ADDR_PRESENT_VELOCITY, reg_type::SIZE_PRESENT_VELOCITY, id_list, velocity_list);
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
     * @brief XDriver<reg_type>::XDriver
     * @param type
     * @param portHandler
     * @param packetHandler
     */
    template<typename reg_type>
    XDriver<reg_type>::XDriver(
                     std::shared_ptr<dynamixel::PortHandler> portHandler,
                     std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
        AbstractMotorDriver(portHandler,
                packetHandler)
    {
    }

    /**
     * @brief XDriver<reg_type>::~XDriver
     */
    template<typename reg_type>
    XDriver<reg_type>::~XDriver()
    {

    }

    template<typename reg_type>
    std::string XDriver<reg_type>::str() const
    {
        std::ostringstream ss;

        ss << "\n"
           << "Driver - type:"
           << common::model::MotorTypeEnum(reg_type::motor_type).toString();

        return ss.str() + AbstractMotorDriver::str();
    }

    template<typename reg_type>
    std::string XDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/)
    {
        return "";
    }

    template<typename reg_type>
    int XDriver<reg_type>::checkModelNumber(uint8_t id)
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
    std::string XDriver<XL320Reg>::interpreteErrorState(uint32_t hw_state)
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
    int XDriver<XL320Reg>::setff1Gain(uint8_t /*id*/, uint32_t /*gain*/)
    {
        std::cout << "setff1Gain not available for motor XL320" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XL320Reg>::setff2Gain(uint8_t /*id*/, uint32_t /*gain*/)
    {
        std::cout << "setff2Gain not available for motor XL320" << std::endl;
        return COMM_TX_ERROR;
    }


    //XL430

    template<>
    std::string XDriver<XL430Reg>::interpreteErrorState(uint32_t hw_state)
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
    int XDriver<XL430Reg>::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setGoalTorque not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XL430Reg>::setMaxTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setMaxTorque not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XL430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &/*id_list*/, const std::vector<uint32_t> &/*torque_list*/)
    {
        std::cout << "syncWriteTorqueGoal not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XL430Reg>::readMaxTorque(uint8_t /*id*/, uint32_t* /*max_torque*/)
    {
        std::cout << "readMaxTorque not available for motor XL430" << std::endl;
        return COMM_TX_ERROR;
    }

    //XC430

    template<>
    std::string XDriver<XC430Reg>::interpreteErrorState(uint32_t hw_state)
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
    int XDriver<XC430Reg>::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setGoalTorque not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XC430Reg>::setMaxTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        std::cout << "setMaxTorque not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XC430Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &/*id_list*/, const std::vector<uint32_t> &/*torque_list*/)
    {
        std::cout << "syncWriteTorqueGoal not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    template<>
    int XDriver<XC430Reg>::readMaxTorque(uint8_t /*id*/, uint32_t* /*max_torque*/)
    {
        std::cout << "readMaxTorque not available for motor XC430" << std::endl;
        return COMM_TX_ERROR;
    }

    //XL330

    template<>
    std::string XDriver<XL330Reg>::interpreteErrorState(uint32_t hw_state)
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

    //CL330 works with current instead of load

    template<>
    int XDriver<XL330Reg>::setGoalTorque(uint8_t id, uint32_t torque)
    {
        return write(XL330Reg::ADDR_GOAL_CURRENT, XL330Reg::SIZE_GOAL_CURRENT, id, torque);
    }

    template<>
    int XDriver<XL330Reg>::setMaxTorque(uint8_t id, uint32_t torque)
    {
        return write(XL330Reg::ADDR_CURRENT_LIMIT, XL330Reg::SIZE_CURRENT_LIMIT, id, torque);
    }

    template<>
    int XDriver<XL330Reg>::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
    {
        return syncWrite(XL330Reg::ADDR_GOAL_CURRENT, XL330Reg::SIZE_GOAL_CURRENT, id_list, torque_list);
    }

    template<>
    int XDriver<XL330Reg>::readMaxTorque(uint8_t id, uint32_t* max_torque)
    {
        return read(XL330Reg::ADDR_CURRENT_LIMIT, XL330Reg::SIZE_CURRENT_LIMIT, id, max_torque);
    }

    template<>
    int XDriver<XL330Reg>::readLoad(uint8_t id, uint32_t *present_load)
    {
        return read(XL330Reg::ADDR_PRESENT_CURRENT, XL330Reg::SIZE_PRESENT_CURRENT, id, present_load);
    }

    template<>
    int XDriver<XL330Reg>::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
    {
        return syncRead(XL330Reg::ADDR_PRESENT_CURRENT, XL330Reg::SIZE_PRESENT_CURRENT, id_list, load_list);
    }

    // Stepper : empty class for now
    // cc will it be in XDriver ? or apart StepperDriver class inheriting from XDriver ?
    template<>
    class XDriver<StepperReg> : public AbstractMotorDriver
    {

    public:
        XDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
            AbstractMotorDriver(portHandler, packetHandler)
        {

        }

        ~XDriver() override
        {

        }

        // XDriver interface
    public:

        //specifics to reg type
        std::string str() const override
        {
            std::string s = "TTL Stepper driver \n";
            return s;
        }
        virtual std::string interpreteErrorState(uint32_t hw_state) override{ std::cout << "not implemented" << std::endl; return ""; }
        virtual int checkModelNumber(uint8_t id) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int changeId(uint8_t id, uint8_t new_id) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int changeBaudRate(uint8_t id, uint32_t new_baudrate) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setReturnDelayTime(uint8_t id, uint32_t return_delay_time) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setLimitTemperature(uint8_t id, uint32_t temperature) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setMaxTorque(uint8_t id, uint32_t torque) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setReturnLevel(uint8_t id, uint32_t return_level) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readLimitTemperature(uint8_t id, uint32_t *limit_temperature) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readMaxTorque(uint8_t id, uint32_t *max_torque) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readReturnLevel(uint8_t id, uint32_t *return_level) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setTorqueEnable(uint8_t id, uint32_t torque_enable) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setLed(uint8_t id, uint32_t led_value) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setGoalPosition(uint8_t id, uint32_t position) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setGoalVelocity(uint8_t id, uint32_t velocity) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setGoalTorque(uint8_t id, uint32_t torque) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setPGain(uint8_t id, uint32_t gain) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setIGain(uint8_t id, uint32_t gain) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setDGain(uint8_t id, uint32_t gain) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setff1Gain(uint8_t id, uint32_t gain) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int setff2Gain(uint8_t id, uint32_t gain) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readPosition(uint8_t id, uint32_t *present_position) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readVelocity(uint8_t id, uint32_t *present_velocity) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readLoad(uint8_t id, uint32_t *present_load) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readTemperature(uint8_t id, uint32_t *temperature) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readVoltage(uint8_t id, uint32_t *voltage) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int readHardwareStatus(uint8_t id, uint32_t *hardware_status) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override{ std::cout << "not implemented" << std::endl; return 0; }
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override{ std::cout << "not implemented" << std::endl; return 0; }
    };

} // DynamixelDriver

#endif // XDriver
