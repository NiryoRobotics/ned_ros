/*
    xl430_driver.cpp
    Copyright (C) 2018 Niryo
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

#include "dynamixel_driver/xl430_driver.hpp"
#include "model/motor_type_enum.hpp"

using namespace std;

namespace DynamixelDriver
{

    /**
     * @brief XL430Driver::XL430Driver
     * @param portHandler
     * @param packetHandler
     */
    XL430Driver::XL430Driver(shared_ptr<dynamixel::PortHandler> portHandler,
                             shared_ptr<dynamixel::PacketHandler> packetHandler)
        : XDriver(common::model::EMotorType::XL430,
                  portHandler,
                  packetHandler)
    {
    }

    string XL430Driver::interpreteErrorState(uint32_t hw_state)
    {
        string hardware_message;

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

    /**
     * @brief XL430Driver::checkModelNumber
     * @param id
     * @return
     */
    int XL430Driver::checkModelNumber(uint8_t id)
    {
        uint16_t model_number;
        int ping_result = getModelNumber(id, &model_number);

        if (ping_result == COMM_SUCCESS)
        {
            if (model_number && model_number != XL430_MODEL_NUMBER)
            {
                return PING_WRONG_MODEL_NUMBER;
            }
        }

        return ping_result;
    }

    /*
     *  -----------------   WRITE   --------------------
     */

    int XL430Driver::changeId(uint8_t id, uint8_t new_id)
    {
        return write(XL430_ADDR_ID, 1, id, new_id);
    }

    int XL430Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
    {
        return write(XL430_ADDR_BAUDRATE, 1, id, new_baudrate);
    }

    int XL430Driver::setLed(uint8_t id, uint32_t led_value)
    {
        return write(XL430_ADDR_LED, 1, id, led_value);
    }

    int XL430Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
    {
        return write(XL430_ADDR_TORQUE_ENABLE, 1, id, torque_enable);
    }

    int XL430Driver::setGoalPosition(uint8_t id, uint32_t position)
    {
        return write(XL430_ADDR_GOAL_POSITION, 4, id, position);
    }

    int XL430Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
    {
        return write(XL430_ADDR_GOAL_VELOCITY, 4, id, velocity);
    }

    int XL430Driver::setGoalTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        // No goal torque for this motor ?
        return COMM_TX_ERROR;
    }

    int XL430Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
    {
        return write(XL430_ADDR_RETURN_DELAY_TIME, 1, id, return_delay_time);
    }

    int XL430Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
    {
        return write(XL430_ADDR_TEMPERATURE_LIMIT, 1, id, temperature);
    }

    int XL430Driver::setMaxTorque(uint8_t /*id*/, uint32_t /*torque*/)
    {
        // No max torque setting for this motor ?
        return COMM_TX_ERROR;
    }

    int XL430Driver::setReturnLevel(uint8_t id, uint32_t return_level)
    {
        return write(XL430_ADDR_STATUS_RETURN_LEVEL, 1, id, return_level);
    }

    int XL430Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
    {
        return write(XL430_ADDR_ALARM_SHUTDOWN, 1, id, alarm_shutdown);
    }
    /*
     *  -----------------   PID   --------------------
     */

    int XL430Driver::setPGain(uint8_t id, uint32_t gain)
    {
        return write(XL430_ADDR_P_GAIN, 2, id, gain);
    }

    int XL430Driver::setIGain(uint8_t id, uint32_t gain)
    {
        return write(XL430_ADDR_I_GAIN, 2, id, gain);
    }

    int XL430Driver::setDGain(uint8_t id, uint32_t gain)
    {
        return write(XL430_ADDR_D_GAIN, 2, id, gain);
    }

    int XL430Driver::setff1Gain(uint8_t id, uint32_t gain)
    {
        return write(XL430_ADDR_FF1_GAIN, 2, id, gain);
    }

    int XL430Driver::setff2Gain(uint8_t id, uint32_t gain)
    {
        return write(XL430_ADDR_FF2_GAIN, 2, id, gain);
    }

    /*
     *  -----------------   SYNC WRITE   --------------------
     */

    int XL430Driver::syncWritePositionGoal(const vector<uint8_t> &id_list,const  vector<uint32_t> &position_list)
    {
        return syncWrite(XL430_ADDR_GOAL_POSITION, 4, id_list, position_list);
    }
    int XL430Driver::syncWriteVelocityGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &velocity_list)
    {
        return syncWrite(XL430_ADDR_GOAL_VELOCITY, 4, id_list, velocity_list);
    }
    int XL430Driver::syncWriteTorqueGoal(const vector<uint8_t>& /*id_list*/, const vector<uint32_t> &/*torque_list*/)
    {
        // No goal torque for this motor ?
        return COMM_TX_ERROR;
    }

    int XL430Driver::syncWriteTorqueEnable(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_enable_list)
    {
        return syncWrite(XL430_ADDR_TORQUE_ENABLE, 1, id_list, torque_enable_list);
    }

    int XL430Driver::syncWriteLed(const vector<uint8_t> &id_list, const vector<uint32_t> &led_list)
    {
        return syncWrite(XL430_ADDR_LED, 1, id_list, led_list);
    }

    /*
     *  -----------------   READ   --------------------
     */

    int XL430Driver::readPosition(uint8_t id, uint32_t *present_position)
    {
        return read(XL430_ADDR_PRESENT_POSITION, 4, id, present_position);
    }

    int XL430Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
    {
        return read(XL430_ADDR_PRESENT_VELOCITY, 4, id, present_velocity);
    }

    int XL430Driver::readLoad(uint8_t id, uint32_t *present_load)
    {
        return read(XL430_ADDR_PRESENT_LOAD, 2, id, present_load);
    }

    int XL430Driver::readTemperature(uint8_t id, uint32_t *temperature)
    {
        return read(XL430_ADDR_PRESENT_TEMPERATURE, 1, id, temperature);
    }

    int XL430Driver::readVoltage(uint8_t id, uint32_t *voltage)
    {
        return read(XL430_ADDR_PRESENT_VOLTAGE, 2, id, voltage);
    }

    int XL430Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
    {
        return read(XL430_ADDR_HW_ERROR_STATUS, 1, id, hardware_status);
    }
    int XL430Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
    {
        return read(XL430_ADDR_RETURN_DELAY_TIME, 1, id, return_delay_time);
    }

    int XL430Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
    {
        return read(XL430_ADDR_TEMPERATURE_LIMIT, 1, id, limit_temperature);
    }

    int XL430Driver::readMaxTorque(uint8_t /*id*/, uint32_t* /*max_torque*/)
    {
        // No max torque setting for this motor ?
        return COMM_TX_ERROR;
    }

    int XL430Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
    {
        return read(XL430_ADDR_STATUS_RETURN_LEVEL, 1, id, return_level);
    }

    int XL430Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
    {
        return read(XL430_ADDR_ALARM_SHUTDOWN, 1, id, alarm_shutdown);
    }

    /*
     *  -----------------   SYNC READ   --------------------
     */

    int XL430Driver::syncReadPosition(const vector<uint8_t> &id_list, vector<uint32_t> &position_list)
    {
        return syncRead(XL430_ADDR_PRESENT_POSITION, 4, id_list, position_list);
    }

    int XL430Driver::syncReadVelocity(const vector<uint8_t> &id_list, vector<uint32_t> &velocity_list)
    {
        return syncRead(XL430_ADDR_PRESENT_VELOCITY, 4, id_list, velocity_list);
    }
    int XL430Driver::syncReadLoad(const vector<uint8_t> &id_list, vector<uint32_t> &load_list)
    {
        return syncRead(XL430_ADDR_PRESENT_LOAD, 2, id_list, load_list);
    }

    int XL430Driver::syncReadTemperature(const vector<uint8_t> &id_list, vector<uint32_t> &temperature_list)
    {
        return syncRead(XL430_ADDR_PRESENT_TEMPERATURE, 1, id_list, temperature_list);
    }

    int XL430Driver::syncReadVoltage(const vector<uint8_t> &id_list, vector<uint32_t> &voltage_list)
    {
        return syncRead(XL430_ADDR_PRESENT_VOLTAGE, 2, id_list, voltage_list);
    }

    int XL430Driver::syncReadHwErrorStatus(const vector<uint8_t> &id_list, vector<uint32_t> &hw_error_list)
    {
        return syncRead(XL430_ADDR_HW_ERROR_STATUS, 1, id_list, hw_error_list);
    }

}
