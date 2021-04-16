/*
    xl330_driver.cpp
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

#include "dynamixel_driver/xl330_driver.hpp"
#include "model/motor_type_enum.hpp"

using namespace std;

namespace DynamixelDriver
{

    /**
     * @brief XL330Driver::XL330Driver
     * @param portHandler
     * @param packetHandler
     */
    XL330Driver::XL330Driver(shared_ptr<dynamixel::PortHandler> portHandler,
                             shared_ptr<dynamixel::PacketHandler> packetHandler)
        : XDriver(common::model::EMotorType::MOTOR_TYPE_XL330,
                  portHandler,
                  packetHandler)
    {
    }

    string XL330Driver::interpreteErrorState(uint32_t hw_state)
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
     * @brief XL330Driver::checkModelNumber
     * @param id
     * @return
     */
    int XL330Driver::checkModelNumber(uint8_t id)
    {
        uint16_t model_number;
        int ping_result = getModelNumber(id, &model_number);

        if (ping_result == COMM_SUCCESS)
        {
            if (model_number && model_number != XL330_MODEL_NUMBER)
            {
                return PING_WRONG_MODEL_NUMBER;
            }
        }

        return ping_result;
    }

    /*
     *  -----------------   WRITE   --------------------
     */

    int XL330Driver::changeId(uint8_t id, uint8_t new_id)
    {
        return write1Byte(XL330_ADDR_ID, id, new_id);
    }

    int XL330Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
    {
        return write1Byte(XL330_ADDR_BAUDRATE, id, static_cast<uint8_t>(new_baudrate));
    }

    int XL330Driver::setLed(uint8_t id, uint32_t led_value)
    {
        return write1Byte(XL330_ADDR_LED, id, static_cast<uint8_t>(led_value));
    }

    int XL330Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
    {
        return write1Byte(XL330_ADDR_TORQUE_ENABLE, id, static_cast<uint8_t>(torque_enable));
    }

    int XL330Driver::setGoalPosition(uint8_t id, uint32_t position)
    {
        return write4Bytes(XL330_ADDR_GOAL_POSITION, id, position);
    }

    int XL330Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
    {
        return write4Bytes(XL330_ADDR_GOAL_VELOCITY, id, velocity);
    }

    int XL330Driver::setGoalTorque(uint8_t id, uint32_t torque)
    {
        return write2Bytes(XL330_ADDR_GOAL_CURRENT, id, static_cast<uint16_t>(torque));
    }

    int XL330Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
    {
        return write1Byte(XL330_ADDR_RETURN_DELAY_TIME, id, static_cast<uint8_t>(return_delay_time));
    }

    int XL330Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
    {
        return write1Byte(XL330_ADDR_TEMPERATURE_LIMIT, id, static_cast<uint8_t>(temperature));
    }

    int XL330Driver::setMaxTorque(uint8_t id, uint32_t torque)
    {
        return write2Bytes(XL330_ADDR_CURRENT_LIMIT, id, static_cast<uint16_t>(torque));
    }

    int XL330Driver::setReturnLevel(uint8_t id, uint32_t return_level)
    {
        return write1Byte(XL330_ADDR_STATUS_RETURN_LEVEL, id, static_cast<uint8_t>(return_level));
    }

    int XL330Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
    {
        return write1Byte(XL330_ADDR_ALARM_SHUTDOWN, id, static_cast<uint8_t>(alarm_shutdown));
    }

    /*
     *  -----------------   PID   --------------------
     */

    int XL330Driver::setPGain(uint8_t id, uint32_t gain)
    {
        return write2Bytes(XL330_ADDR_P_GAIN, id, static_cast<uint16_t>(gain));
    }

    int XL330Driver::setIGain(uint8_t id, uint32_t gain)
    {
        return write2Bytes(XL330_ADDR_I_GAIN, id, static_cast<uint16_t>(gain));
    }

    int XL330Driver::setDGain(uint8_t id, uint32_t gain)
    {
        return write2Bytes(XL330_ADDR_D_GAIN, id, static_cast<uint16_t>(gain));
    }

    int XL330Driver::setff1Gain(uint8_t id, uint32_t gain)
    {
        return write2Bytes(XL330_ADDR_FF1_GAIN, id, static_cast<uint16_t>(gain));
    }

    int XL330Driver::setff2Gain(uint8_t id, uint32_t gain)
    {
        return write2Bytes(XL330_ADDR_FF2_GAIN, id, static_cast<uint16_t>(gain));
    }

    /*
     *  -----------------   SYNC WRITE   --------------------
     */

    int XL330Driver::syncWritePositionGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &position_list)
    {
        return syncWrite(XL330_ADDR_GOAL_POSITION, DXL_LEN_FOUR_BYTES, id_list, position_list);
    }

    int XL330Driver::syncWriteVelocityGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &velocity_list)
    {
        return syncWrite(XL330_ADDR_GOAL_VELOCITY, DXL_LEN_FOUR_BYTES, id_list, velocity_list);
    }

    int XL330Driver::syncWriteTorqueGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_list)
    {
        return syncWrite(XL330_ADDR_GOAL_CURRENT, DXL_LEN_TWO_BYTES, id_list, torque_list);
    }

    int XL330Driver::syncWriteTorqueEnable(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_enable_list)
    {
        return syncWrite(XL330_ADDR_TORQUE_ENABLE, DXL_LEN_ONE_BYTE, id_list, torque_enable_list);
    }

    int XL330Driver::syncWriteLed(const vector<uint8_t> &id_list, const vector<uint32_t> &led_list)
    {
        return syncWrite(XL330_ADDR_LED, DXL_LEN_ONE_BYTE, id_list, led_list);
    }

    /*
     *  -----------------   READ   --------------------
     */

    int XL330Driver::readPosition(uint8_t id, uint32_t *present_position)
    {
        return read4Bytes(XL330_ADDR_PRESENT_POSITION, id, present_position);
    }

    int XL330Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
    {
        return read4Bytes(XL330_ADDR_PRESENT_VELOCITY, id, present_velocity);
    }

    int XL330Driver::readLoad(uint8_t id, uint32_t *present_load)
    {
        return read2Bytes(XL330_ADDR_PRESENT_CURRENT, id, present_load);
    }

    int XL330Driver::readTemperature(uint8_t id, uint32_t *temperature)
    {
        return read1Byte(XL330_ADDR_PRESENT_TEMPERATURE, id, temperature);
    }

    int XL330Driver::readVoltage(uint8_t id, uint32_t *voltage)
    {
        return read2Bytes(XL330_ADDR_PRESENT_VOLTAGE, id, voltage);
    }

    int XL330Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
    {
        return read1Byte(XL330_ADDR_HW_ERROR_STATUS, id, hardware_status);
    }

    int XL330Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
    {
        return read1Byte(XL330_ADDR_RETURN_DELAY_TIME, id, return_delay_time);
    }

    int XL330Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
    {
        return read1Byte(XL330_ADDR_TEMPERATURE_LIMIT, id, limit_temperature);
    }

    int XL330Driver::readMaxTorque(uint8_t id, uint32_t *max_torque)
    {
        return read2Bytes(XL330_ADDR_CURRENT_LIMIT, id, max_torque);
    }

    int XL330Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
    {
        return read1Byte(XL330_ADDR_STATUS_RETURN_LEVEL, id, return_level);
    }

    int XL330Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
    {
        return read1Byte(XL330_ADDR_ALARM_SHUTDOWN, id, alarm_shutdown);
    }

    /*
     *  -----------------   SYNC READ   --------------------
     */

    int XL330Driver::syncReadPosition(const vector<uint8_t> &id_list, vector<uint32_t> &position_list)
    {
        return syncRead(XL330_ADDR_PRESENT_POSITION, DXL_LEN_FOUR_BYTES, id_list, position_list);
    }

    int XL330Driver::syncReadVelocity(const vector<uint8_t> &id_list, vector<uint32_t> &velocity_list)
    {
        return syncRead(XL330_ADDR_PRESENT_VELOCITY, DXL_LEN_FOUR_BYTES, id_list, velocity_list);
    }
    int XL330Driver::syncReadLoad(const vector<uint8_t> &id_list, vector<uint32_t> &load_list)
    {
        return syncRead(XL330_ADDR_PRESENT_CURRENT, DXL_LEN_TWO_BYTES, id_list, load_list);
    }

    int XL330Driver::syncReadTemperature(const vector<uint8_t> &id_list, vector<uint32_t> &temperature_list)
    {
        return syncRead(XL330_ADDR_PRESENT_TEMPERATURE, DXL_LEN_ONE_BYTE, id_list, temperature_list);
    }

    int XL330Driver::syncReadVoltage(const vector<uint8_t> &id_list, vector<uint32_t> &voltage_list)
    {
        return syncRead(XL330_ADDR_PRESENT_VOLTAGE, DXL_LEN_TWO_BYTES, id_list, voltage_list);
    }

    int XL330Driver::syncReadHwErrorStatus(const vector<uint8_t> &id_list, vector<uint32_t> &hw_error_list)
    {
        return syncRead(XL330_ADDR_HW_ERROR_STATUS, DXL_LEN_ONE_BYTE, id_list, hw_error_list);
    }

}
