/*
    driver.cpp
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

#include "dynamixel_driver/xl320_driver.hpp"
#include "model/motor_type_enum.hpp"

#include "dynamixel_driver/xl320_reg.hpp"
#include "dynamixel_driver/motor_driver.hpp"

using namespace std;

namespace DynamixelDriver
{
    /**
     * @brief XL320Driver::XL320Driver
     * @param portHandler
     * @param packetHandler
     */
    XL320Driver::XL320Driver(shared_ptr<dynamixel::PortHandler> portHandler,
                             shared_ptr<dynamixel::PacketHandler> packetHandler) :
        XDriver(portHandler,
                packetHandler)
    {
    }

    /**
     * @brief XL320Driver::~XL320Driver
     */
    XL320Driver::~XL320Driver()
    {

    }

    /**
     * @brief XL320Driver::str
     * @return
     */
    string XL320Driver::str() const
    {
       return "Driver - type: XL320" + XDriver::str();
    }

    /**
     * @brief XL320Driver::interpreteErrorState
     * @param hw_state
     * @return
     */
    string XL320Driver::interpreteErrorState(uint32_t hw_state)
    {
        string hardware_message;

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

    /**
     * @brief XL320Driver::checkModelNumber
     * @param id
     * @return
     */
    int XL320Driver::checkModelNumber(uint8_t id)
    {
        uint16_t model_number;
        int ping_result = getModelNumber(id, &model_number);

        if (ping_result == COMM_SUCCESS)
        {
            if (model_number && model_number != XL320Reg::MODEL_NUMBER)
            {
                return PING_WRONG_MODEL_NUMBER;
            }
        }

        return ping_result;
    }

    /*
     *  -----------------   WRITE   --------------------
     */

    int XL320Driver::changeId(uint8_t id, uint8_t new_id)
    {
        return write(XL320Reg::ADDR_ID, XL320Reg::SIZE_ID, id, new_id);
    }

    int XL320Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
    {
        return write(XL320Reg::ADDR_BAUDRATE, XL320Reg::SIZE_BAUDRATE, id, new_baudrate);
    }

    int XL320Driver::setLed(uint8_t id, uint32_t led_value)
    {
        return write(XL320Reg::ADDR_LED, XL320Reg::SIZE_LED, id, led_value);
    }

    int XL320Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
    {
        return write(XL320Reg::ADDR_TORQUE_ENABLE, XL320Reg::SIZE_TORQUE_ENABLE, id, torque_enable);
    }

    int XL320Driver::setGoalPosition(uint8_t id, uint32_t position)
    {
        return write(XL320Reg::ADDR_GOAL_POSITION, XL320Reg::SIZE_GOAL_POSITION, id, position);
    }

    int XL320Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
    {
        return write(XL320Reg::ADDR_GOAL_SPEED, XL320Reg::SIZE_GOAL_SPEED, id, velocity);
    }

    int XL320Driver::setGoalTorque(uint8_t id, uint32_t torque)
    {
        return write(XL320Reg::ADDR_GOAL_TORQUE, XL320Reg::SIZE_GOAL_TORQUE, id, torque);
    }

    /*
     *  -----------------   PID   --------------------
     */

    int XL320Driver::setPGain(uint8_t id, uint32_t gain)
    {
        return write(XL320Reg::ADDR_POSITION_P_GAIN, XL320Reg::SIZE_POSITION_P_GAIN, id, gain);
    }

    int XL320Driver::setIGain(uint8_t id, uint32_t gain)
    {
        return write(XL320Reg::ADDR_POSITION_I_GAIN, XL320Reg::SIZE_POSITION_I_GAIN, id, gain);
    }

    int XL320Driver::setDGain(uint8_t id, uint32_t gain)
    {
        return write(XL320Reg::ADDR_POSITION_D_GAIN, XL320Reg::SIZE_POSITION_D_GAIN, id, gain);
    }

    int XL320Driver::setff1Gain(uint8_t /*id*/, uint32_t /*gain*/)
    {
        //not available for XL320
        return COMM_TX_ERROR;
    }

    int XL320Driver::setff2Gain(uint8_t /*id*/, uint32_t /*gain*/)
    {
        //not available for XL320
        return COMM_TX_ERROR;
    }

    // others

    int XL320Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
    {
        return write(XL320Reg::ADDR_RETURN_DELAY_TIME, XL320Reg::SIZE_RETURN_DELAY_TIME, id, return_delay_time);
    }

    int XL320Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
    {
        return write(XL320Reg::ADDR_TEMPERATURE_LIMIT, XL320Reg::SIZE_TEMPERATURE_LIMIT, id, temperature);
    }

    int XL320Driver::setMaxTorque(uint8_t id, uint32_t torque)
    {
        return write(XL320Reg::ADDR_MAX_TORQUE, XL320Reg::SIZE_MAX_TORQUE, id, torque);
    }

    int XL320Driver::setReturnLevel(uint8_t id, uint32_t return_level)
    {
        return write(XL320Reg::ADDR_RETURN_LEVEL, XL320Reg::SIZE_RETURN_LEVEL, id, return_level);
    }

    int XL320Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
    {
        return write(XL320Reg::ADDR_ALARM_SHUTDOWN, XL320Reg::SIZE_ALARM_SHUTDOWN, id, alarm_shutdown);
    }

    /*
     *  -----------------   SYNC WRITE   --------------------
     */

    int XL320Driver::syncWritePositionGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &position_list)
    {
        return syncWrite(XL320Reg::ADDR_GOAL_POSITION, XL320Reg::SIZE_GOAL_POSITION, id_list, position_list);
    }
    int XL320Driver::syncWriteVelocityGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &velocity_list)
    {
        return syncWrite(XL320Reg::ADDR_GOAL_SPEED, XL320Reg::SIZE_GOAL_SPEED, id_list, velocity_list);
    }
    int XL320Driver::syncWriteTorqueGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_list)
    {
        return syncWrite(XL320Reg::ADDR_GOAL_TORQUE, XL320Reg::SIZE_GOAL_TORQUE, id_list, torque_list);
    }

    int XL320Driver::syncWriteTorqueEnable(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_enable_list)
    {
        return syncWrite(XL320Reg::ADDR_TORQUE_ENABLE, XL320Reg::SIZE_TORQUE_ENABLE, id_list, torque_enable_list);
    }

    int XL320Driver::syncWriteLed(const vector<uint8_t> &id_list, const vector<uint32_t> &led_list)
    {
        return syncWrite(XL320Reg::ADDR_LED, XL320Reg::SIZE_LED, id_list, led_list);
    }

    /*
     *  -----------------   READ   --------------------
     */

    int XL320Driver::readPosition(uint8_t id, uint32_t *present_position)
    {
        return read(XL320Reg::ADDR_PRESENT_POSITION, XL320Reg::SIZE_PRESENT_POSITION, id, present_position);
    }

    int XL320Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
    {
        return read(XL320Reg::ADDR_PRESENT_SPEED, XL320Reg::SIZE_PRESENT_SPEED, id, present_velocity);
    }

    int XL320Driver::readLoad(uint8_t id, uint32_t *present_load)
    {
        return read(XL320Reg::ADDR_PRESENT_LOAD, XL320Reg::SIZE_PRESENT_LOAD, id, present_load);
    }

    int XL320Driver::readTemperature(uint8_t id, uint32_t *temperature)
    {
        return read(XL320Reg::ADDR_PRESENT_TEMPERATURE, XL320Reg::SIZE_PRESENT_TEMPERATURE, id, temperature);
    }

    int XL320Driver::readVoltage(uint8_t id, uint32_t *voltage)
    {
        return read(XL320Reg::ADDR_PRESENT_VOLTAGE, XL320Reg::SIZE_PRESENT_VOLTAGE, id, voltage);
    }

    int XL320Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
    {
        return read(XL320Reg::ADDR_HW_ERROR_STATUS, XL320Reg::SIZE_HW_ERROR_STATUS, id, hardware_status);
    }
    int XL320Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
    {
        return read(XL320Reg::ADDR_RETURN_DELAY_TIME, XL320Reg::SIZE_RETURN_DELAY_TIME, id, return_delay_time);
    }

    int XL320Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
    {
        return read(XL320Reg::ADDR_TEMPERATURE_LIMIT, XL320Reg::SIZE_TEMPERATURE_LIMIT, id, limit_temperature);
    }

    int XL320Driver::readMaxTorque(uint8_t id, uint32_t *max_torque)
    {
        return read(XL320Reg::ADDR_MAX_TORQUE, XL320Reg::SIZE_MAX_TORQUE, id, max_torque);
    }

    int XL320Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
    {
        return read(XL320Reg::ADDR_RETURN_LEVEL, XL320Reg::SIZE_RETURN_LEVEL, id, return_level);
    }

    int XL320Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
    {
        return read(XL320Reg::ADDR_ALARM_SHUTDOWN, XL320Reg::SIZE_ALARM_SHUTDOWN, id, alarm_shutdown);
    }

    /*
     *  -----------------   SYNC READ   --------------------
     */

    int XL320Driver::syncReadPosition(const vector<uint8_t> &id_list, vector<uint32_t> &position_list)
    {
        return syncRead(XL320Reg::ADDR_PRESENT_POSITION, XL320Reg::SIZE_PRESENT_POSITION, id_list, position_list);
    }

    int XL320Driver::syncReadVelocity(const vector<uint8_t> &id_list, vector<uint32_t> &velocity_list)
    {
        return syncRead(XL320Reg::ADDR_PRESENT_SPEED, XL320Reg::SIZE_PRESENT_SPEED, id_list, velocity_list);
    }
    int XL320Driver::syncReadLoad(const vector<uint8_t> &id_list, vector<uint32_t> &load_list)
    {
        return syncRead(XL320Reg::ADDR_PRESENT_LOAD, XL320Reg::SIZE_PRESENT_LOAD, id_list, load_list);
    }

    int XL320Driver::syncReadTemperature(const vector<uint8_t> &id_list, vector<uint32_t> &temperature_list)
    {
        return syncRead(XL320Reg::ADDR_PRESENT_TEMPERATURE, XL320Reg::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
    }

    int XL320Driver::syncReadVoltage(const vector<uint8_t> &id_list, vector<uint32_t> &voltage_list)
    {
        return syncRead(XL320Reg::ADDR_PRESENT_VOLTAGE, XL320Reg::SIZE_PRESENT_VOLTAGE, id_list, voltage_list);
    }

    int XL320Driver::syncReadHwErrorStatus(const vector<uint8_t> &id_list, vector<uint32_t> &hw_error_list)
    {
        return syncRead(XL320Reg::ADDR_HW_ERROR_STATUS, XL320Reg::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
    }

} // DynamixelDriver
