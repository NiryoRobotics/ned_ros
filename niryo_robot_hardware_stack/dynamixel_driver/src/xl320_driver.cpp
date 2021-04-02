/*
    xl320_driver.cpp
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

using namespace std;

namespace DynamixelDriver
{
    /**
     * @brief XL320Driver::XL320Driver
     * @param portHandler
     * @param packetHandler
     */
    XL320Driver::XL320Driver(shared_ptr<dynamixel::PortHandler> &portHandler,
                             shared_ptr<dynamixel::PacketHandler> &packetHandler)
        : XDriver(DxlMotorType_t::MOTOR_TYPE_XL320, portHandler, packetHandler)
    {
    }

    string XL320Driver::interpreteErrorState(uint32_t hw_state)
    {
        string hardware_message;

        if (hw_state & 0b00000001)
        {
            hardware_message += "Overload";
        }
        if (hw_state & 0b00000010)
        {
            if (hardware_message != "")
                hardware_message += ", ";
            hardware_message += "OverHeating";
        }
        if (hw_state & 0b00000100)
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
            if (model_number && model_number != XL320_MODEL_NUMBER)
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
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_ID, new_id);
    }

    int XL320Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_BAUDRATE, (uint8_t)new_baudrate);
    }

    int XL320Driver::setLed(uint8_t id, uint32_t led_value)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_LED, (uint8_t)led_value);
    }

    int XL320Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_TORQUE_ENABLE, (uint8_t)torque_enable);
    }

    int XL320Driver::setGoalPosition(uint8_t id, uint32_t position)
    {
        return _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_GOAL_POSITION, (uint16_t)position);
    }

    int XL320Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
    {
        return _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_GOAL_SPEED, (uint16_t)velocity);
    }

    int XL320Driver::setGoalTorque(uint8_t id, uint32_t torque)
    {
        return _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_GOAL_TORQUE, (uint16_t)torque);
    }

    int XL320Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_RETURN_DELAY_TIME, (uint8_t)return_delay_time);
    }

    int XL320Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_LIMIT_TEMPERATURE, (uint8_t)temperature);
    }

    int XL320Driver::setMaxTorque(uint8_t id, uint32_t torque)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_MAX_TORQUE, (uint8_t)torque);
    }

    int XL320Driver::setReturnLevel(uint8_t id, uint32_t return_level)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_RETURN_LEVEL, (uint8_t)return_level);
    }

    int XL320Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL320_ADDR_ALARM_SHUTDOWN, (uint8_t)alarm_shutdown);
    }

    /*
     *  -----------------   SYNC WRITE   --------------------
     */

    int XL320Driver::syncWritePositionGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &position_list)
    {
        return syncWrite2Bytes(XL320_ADDR_GOAL_POSITION, id_list, position_list);
    }
    int XL320Driver::syncWriteVelocityGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &velocity_list)
    {
        return syncWrite2Bytes(XL320_ADDR_GOAL_SPEED, id_list, velocity_list);
    }
    int XL320Driver::syncWriteTorqueGoal(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_list)
    {
        return syncWrite2Bytes(XL320_ADDR_GOAL_TORQUE, id_list, torque_list);
    }

    int XL320Driver::syncWriteTorqueEnable(const vector<uint8_t> &id_list, const vector<uint32_t> &torque_enable_list)
    {
        return syncWrite1Byte(XL320_ADDR_TORQUE_ENABLE, id_list, torque_enable_list);
    }

    int XL320Driver::syncWriteLed(const vector<uint8_t> &id_list, const vector<uint32_t> &led_list)
    {
        return syncWrite1Byte(XL320_ADDR_LED, id_list, led_list);
    }

    /*
     *  -----------------   READ   --------------------
     */

    int XL320Driver::readPosition(uint8_t id, uint32_t *present_position)
    {
        return read2Bytes(XL320_ADDR_PRESENT_POSITION, id, present_position);
    }

    int XL320Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
    {
        return read2Bytes(XL320_ADDR_PRESENT_SPEED, id, present_velocity);
    }

    int XL320Driver::readLoad(uint8_t id, uint32_t *present_load)
    {
        return read2Bytes(XL320_ADDR_PRESENT_LOAD, id, present_load);
    }

    int XL320Driver::readTemperature(uint8_t id, uint32_t *temperature)
    {
        return read1Byte(XL320_ADDR_PRESENT_TEMPERATURE, id, temperature);
    }

    int XL320Driver::readVoltage(uint8_t id, uint32_t *voltage)
    {
        return read1Byte(XL320_ADDR_PRESENT_VOLTAGE, id, voltage);
    }

    int XL320Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
    {
        return read1Byte(XL320_ADDR_HW_ERROR_STATUS, id, hardware_status);
    }
    int XL320Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
    {
        return read1Byte(XL320_ADDR_RETURN_DELAY_TIME, id, return_delay_time);
    }

    int XL320Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
    {
        return read1Byte(XL320_ADDR_LIMIT_TEMPERATURE, id, limit_temperature);
    }

    int XL320Driver::readMaxTorque(uint8_t id, uint32_t *max_torque)
    {
        return read2Bytes(XL320_ADDR_MAX_TORQUE, id, max_torque);
    }

    int XL320Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
    {
        return read1Byte(XL320_ADDR_RETURN_LEVEL, id, return_level);
    }

    int XL320Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
    {
        return read1Byte(XL320_ADDR_ALARM_SHUTDOWN, id, alarm_shutdown);
    }

    /*
     *  -----------------   SYNC READ   --------------------
     */

    int XL320Driver::syncReadPosition(const vector<uint8_t> &id_list, vector<uint32_t> &position_list)
    {
        return syncRead(XL320_ADDR_PRESENT_POSITION, DXL_LEN_TWO_BYTES, id_list, position_list);
    }

    int XL320Driver::syncReadVelocity(const vector<uint8_t> &id_list, vector<uint32_t> &velocity_list)
    {
        return syncRead(XL320_ADDR_PRESENT_SPEED, DXL_LEN_TWO_BYTES, id_list, velocity_list);
    }
    int XL320Driver::syncReadLoad(const vector<uint8_t> &id_list, vector<uint32_t> &load_list)
    {
        return syncRead(XL320_ADDR_PRESENT_LOAD, DXL_LEN_TWO_BYTES, id_list, load_list);
    }

    int XL320Driver::syncReadTemperature(const vector<uint8_t> &id_list, vector<uint32_t> &temperature_list)
    {
        return syncRead(XL320_ADDR_PRESENT_TEMPERATURE, DXL_LEN_ONE_BYTE, id_list, temperature_list);
    }

    int XL320Driver::syncReadVoltage(const vector<uint8_t> &id_list, vector<uint32_t> &voltage_list)
    {
        return syncRead(XL320_ADDR_PRESENT_VOLTAGE, DXL_LEN_ONE_BYTE, id_list, voltage_list);
    }

    int XL320Driver::syncReadHwErrorStatus(const vector<uint8_t> &id_list, vector<uint32_t> &hw_error_list)
    {
        return syncRead(XL320_ADDR_HW_ERROR_STATUS, DXL_LEN_ONE_BYTE, id_list, hw_error_list);
    }

    /**
     * @brief XL320Driver::customWrite : restricted to 1 and 2 bytes setting
     * @param id
     * @param value
     * @param reg_address
     * @param byte_number
     * @return
     */
    int XL320Driver::customWrite(uint8_t id, uint8_t reg_address, uint32_t value, uint8_t byte_number)
    {
        if (4 != byte_number)
            return XDriver::customWrite(id, reg_address, value, byte_number);

        printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
        return COMM_TX_FAIL;
    }

    /**
     * @brief XL320Driver::customRead : restricted to 1 and 2 bytes reading
     * @param id
     * @param value
     * @param reg_address
     * @param byte_number
     * @return
     */
    int XL320Driver::customRead(uint8_t id, uint8_t reg_address, uint32_t &value, uint8_t byte_number)
    {
        if (4 != byte_number)
            return XDriver::customRead(id, reg_address, value, byte_number);

        printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
        return COMM_TX_FAIL;
    }
}
