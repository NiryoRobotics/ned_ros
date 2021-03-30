/*
    xc430_driver.cpp
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

#include "dynamixel_driver/xc430_driver.hpp"

namespace DynamixelDriver
{
    /**
     * @brief XC430Driver::XC430Driver
     * @param portHandler
     * @param packetHandler
     */
    XC430Driver::XC430Driver(std::shared_ptr<dynamixel::PortHandler> &portHandler,
                             std::shared_ptr<dynamixel::PacketHandler> &packetHandler)
        : XDriver(portHandler, packetHandler)
    {
    }

    /**
     * @brief XC430Driver::checkModelNumber
     * @param id
     * @return
     */
    int XC430Driver::checkModelNumber(uint8_t id)
    {
        uint16_t model_number;
        int ping_result = getModelNumber(id, &model_number);

        if (COMM_SUCCESS == ping_result)
        {
            if (model_number && model_number != XC430_MODEL_NUMBER)
            {
                return PING_WRONG_MODEL_NUMBER;
            }
        }

        return ping_result;
    }

    /*
     *  -----------------   WRITE   --------------------
     */

    int XC430Driver::changeId(uint8_t id, uint8_t new_id)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_ID, new_id);
    }

    int XC430Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_BAUDRATE, (uint8_t)new_baudrate);
    }

    int XC430Driver::setLed(uint8_t id, uint32_t led_value)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_LED, (uint8_t)led_value);
    }

    int XC430Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_TORQUE_ENABLE, (uint8_t)torque_enable);
    }

    int XC430Driver::setGoalPosition(uint8_t id, uint32_t position)
    {
        return _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_GOAL_POSITION, position);
    }

    int XC430Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
    {
        return _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_GOAL_VELOCITY, velocity);
    }

    int XC430Driver::setGoalTorque(uint8_t id, uint32_t torque)
    {
        // No goal torque for this motor ?
        return COMM_TX_ERROR;
    }

    int XC430Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_RETURN_DELAY_TIME, (uint8_t)return_delay_time);
    }

    int XC430Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_TEMPERATURE_LIMIT, (uint8_t)temperature);
    }

    int XC430Driver::setMaxTorque(uint8_t id, uint32_t torque)
    {
        // No max torque setting for this motor ?
        return COMM_TX_ERROR;
    }

    int XC430Driver::setReturnLevel(uint8_t id, uint32_t return_level)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_STATUS_RETURN_LEVEL, (uint8_t)return_level);
    }

    int XC430Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XC430_ADDR_ALARM_SHUTDOWN, (uint8_t)alarm_shutdown);
    }

    /*
     *  -----------------   SYNC WRITE   --------------------
     */

    int XC430Driver::syncWritePositionGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
    {
        return syncWrite4Bytes(XC430_ADDR_GOAL_POSITION, id_list, position_list);
    }
    int XC430Driver::syncWriteVelocityGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
    {
        return syncWrite4Bytes(XC430_ADDR_GOAL_VELOCITY, id_list, velocity_list);
    }
    int XC430Driver::syncWriteTorqueGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list)
    {
        // No goal torque for this motor ?
        return COMM_TX_ERROR;
    }

    int XC430Driver::syncWriteTorqueEnable(std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list)
    {
        return syncWrite1Byte(XC430_ADDR_TORQUE_ENABLE, id_list, torque_enable_list);
    }

    int XC430Driver::syncWriteLed(std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list)
    {
        return syncWrite1Byte(XC430_ADDR_LED, id_list, led_list);
    }

    /*
     *  -----------------   READ   --------------------
     */

    int XC430Driver::readPosition(uint8_t id, uint32_t *present_position)
    {
        return read4Bytes(XC430_ADDR_PRESENT_POSITION, id, present_position);
    }

    int XC430Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
    {
        return read4Bytes(XC430_ADDR_PRESENT_VELOCITY, id, present_velocity);
    }

    int XC430Driver::readLoad(uint8_t id, uint32_t *present_load)
    {
        return read2Bytes(XC430_ADDR_PRESENT_LOAD, id, present_load);
    }

    int XC430Driver::readTemperature(uint8_t id, uint32_t *temperature)
    {
        return read1Byte(XC430_ADDR_PRESENT_TEMPERATURE, id, temperature);
    }

    int XC430Driver::readVoltage(uint8_t id, uint32_t *voltage)
    {
        return read2Bytes(XC430_ADDR_PRESENT_VOLTAGE, id, voltage);
    }

    int XC430Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
    {
        return read1Byte(XC430_ADDR_HW_ERROR_STATUS, id, hardware_status);
    }
    int XC430Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
    {
        return read1Byte(XC430_ADDR_RETURN_DELAY_TIME, id, return_delay_time);
    }

    int XC430Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
    {
        return read1Byte(XC430_ADDR_TEMPERATURE_LIMIT, id, limit_temperature);
    }

    int XC430Driver::readMaxTorque(uint8_t id, uint32_t *max_torque)
    {
        // No max torque setting for this motor ?
        return COMM_TX_ERROR;
    }

    int XC430Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
    {
        return read1Byte(XC430_ADDR_STATUS_RETURN_LEVEL, id, return_level);
    }

    int XC430Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
    {
        return read1Byte(XC430_ADDR_ALARM_SHUTDOWN, id, alarm_shutdown);
    }

    /*
     *  -----------------   SYNC READ   --------------------
     */

    int XC430Driver::syncReadPosition(std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
    {
        return syncRead(XC430_ADDR_PRESENT_POSITION, DXL_LEN_FOUR_BYTES, id_list, position_list);
    }

    int XC430Driver::syncReadVelocity(std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
    {
        return syncRead(XC430_ADDR_PRESENT_VELOCITY, DXL_LEN_FOUR_BYTES, id_list, velocity_list);
    }
    int XC430Driver::syncReadLoad(std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
    {
        return syncRead(XC430_ADDR_PRESENT_LOAD, DXL_LEN_TWO_BYTES, id_list, load_list);
    }

    int XC430Driver::syncReadTemperature(std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
    {
        return syncRead(XC430_ADDR_PRESENT_TEMPERATURE, DXL_LEN_ONE_BYTE, id_list, temperature_list);
    }

    int XC430Driver::syncReadVoltage(std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
    {
        return syncRead(XC430_ADDR_PRESENT_VOLTAGE, DXL_LEN_TWO_BYTES, id_list, voltage_list);
    }

    int XC430Driver::syncReadHwErrorStatus(std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
    {
        return syncRead(XC430_ADDR_HW_ERROR_STATUS, DXL_LEN_ONE_BYTE, id_list, hw_error_list);
    }
}
