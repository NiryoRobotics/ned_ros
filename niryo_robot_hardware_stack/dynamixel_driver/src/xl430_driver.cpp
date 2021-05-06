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

XL430Driver::XL430Driver(boost::shared_ptr<dynamixel::PortHandler> &portHandler, boost::shared_ptr<dynamixel::PacketHandler> &packetHandler)
    : _dxlPortHandler(portHandler), _dxlPacketHandler(packetHandler)
{
}

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

int XL430Driver::ping(uint8_t id)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(), id, &dxl_error);

    if (dxl_error != 0)
    {
        return dxl_error;
    }

    return result;
}

int XL430Driver::getModelNumber(uint8_t id, uint16_t *dxl_model_number)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(), id, dxl_model_number, &dxl_error);

    if (dxl_error != 0)
    {
        return dxl_error;
    }

    return result;
}

int XL430Driver::scan(std::vector<uint8_t> &id_list)
{
    return _dxlPacketHandler->broadcastPing(_dxlPortHandler.get(), id_list);
}

int XL430Driver::reboot(uint8_t id)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->reboot(_dxlPortHandler.get(), id, &dxl_error);

    if (dxl_error != 0)
    {
        return dxl_error;
    }

    return result;
}

/*
 *  -----------------   SYNC WRITE   --------------------
 */

int XL430Driver::syncWrite1Byte(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, DXL_LEN_ONE_BYTE);

    if (id_list.size() != data_list.size())
    {
        return LEN_ID_DATA_NOT_SAME;
    }

    if (id_list.size() == 0)
    {
        return COMM_SUCCESS;
    }

    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id = id_list.begin(), it_data = data_list.begin();
         it_id < id_list.end() && it_data < data_list.end();
         it_id++, it_data++)
    {
        uint8_t params[1] = {(uint8_t)(*it_data)};
        if (!groupSyncWrite.addParam(*it_id, params))
        {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }

    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

int XL430Driver::syncWrite2Bytes(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, DXL_LEN_TWO_BYTES);

    if (id_list.size() != data_list.size())
    {
        return LEN_ID_DATA_NOT_SAME;
    }

    if (id_list.size() == 0)
    {
        return COMM_SUCCESS;
    }

    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id = id_list.begin(), it_data = data_list.begin();
         it_id < id_list.end() && it_data < data_list.end();
         it_id++, it_data++)
    {
        uint8_t params[2] = {DXL_LOBYTE((uint16_t)(*it_data)), DXL_HIBYTE((uint16_t)(*it_data))};
        if (!groupSyncWrite.addParam(*it_id, params))
        {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }

    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

int XL430Driver::syncWrite4Bytes(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, DXL_LEN_FOUR_BYTES);

    if (id_list.size() != data_list.size())
    {
        return LEN_ID_DATA_NOT_SAME;
    }

    if (id_list.size() == 0)
    {
        return COMM_SUCCESS;
    }

    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id = id_list.begin(), it_data = data_list.begin();
         it_id < id_list.end() && it_data < data_list.end();
         it_id++, it_data++)
    {
        uint8_t params[4] = {DXL_LOBYTE(DXL_LOWORD(*it_data)), DXL_HIBYTE(DXL_LOWORD(*it_data)),
                             DXL_LOBYTE(DXL_HIWORD(*it_data)), DXL_HIBYTE(DXL_HIWORD(*it_data))};
        if (!groupSyncWrite.addParam(*it_id, params))
        {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }

    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

/*
 *  -----------------   READ   --------------------
 */

int XL430Driver::read1Byte(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t read_data;
    dxl_comm_result = _dxlPacketHandler->read1ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0)
    {
        return dxl_error;
    }

    return dxl_comm_result;
}

int XL430Driver::read2Bytes(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint16_t read_data;
    dxl_comm_result = _dxlPacketHandler->read2ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0)
    {
        return dxl_error;
    }

    return dxl_comm_result;
}

int XL430Driver::read4Bytes(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint32_t read_data;
    dxl_comm_result = _dxlPacketHandler->read4ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0)
    {
        return dxl_error;
    }

    return dxl_comm_result;
}

/*
 *  -----------------   SYNC READ   --------------------
 */

int XL430Driver::syncRead(uint8_t address, uint8_t data_len, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    data_list.clear();

    dynamixel::GroupSyncRead groupSyncRead(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, data_len);
    bool dxl_getdata_result = false;
    int dxl_comm_result = COMM_TX_FAIL;

    std::vector<uint8_t>::iterator it_id;

    for (it_id = id_list.begin(); it_id < id_list.end(); it_id++)
    {
        if (!groupSyncRead.addParam(*it_id))
        {
            groupSyncRead.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }

    dxl_comm_result = groupSyncRead.txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS)
    {
        groupSyncRead.clearParam();
        return dxl_comm_result;
    }

    for (it_id = id_list.begin(); it_id < id_list.end(); it_id++)
    {
        dxl_getdata_result = groupSyncRead.isAvailable(*it_id, address, data_len);
        if (!dxl_getdata_result)
        {
            groupSyncRead.clearParam();
            return GROUP_SYNC_READ_RX_FAIL;
        }
        if (data_len == DXL_LEN_ONE_BYTE)
        {
            data_list.push_back((uint8_t)groupSyncRead.getData(*it_id, address, data_len));
        }
        else if (data_len == DXL_LEN_TWO_BYTES)
        {
            data_list.push_back((uint16_t)groupSyncRead.getData(*it_id, address, data_len));
        }
        else if (data_len == DXL_LEN_FOUR_BYTES)
        {
            data_list.push_back((uint32_t)groupSyncRead.getData(*it_id, address, data_len));
        }
    }

    groupSyncRead.clearParam();
    return dxl_comm_result;
}
/*
 *  -----------------   WRITE   --------------------
 */

int XL430Driver::changeId(uint8_t id, uint8_t new_id)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_ID, new_id);
}

int XL430Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_BAUDRATE, (uint8_t)new_baudrate);
}

int XL430Driver::setLed(uint8_t id, uint32_t led_value)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_LED, (uint8_t)led_value);
}

int XL430Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_TORQUE_ENABLE, (uint8_t)torque_enable);
}

int XL430Driver::setGoalPosition(uint8_t id, uint32_t position)
{
    return _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_GOAL_POSITION, position);
}

int XL430Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    return _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_GOAL_VELOCITY, velocity);
}

int XL430Driver::setGoalTorque(uint8_t id, uint32_t torque)
{
    (void)id;
    (void)torque;
    // No goal torque for this motor ?
    return COMM_TX_ERROR;
}

int XL430Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_RETURN_DELAY_TIME, (uint8_t)return_delay_time);
}

int XL430Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_TEMPERATURE_LIMIT, (uint8_t)temperature);
}

int XL430Driver::setMaxTorque(uint8_t id, uint32_t torque)
{
    (void)id;
    (void)torque;
    // No max torque setting for this motor ?
    return COMM_TX_ERROR;
}

int XL430Driver::setReturnLevel(uint8_t id, uint32_t return_level)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_STATUS_RETURN_LEVEL, (uint8_t)return_level);
}

int XL430Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
{
    return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, XL430_ADDR_ALARM_SHUTDOWN, (uint8_t)alarm_shutdown);
}

int XL430Driver::customWrite(uint8_t id, uint32_t value, uint8_t reg_address, uint8_t byte_number)
{
    if (byte_number == 1)
    {
        return _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, reg_address, (uint8_t)value);
    }
    else if (byte_number == 2)
    {
        return _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(), id, reg_address, (uint16_t)value);
    }
    else if (byte_number == 4)
    {
        return _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id, reg_address, value);
    }
    else
    {
        return -1;
    }
}

/*
 *  -----------------   SYNC WRITE   --------------------
 */

int XL430Driver::syncWritePositionGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    return syncWrite4Bytes(XL430_ADDR_GOAL_POSITION, id_list, position_list);
}
int XL430Driver::syncWriteVelocityGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    return syncWrite4Bytes(XL430_ADDR_GOAL_VELOCITY, id_list, velocity_list);
}
int XL430Driver::syncWriteTorqueGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list)
{
    (void)id_list;
    (void)torque_list;
    // No goal torque for this motor ?
    return COMM_TX_ERROR;
}

int XL430Driver::syncWriteTorqueEnable(std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list)
{
    return syncWrite1Byte(XL430_ADDR_TORQUE_ENABLE, id_list, torque_enable_list);
}

int XL430Driver::syncWriteLed(std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list)
{
    return syncWrite1Byte(XL430_ADDR_LED, id_list, led_list);
}

/*
 *  -----------------   READ   --------------------
 */

int XL430Driver::readPosition(uint8_t id, uint32_t *present_position)
{
    return read4Bytes(XL430_ADDR_PRESENT_POSITION, id, present_position);
}

int XL430Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
{
    return read4Bytes(XL430_ADDR_PRESENT_VELOCITY, id, present_velocity);
}

int XL430Driver::readLoad(uint8_t id, uint32_t *present_load)
{
    return read2Bytes(XL430_ADDR_PRESENT_LOAD, id, present_load);
}

int XL430Driver::readTemperature(uint8_t id, uint32_t *temperature)
{
    return read1Byte(XL430_ADDR_PRESENT_TEMPERATURE, id, temperature);
}

int XL430Driver::readVoltage(uint8_t id, uint32_t *voltage)
{
    return read2Bytes(XL430_ADDR_PRESENT_VOLTAGE, id, voltage);
}

int XL430Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
{
    return read1Byte(XL430_ADDR_HW_ERROR_STATUS, id, hardware_status);
}
int XL430Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
{
    return read1Byte(XL430_ADDR_RETURN_DELAY_TIME, id, return_delay_time);
}

int XL430Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
{
    return read1Byte(XL430_ADDR_TEMPERATURE_LIMIT, id, limit_temperature);
}

int XL430Driver::readMaxTorque(uint8_t id, uint32_t *max_torque)
{
    (void)id;
    (void)max_torque;
    // No max torque setting for this motor ?
    return COMM_TX_ERROR;
}

int XL430Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
{
    return read1Byte(XL430_ADDR_STATUS_RETURN_LEVEL, id, return_level);
}

int XL430Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
{
    return read1Byte(XL430_ADDR_ALARM_SHUTDOWN, id, alarm_shutdown);
}

/*
 *  -----------------   SYNC READ   --------------------
 */

int XL430Driver::syncReadPosition(std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    return syncRead(XL430_ADDR_PRESENT_POSITION, DXL_LEN_FOUR_BYTES, id_list, position_list);
}

int XL430Driver::syncReadVelocity(std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    return syncRead(XL430_ADDR_PRESENT_VELOCITY, DXL_LEN_FOUR_BYTES, id_list, velocity_list);
}
int XL430Driver::syncReadLoad(std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    return syncRead(XL430_ADDR_PRESENT_LOAD, DXL_LEN_TWO_BYTES, id_list, load_list);
}

int XL430Driver::syncReadTemperature(std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(XL430_ADDR_PRESENT_TEMPERATURE, DXL_LEN_ONE_BYTE, id_list, temperature_list);
}

int XL430Driver::syncReadVoltage(std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    return syncRead(XL430_ADDR_PRESENT_VOLTAGE, DXL_LEN_TWO_BYTES, id_list, voltage_list);
}

int XL430Driver::syncReadHwErrorStatus(std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    return syncRead(XL430_ADDR_HW_ERROR_STATUS, DXL_LEN_ONE_BYTE, id_list, hw_error_list);
}
