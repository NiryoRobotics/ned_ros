/*
    xdriver.cpp
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

#include "dynamixel_driver/xdriver.hpp"
#include <sstream>

using namespace std;

namespace DynamixelDriver
{
    /**
     * @brief XDriver::XDriver
     * @param portHandler
     * @param packetHandler
     */
    XDriver::XDriver(DxlMotorType_t type, shared_ptr<dynamixel::PortHandler> &portHandler,
                     shared_ptr<dynamixel::PacketHandler> &packetHandler) :
        _type(type),
        _dxlPortHandler(portHandler),
        _dxlPacketHandler(packetHandler)
    {
    }

    /**
     * @brief XDriver::ping
     * @param id
     * @return
     */
    int XDriver::ping(uint8_t id)
    {
        uint8_t dxl_error = 0;

        int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                             id, &dxl_error);

        if (dxl_error != 0)
            result = dxl_error;

        return result;
    }

    /**
     * @brief XDriver::getModelNumber
     * @param id
     * @param dxl_model_number
     * @return
     */
    int XDriver::getModelNumber(uint8_t id, uint16_t *dxl_model_number)
    {
        uint8_t dxl_error = 0;

        int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                             id,
                                             dxl_model_number,
                                             &dxl_error);

        if (0 != dxl_error)
            result = dxl_error;

        return result;
    }

    /**
     * @brief XDriver::scan
     * @param id_list
     * @return
     */
    int XDriver::scan(vector<uint8_t> &id_list)
    {
        return _dxlPacketHandler->broadcastPing(_dxlPortHandler.get(), id_list);
    }

    /**
     * @brief XDriver::reboot
     * @param id
     * @return
     */
    int XDriver::reboot(uint8_t id)
    {
        int result = -1;

        uint8_t dxl_error = 0;
        result = _dxlPacketHandler->reboot(_dxlPortHandler.get(), id, &dxl_error);

        if (0 != dxl_error)
            result = dxl_error;

        return result;
    }

    /**
     * @brief XDriver::str : build a string describing the object. For debug purpose only
     * @return
     */
    string XDriver::str() const
    {
        ostringstream ss;

        ss << "Driver - type " << static_cast<int>(_type) << "\n";
        ss << "packet handler " << (_dxlPacketHandler ? "OK" : "Not Ok") << "\n";
        ss << "port handler " << (_dxlPortHandler ? "OK" : "Not Ok") << "\n";

        return ss.str();
    }

    /*
     *  -----------------   SYNC WRITE   --------------------
     */

    /**
     * @brief XDriver::syncWrite1Byte
     * @param address
     * @param id_list
     * @param data_list
     * @return
     */
    int XDriver::syncWrite1Byte(uint8_t address, const vector<uint8_t> &id_list, const vector<uint32_t> &data_list)
    {
        int dxl_comm_result = -1;
        dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, DXL_LEN_ONE_BYTE);

        if (id_list.size() != data_list.size())
        {
            return LEN_ID_DATA_NOT_SAME;
        }

        if (id_list.size() == 0)
        {
            return COMM_SUCCESS;
        }

        vector<uint8_t>::const_iterator it_id;
        vector<uint32_t>::const_iterator it_data;

        for (it_id = id_list.cbegin(), it_data = data_list.cbegin();
             it_id < id_list.cend() && it_data < data_list.cend();
             it_id++, it_data++)
        {
            uint8_t params[1] = {(uint8_t)(*it_data)};
            if (!groupSyncWrite.addParam(*it_id, params))
            {
                groupSyncWrite.clearParam();
                return GROUP_SYNC_REDONDANT_ID;
            }
        }

        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        return dxl_comm_result;
    }

    /**
     * @brief XDriver::syncWrite2Bytes
     * @param address
     * @param id_list
     * @param data_list
     * @return
     */
    int XDriver::syncWrite2Bytes(uint8_t address, const vector<uint8_t> &id_list, const vector<uint32_t> &data_list)
    {
        int dxl_comm_result = -1;

        dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, DXL_LEN_TWO_BYTES);

        if (id_list.size() != data_list.size())
        {
            return LEN_ID_DATA_NOT_SAME;
        }

        if (id_list.size() == 0)
        {
            return COMM_SUCCESS;
        }

        vector<uint8_t>::const_iterator it_id;
        vector<uint32_t>::const_iterator it_data;

        for (it_id = id_list.cbegin(), it_data = data_list.cbegin();
             it_id < id_list.cend() && it_data < data_list.cend();
             it_id++, it_data++)
        {
            uint8_t params[2] = {DXL_LOBYTE((uint16_t)(*it_data)), DXL_HIBYTE((uint16_t)(*it_data))};
            if (!groupSyncWrite.addParam(*it_id, params))
            {
                groupSyncWrite.clearParam();
                return GROUP_SYNC_REDONDANT_ID;
            }
        }

        dxl_comm_result = groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        return dxl_comm_result;
    }

    /**
     * @brief XDriver::syncWrite4Bytes
     * @param address
     * @param id_list
     * @param data_list
     * @return
     */
    int XDriver::syncWrite4Bytes(uint8_t address, const vector<uint8_t> &id_list, const vector<uint32_t> &data_list)
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

        vector<uint8_t>::const_iterator it_id;
        vector<uint32_t>::const_iterator it_data;

        for (it_id = id_list.cbegin(), it_data = data_list.cbegin();
             it_id < id_list.cend() && it_data < data_list.cend();
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

    /**
     * @brief XDriver::read1Byte
     * @param address
     * @param id
     * @param data
     * @return
     */
    int XDriver::read1Byte(uint8_t address, uint8_t id, uint32_t *data)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint8_t read_data;
        dxl_comm_result = _dxlPacketHandler->read1ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
        (*data) = read_data;

        if (0 != dxl_error)
            dxl_comm_result = dxl_error;

        return dxl_comm_result;
    }

    /**
     * @brief XDriver::read2Bytes
     * @param address
     * @param id
     * @param data
     * @return
     */
    int XDriver::read2Bytes(uint8_t address, uint8_t id, uint32_t *data)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint16_t read_data;
        dxl_comm_result = _dxlPacketHandler->read2ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
        (*data) = read_data;

        if (0 != dxl_error)
            dxl_comm_result = dxl_error;

        return dxl_comm_result;
    }

    /**
     * @brief XDriver::read4Bytes
     * @param address
     * @param id
     * @param data
     * @return
     */
    int XDriver::read4Bytes(uint8_t address, uint8_t id, uint32_t *data)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        uint32_t read_data;
        dxl_comm_result = _dxlPacketHandler->read4ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
        (*data) = read_data;

        if (0 != dxl_error)
            dxl_comm_result = dxl_error;

        return dxl_comm_result;
    }

    /*
     *  -----------------   SYNC READ   --------------------
     */

    /**
     * @brief XDriver::syncRead
     * @param address
     * @param data_len
     * @param id_list
     * @param data_list
     * @return
     */
    int XDriver::syncRead(uint8_t address, uint8_t data_len, const vector<uint8_t> &id_list, vector<uint32_t> &data_list)
    {
        data_list.clear();

        dynamixel::GroupSyncRead groupSyncRead(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, data_len);
        int dxl_comm_result = COMM_TX_FAIL;

        vector<uint8_t>::const_iterator it_id;

        for (it_id = id_list.cbegin(); it_id < id_list.cend(); it_id++)
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

        bool dxl_getdata_result = false;
        for (it_id = id_list.cbegin(); it_id < id_list.cend(); it_id++)
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
     *  -----------------   CUSTOM   --------------------
     */

    /**
     * @brief XDriver::customWrite
     * @param id
     * @param value
     * @param reg_address
     * @param byte_number
     * @return
     */
    int XDriver::customWrite(uint8_t id, uint8_t reg_address, uint32_t value, uint8_t byte_number)
    {
        int dxl_comm_result = COMM_TX_FAIL;

        switch(byte_number) {
            case 1:
                dxl_comm_result = _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id,
                        reg_address, (uint8_t)value);
            break;
            case 2:
                dxl_comm_result = _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(), id,
                        reg_address, (uint16_t)value);
            break;
            case 4:
                dxl_comm_result = _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id,
                        reg_address, (uint32_t)value);
            break;
            default:
                printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
            break;
        }

        return dxl_comm_result;
    }

    /**
     * @brief XDriver::customRead
     * @param id
     * @param value
     * @param reg_address
     * @param byte_number
     * @return
     */
    int XDriver::customRead(uint8_t id, uint8_t reg_address, uint32_t &value, uint8_t byte_number)
    {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

        switch(byte_number) {
            case 1:
            {
                uint8_t read_data;
                dxl_comm_result = _dxlPacketHandler->read1ByteTxRx(_dxlPortHandler.get(), id,
                                            reg_address, &read_data, &dxl_error);
                value = read_data;
            }
            break;
            case 2:
            {
                uint16_t read_data;
                dxl_comm_result = _dxlPacketHandler->read2ByteTxRx(_dxlPortHandler.get(), id,
                                            reg_address, &read_data, &dxl_error);
                value = read_data;
            }
            break;
            case 4:
            {
                uint32_t read_data;
                dxl_comm_result = _dxlPacketHandler->read4ByteTxRx(_dxlPortHandler.get(), id,
                                            reg_address, &read_data, &dxl_error);
                value = read_data;
            }
            break;
            default:
                printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
            break;
        }

        if (0 != dxl_error)
            dxl_comm_result = dxl_error;

        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to get register: %d\n", dxl_comm_result);
        }
        else {
            printf("Retrieved value at address %d : %d\n", reg_address, value);
        }

        return dxl_comm_result;
    }
}
