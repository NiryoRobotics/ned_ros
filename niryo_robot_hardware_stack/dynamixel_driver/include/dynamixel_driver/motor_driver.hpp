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

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common_defs.hpp"
#include "model/motor_type_enum.hpp"

namespace MotorDriver
{
    /**
     * @brief The TTLMotorDriver class
     */
    template<typename reg_type>
    class TTLMotorDriver
    {

    public:
        TTLMotorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                       std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        ~TTLMotorDriver();

        int ping(uint8_t id);
        int getModelNumber(uint8_t id,
                           uint16_t *dxl_model_number);
        int scan(std::vector<uint8_t> &id_list);
        int reboot(uint8_t id);

        std::string str() const;

        std::string interpreteErrorState(uint32_t hw_state);

        //get model number - specific to a child
        int checkModelNumber(uint8_t id);

        // eeprom write
        int changeId(uint8_t id, uint8_t new_id)
        {
            return write(reg_type::ADDR_ID, reg_type::SIZE_ID, id, new_id);
        }

        int changeBaudRate(uint8_t id, uint32_t new_baudrate)
        {
            return write(reg_type::ADDR_BAUDRATE, reg_type::SIZE_BAUDRATE, id, new_baudrate);
        }

        int setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
        {
            return write(reg_type::ADDR_RETURN_DELAY_TIME, reg_type::SIZE_RETURN_DELAY_TIME, id, return_delay_time);
        }

        int setLimitTemperature(uint8_t id, uint32_t temperature)
        {
            return write(reg_type::ADDR_TEMPERATURE_LIMIT, reg_type::SIZE_TEMPERATURE_LIMIT, id, temperature);
        }
        int setMaxTorque(uint8_t id, uint32_t torque)
        {
            return write(reg_type::ADDR_MAX_TORQUE, reg_type::SIZE_MAX_TORQUE, id, torque);
        }
        int setReturnLevel(uint8_t id, uint32_t return_level)
        {
            return write(reg_type::ADDR_RETURN_LEVEL, reg_type::SIZE_RETURN_LEVEL, id, return_level);
        }
        int setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
        {
            return write(reg_type::ADDR_ALARM_SHUTDOWN, reg_type::SIZE_ALARM_SHUTDOWN, id, alarm_shutdown);
        }

        // eeprom read
        int readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
        {
            return read(reg_type::ADDR_RETURN_DELAY_TIME, reg_type::SIZE_RETURN_DELAY_TIME, id, return_delay_time);
        }
        int readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
        {
            return read(reg_type::ADDR_TEMPERATURE_LIMIT, reg_type::SIZE_TEMPERATURE_LIMIT, id, limit_temperature);
        }
        int readMaxTorque(uint8_t id, uint32_t *max_torque)
        {
            return read(reg_type::ADDR_MAX_TORQUE, reg_type::SIZE_MAX_TORQUE, id, max_torque);
        }
        int readReturnLevel(uint8_t id, uint32_t *return_level)
        {
            return read(reg_type::ADDR_RETURN_LEVEL, reg_type::SIZE_RETURN_LEVEL, id, return_level);
        }
        int readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
        {
            return read(reg_type::ADDR_ALARM_SHUTDOWN, reg_type::SIZE_ALARM_SHUTDOWN, id, alarm_shutdown);
        }

        // ram write
        int setTorqueEnable(uint8_t id, uint32_t torque_enable)
        {
            return write(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id, torque_enable);
        }
        int setLed(uint8_t id, uint32_t led_value)
        {
            return write(reg_type::ADDR_LED, reg_type::SIZE_LED, id, led_value);
        }
        int setGoalPosition(uint8_t id, uint32_t position)
        {
            return write(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id, position);
        }
        int setGoalVelocity(uint8_t id, uint32_t velocity)
        {
            return write(reg_type::ADDR_GOAL_SPEED, reg_type::SIZE_GOAL_SPEED, id, velocity);
        }

        int setGoalTorque(uint8_t id, uint32_t torque)
        {
            return write(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id, torque);
        }

        int setPGain(uint8_t id, uint32_t gain)
        {
            return write(reg_type::ADDR_POSITION_P_GAIN, reg_type::SIZE_POSITION_P_GAIN, id, gain);
        }
        int setIGain(uint8_t id, uint32_t gain)
        {
            return write(reg_type::ADDR_POSITION_I_GAIN, reg_type::SIZE_POSITION_I_GAIN, id, gain);
        }
        int setDGain(uint8_t id, uint32_t gain)
        {
            return write(reg_type::ADDR_POSITION_D_GAIN, reg_type::SIZE_POSITION_D_GAIN, id, gain);
        }
        int setff1Gain(uint8_t id, uint32_t gain)
        {
            return write(reg_type::ADDR_FF1_GAIN, reg_type::SIZE_FF1_GAIN, id, gain);
        }
        int setff2Gain(uint8_t id, uint32_t gain)
        {
            return write(reg_type::ADDR_FF2_GAIN, reg_type::SIZE_FF2_GAIN, id, gain);
        }

        int syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list)
        {
            return syncWrite(reg_type::ADDR_LED, reg_type::SIZE_LED, id_list, led_list);
        }
        int syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
        {
            return syncWrite(reg_type::ADDR_TORQUE_ENABLE, reg_type::SIZE_TORQUE_ENABLE, id_list, torque_enable_list);
        }
        int syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
        {
            return syncWrite(reg_type::ADDR_GOAL_POSITION, reg_type::SIZE_GOAL_POSITION, id_list, position_list);
        }
        int syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
        {
            return syncWrite(reg_type::ADDR_GOAL_SPEED, reg_type::SIZE_GOAL_SPEED, id_list, velocity_list);
        }
        int syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
        {
            return syncWrite(reg_type::ADDR_GOAL_TORQUE, reg_type::SIZE_GOAL_TORQUE, id_list, torque_list);
        }

        // ram read
        int readPosition(uint8_t id, uint32_t *present_position)
        {
            return read(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id, present_position);
        }
        int readVelocity(uint8_t id, uint32_t *present_velocity)
        {
            return read(reg_type::ADDR_PRESENT_SPEED, reg_type::SIZE_PRESENT_SPEED, id, present_velocity);
        }
        int readLoad(uint8_t id, uint32_t *present_load)
        {
            return read(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id, present_load);
        }

        int readTemperature(uint8_t id, uint32_t *temperature)
        {
            return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
        }
        int readVoltage(uint8_t id, uint32_t *voltage)
        {
            return read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage);
        }
        int readHardwareStatus(uint8_t id, uint32_t *hardware_status)
        {
            return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
        }

        int syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
        {
            return syncRead(reg_type::ADDR_PRESENT_POSITION, reg_type::SIZE_PRESENT_POSITION, id_list, position_list);
        }
        int syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
        {
            return syncRead(reg_type::ADDR_PRESENT_SPEED, reg_type::SIZE_PRESENT_SPEED, id_list, velocity_list);
        }
        int syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
        {
            return syncRead(reg_type::ADDR_PRESENT_LOAD, reg_type::SIZE_PRESENT_LOAD, id_list, load_list);
        }
        int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
        {
            return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
        }
        int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
        {
            return syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, voltage_list);
        }
        int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
        {
            return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
        }

        // custom write and read
        int read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t *data);
        int write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data);

    protected:
        int syncRead(uint8_t address, uint8_t data_len, const std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
        int syncWrite(uint8_t address, uint8_t data_len, const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &data_list);

        static constexpr int PING_WRONG_MODEL_NUMBER = 30;

    private:
        std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
        std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

        static constexpr uint8_t DXL_LEN_ONE_BYTE    = 1;
        static constexpr uint8_t DXL_LEN_TWO_BYTES   = 2;
        static constexpr uint8_t DXL_LEN_FOUR_BYTES  = 4;

        static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
        static constexpr int GROUP_SYNC_READ_RX_FAIL = 11;
        static constexpr int LEN_ID_DATA_NOT_SAME    = 20;
    };

    // definition of methods

    /**
     * @brief TTLMotorDriver<reg_type>::TTLMotorDriver
     * @param type
     * @param portHandler
     * @param packetHandler
     */
    template<typename reg_type>
    TTLMotorDriver<reg_type>::TTLMotorDriver(
                     std::shared_ptr<dynamixel::PortHandler> portHandler,
                     std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
        _dxlPortHandler(portHandler),
        _dxlPacketHandler(packetHandler)
    {
    }

    /**
     * @brief TTLMotorDriver<reg_type>::~TTLMotorDriver
     */
    template<typename reg_type>
    TTLMotorDriver<reg_type>::~TTLMotorDriver()
    {

    }

    /**
     * @brief TTLMotorDriver<reg_type>::ping
     * @param id
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::ping(uint8_t id)
    {
        uint8_t dxl_error = 0;

        int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                             id, &dxl_error);

        if (dxl_error != 0)
            result = dxl_error;

        return result;
    }

    /**
     * @brief TTLMotorDriver<reg_type>::getModelNumber
     * @param id
     * @param dxl_model_number
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::getModelNumber(uint8_t id, uint16_t *dxl_model_number)
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
     * @brief TTLMotorDriver<reg_type>::scan
     * @param id_list
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::scan(std::vector<uint8_t> &id_list)
    {
        return _dxlPacketHandler->broadcastPing(_dxlPortHandler.get(), id_list);
    }

    /**
     * @brief TTLMotorDriver<reg_type>::reboot
     * @param id
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::reboot(uint8_t id)
    {
        int result = -1;

        uint8_t dxl_error = 0;
        result = _dxlPacketHandler->reboot(_dxlPortHandler.get(), id, &dxl_error);

        if (0 != dxl_error)
            result = dxl_error;

        return result;
    }

    /**
     * @brief TTLMotorDriver<reg_type>::str : build a string describing the object. For debug purpose only
     * @return
     */
    template<typename reg_type>
    std::string TTLMotorDriver<reg_type>::str() const
    {
        std::ostringstream ss;

        ss << "Driver - type " << static_cast<int>(reg_type::motor_type) << "\n";
        ss << "packet handler " << (_dxlPacketHandler ? "OK" : "Not Ok") << "\n";
        ss << "port handler " << (_dxlPortHandler ? "OK" : "Not Ok") << "\n";

        return ss.str();
    }

    /*
     *  -----------------   Read Write operations   --------------------
     */

    /**
     * @brief TTLMotorDriver<reg_type>::read
     * @param address
     * @param data_len
     * @param id
     * @param data
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t *data)
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        switch(data_len)
        {
            case DXL_LEN_ONE_BYTE:
            {
                uint8_t read_data;
                dxl_comm_result = _dxlPacketHandler->read1ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
                (*data) = read_data;
            }
            break;
            case DXL_LEN_TWO_BYTES:
            {
                uint16_t read_data;
                dxl_comm_result = _dxlPacketHandler->read2ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
                (*data) = read_data;
            }
            break;
            case DXL_LEN_FOUR_BYTES:
            {
                uint32_t read_data;
                dxl_comm_result = _dxlPacketHandler->read4ByteTxRx(_dxlPortHandler.get(), id, address, &read_data, &dxl_error);
                (*data) = read_data;
            }
            break;
            default:
                printf("TTLMotorDriver::read ERROR: Size param must be 1, 2 or 4 bytes\n");
            break;
        }

        if (0 != dxl_error)
            dxl_comm_result = dxl_error;

        return dxl_comm_result;
    }


    /**
     * @brief TTLMotorDriver<reg_type>::write
     * @param address
     * @param data_len
     * @param id
     * @param data
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data)
    {
        int dxl_comm_result = COMM_TX_FAIL;

        switch(data_len)
        {
            case DXL_LEN_ONE_BYTE:
                dxl_comm_result = _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(), id, address, static_cast<uint8_t>(data));
            break;
            case DXL_LEN_TWO_BYTES:
                dxl_comm_result = _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(), id, address, static_cast<uint16_t>(data));
            break;
            case DXL_LEN_FOUR_BYTES:
                dxl_comm_result = _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(), id, address, data);
            break;
            default:
                printf("TTLMotorDriver::write ERROR: Size param must be 1, 2 or 4 bytes\n");
            break;
        }

        return dxl_comm_result;
    }

    /**
     * @brief TTLMotorDriver<reg_type>::syncRead
     * @param address
     * @param data_len
     * @param id_list
     * @param data_list
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::syncRead(uint8_t address, uint8_t data_len, const std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
    {
        data_list.clear();

        dynamixel::GroupSyncRead groupSyncRead(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, data_len);
        int dxl_comm_result = COMM_TX_FAIL;

        for (auto const& id : id_list)
        {
            if (!groupSyncRead.addParam(id))
            {
                groupSyncRead.clearParam();
                return GROUP_SYNC_REDONDANT_ID;
            }
        }

        dxl_comm_result = groupSyncRead.txRxPacket();

        if (COMM_SUCCESS == dxl_comm_result)
        {
            for (auto const& id : id_list)
            {
                if (groupSyncRead.isAvailable(id, address, data_len))
                {
                    switch(data_len)
                    {
                        case DXL_LEN_ONE_BYTE:
                            data_list.emplace_back(static_cast<uint8_t>(groupSyncRead.getData(id, address, data_len)));
                        break;
                        case DXL_LEN_TWO_BYTES:
                            data_list.emplace_back(static_cast<uint16_t>(groupSyncRead.getData(id, address, data_len)));
                        break;
                        case DXL_LEN_FOUR_BYTES:
                            data_list.emplace_back(groupSyncRead.getData(id, address, data_len));
                        break;
                        default:
                            printf("TTLMotorDriver::syncRead ERROR: Size param must be 1, 2 or 4 bytes\n");
                        break;
                    }
                }
                else
                {
                    dxl_comm_result = GROUP_SYNC_READ_RX_FAIL;
                    break;
                }
            }
        }

        groupSyncRead.clearParam();

        return dxl_comm_result;
    }

    /**
     * @brief TTLMotorDriver<reg_type>::syncWrite
     * @param address
     * @param data_len
     * @param id_list
     * @param data_list
     * @return
     */
    template<typename reg_type>
    int TTLMotorDriver<reg_type>::syncWrite(uint8_t address, uint8_t data_len, const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &data_list)
    {
        int dxl_comm_result = COMM_SUCCESS;

        if (!id_list.empty())
        {
            if (id_list.size() == data_list.size())
            {
                dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(),
                                                         _dxlPacketHandler.get(),
                                                         address,
                                                         data_len);

                bool dxl_senddata_result = false;

                for (size_t i = 0; i < id_list.size(); ++i)
                {
                    uint8_t id = id_list.at(i);
                    uint32_t data = data_list.at(i);

                    switch(data_len)
                    {
                        case DXL_LEN_ONE_BYTE:
                        {
                            uint8_t params[1] = {static_cast<uint8_t>(data)};
                            dxl_senddata_result = groupSyncWrite.addParam(id, params);
                        }
                        break;
                        case DXL_LEN_TWO_BYTES:
                        {
                            uint8_t params[2] = {DXL_LOBYTE(static_cast<uint16_t>(data)),
                                                 DXL_HIBYTE(static_cast<uint16_t>(data))};
                            dxl_senddata_result = groupSyncWrite.addParam(id, params);
                        }
                        break;
                        case DXL_LEN_FOUR_BYTES:
                        {
                            uint8_t params[4] = {DXL_LOBYTE(DXL_LOWORD(data)),
                                                 DXL_HIBYTE(DXL_LOWORD(data)),
                                                 DXL_LOBYTE(DXL_HIWORD(data)),
                                                 DXL_HIBYTE(DXL_HIWORD(data))};
                            dxl_senddata_result = groupSyncWrite.addParam(id, params);
                        }
                        break;
                        default:
                            printf("TTLMotorDriver::syncWrite ERROR: Size param must be 1, 2 or 4 bytes\n");
                        break;
                    }

                    if (!dxl_senddata_result)
                    {
                        dxl_comm_result = GROUP_SYNC_REDONDANT_ID;
                        break;
                    }
                }

                //send group if no error
                if(GROUP_SYNC_REDONDANT_ID != dxl_comm_result)
                    dxl_comm_result = groupSyncWrite.txPacket();

                groupSyncWrite.clearParam();
            }
            else {
                dxl_comm_result = LEN_ID_DATA_NOT_SAME;
            }
        }

        return dxl_comm_result;
    }

} //DynamixelDriver

#endif
