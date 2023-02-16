/*
    can_tools.cpp
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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "can_debug_tools/can_tools.hpp"

namespace can_debug_tools
{

/**
 * @brief CanTools::CanTools
 * @param mcp_can
 */
CanTools::CanTools(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can) : _mcp_can(std::move(mcp_can)) {}

CanTools::~CanTools()
{
    _control_loop_ok = false;

    if (_control_loop_thread.joinable())
        _control_loop_thread.join();
}

/**
 * @brief CanTools::setupCommunication
 * @return
 */
int CanTools::setupCommunication()
{
    int ret = CAN_FAILINIT;

    // Can bus setup
    if (_mcp_can)
    {
        if (_mcp_can->setupInterruptGpio())
        {
            std::cout << "CanTools::setupCommunication - Setup Interrupt GPIO successfull" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if (_mcp_can->setupSpi())
            {
                std::cout << "CanTools::setupCommunication - Setup SPI successfull" << std::endl;

                std::this_thread::sleep_for(std::chrono::milliseconds(50));

                // no mask or filter used, receive all messages from CAN bus
                // messages with ids != motor_id will be sent to another ROS interface
                // so we can use many CAN devices with this only driver
                ret = _mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ);

                if (CAN_OK == ret)
                {
                    std::cout << "CanTools::setupCommunication - MCP can initialized" << std::endl;

                    // set mode to normal
                    _mcp_can->setMode(MCP_NORMAL);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                else
                {
                    std::cout << "CanTools::setupCommunication - Failed to init MCP2515 (CAN bus)" << std::endl;
                }
            }
            else
            {
                std::cout << "CanTools::setupCommunication - Failed to start spi" << std::endl;
                ret = CAN_SPI_FAILINIT;
            }
        }
        else
        {
            std::cout << "CanTools::setupCommunication - Failed to start gpio" << std::endl;
            ret = CAN_GPIO_FAILINIT;
        }
    }
    else
        std::cout << "CanTools::setupCommunication - Invalid CAN handler" << std::endl;

    return ret;
}

/**
 * @brief CanTools::startDump
 * @return
 */
void CanTools::startDump(double check_data_freq)
{
    _check_data_delay_ms = 1000 * static_cast<int>(1.0 / check_data_freq);
    _control_loop_thread = std::thread(&CanTools::controlLoop, this);
}

/**
 * @brief can_debug_tools::CanTools::controlLoop
 */
void can_debug_tools::CanTools::controlLoop()
{
    std::cout << "no "
              << ":\t"
              << "status "
              << "|\t"
              << "id "
              << "|\t"
              << "control_byte"
              << "|\t"
              << "[0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07]" << std::endl;

    for (int i = 0; _control_loop_ok; ++i)
    {
        if (_mcp_can->canReadData())
        {
            std::cout << std::setfill('0') << std::setw(sizeof(int) * 2) << i << ":\t" << dumpData() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(_check_data_delay_ms));
    }
}

/**
 * @brief can_debug_tools::CanTools::dumpData
 * @return
 */
std::string CanTools::dumpData()
{
    INT32U rxId{};
    uint8_t len{};
    std::array<uint8_t, 8> rxBuf{};

    uint8_t status = read(&rxId, &len, rxBuf);
    uint8_t id = rxId & 0x0F;
    int control_byte = rxBuf[0];

    std::ostringstream ss;
    ss << std::to_string(status) << "\t|" << std::to_string(id) << "\t|" << control_byte << "\t"
       << "[";

    for (auto const &d : rxBuf)
    {
        ss << "0x" << std::setfill('0') << std::setw(sizeof(uint8_t) * 2) << std::uppercase << std::hex << static_cast<int>(d) << ",";
    }

    std::string dump_data = ss.str();
    dump_data.pop_back();
    dump_data += "]";

    return dump_data;
}

/**
 * @brief CanTools::read
 * @param id
 * @param len
 * @param buf
 * @return
 */
uint8_t CanTools::read(INT32U *id, uint8_t *len, std::array<uint8_t, MAX_MESSAGE_LENGTH> &buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = _mcp_can->readMsgBuf(id, len, buf.data());
    }

    return status;
}

}  // namespace can_debug_tools
