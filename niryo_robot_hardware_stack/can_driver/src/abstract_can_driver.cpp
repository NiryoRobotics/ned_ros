/*
    abstract_can_driver.cpp
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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include "can_driver/abstract_can_driver.hpp"

#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using ::std::ostringstream;
using ::std::shared_ptr;
using ::std::string;
using ::std::vector;

namespace can_driver
{

/**
 * @brief AbstractCanDriver::AbstractCanDriver
 * @param mcp_can
 */
AbstractCanDriver::AbstractCanDriver(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can) : _mcp_can(std::move(mcp_can)) {}

/**
 * @brief StepperDriver::ping
 * @param id
 * @return
 */
int AbstractCanDriver::ping(uint8_t id)
{
    int res = CAN_FAIL;

    double time_begin_scan = ros::Time::now().toSec();

    while (ros::Time::now().toSec() - time_begin_scan < PING_TIME_OUT)
    {
        ros::Duration(0.001).sleep();  // check at 1000 Hz
        if (canReadData())
        {
            INT32U rxId;
            uint8_t len;
            std::array<uint8_t, 8> rxBuf{};
            read(&rxId, &len, rxBuf);
            uint8_t motor_id = rxId & 0x0F;

            if (motor_id == id)
            {
                res = CAN_OK;
                break;
            }
        }
    }

    return res;
}

/**
 * @brief AbstractCanDriver::scan : try to find "motors_to_find" list of motors for a given time
 * @param motors_unfound
 * @param id_list
 * @return
 */
int AbstractCanDriver::scan(std::set<uint8_t> &motors_unfound, std::vector<uint8_t> &id_list)
{
    int result = CAN_FAIL;

    id_list.clear();

    double time_begin_scan = ros::Time::now().toSec();
    double timeout = 0.5;

    while ((!motors_unfound.empty()) && (ros::Time::now().toSec() - time_begin_scan < timeout))
    {
        ros::Duration(0.001).sleep();  // check at 1000 Hz
        if (canReadData())
        {
            INT32U rxId;
            uint8_t len;
            std::array<uint8_t, 8> rxBuf{};
            read(&rxId, &len, rxBuf);
            uint8_t motor_id = rxId & 0x0F;

            if (motors_unfound.count(motor_id))
            {
                motors_unfound.erase(motor_id);
                id_list.emplace_back(motor_id);
            }
        }
    }

    // if found everything
    if (motors_unfound.empty())
        result = CAN_OK;

    return result;
}

/**
 * @brief AbstractTtlDriver::str : build a string describing the object. For debug purpose only
 * @return
 */
std::string AbstractCanDriver::str() const
{
    ostringstream ss;

    ss << "CAN Driver : "
       << "packet handler " << (_mcp_can ? "OK" : "Not Ok");

    return ss.str();
}

/*
 *  -----------------   Read Write operations   --------------------
 */

/**
 * @brief AbstractCanDriver::readData
 * @param id
 * @param control_byte
 * @param rxBuf
 * @param error_message
 * @return
 */
uint8_t AbstractCanDriver::readData(uint8_t &id, int &control_byte, std::array<uint8_t, MAX_MESSAGE_LENGTH> &rxBuf, std::string &error_message)
{
    uint8_t res = CAN_FAIL;

    error_message.clear();
    INT32U rxId;
    uint8_t len{};
    read(&rxId, &len, rxBuf);
    id = rxId & 0x0F;
    control_byte = rxBuf[0];
    if (MESSAGE_LENGTH == len)
    {
        res = CAN_OK;
    }
    else
    {
        error_message = "invalid frame size (" + std::to_string(len) + " bytes received)";
    }
    return res;
}

/**
 * @brief AbstractCanDriver::read
 * @param id
 * @param len
 * @param buf
 * @return
 */
uint8_t AbstractCanDriver::read(INT32U *id, uint8_t *len, std::array<uint8_t, MAX_MESSAGE_LENGTH> &buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = _mcp_can->readMsgBuf(id, len, buf.data());
        if (CAN_OK != status)
            ROS_WARN_THROTTLE(1.0, "StepperDriver::read - Reading Stepper message on CAN Bus failed");
    }

    return status;
}

/**
 * @brief AbstractCanDriver::write
 * @param id
 * @param ext
 * @param len
 * @param buf
 * @return
 */
uint8_t AbstractCanDriver::write(uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
    uint8_t status = CAN_FAIL;

    for (auto i = 0; i < 10 && CAN_OK != status; ++i)
    {
        status = _mcp_can->sendMsgBuf(id, ext, len, buf);
        ROS_WARN_COND(CAN_OK != status, "StepperDriver::write - Sending Stepper message on CAN Bus failed");
    }

    return status;
}

}  // namespace can_driver
