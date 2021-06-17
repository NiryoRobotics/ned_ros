/*
    ttl_driver.hpp
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

#ifndef TTL_DRIVER_HPP
#define TTL_DRIVER_HPP

#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <algorithm>

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "ttl_driver/DxlMotorCommand.h"
#include "niryo_robot_msgs/MotorHeader.h"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include "common/model/idriver.hpp"

//drivers
#include "ttl_driver/abstract_motor_driver.hpp"
#include "common/model/dxl_motor_state.hpp"
#include "common/model/synchronize_motor_cmd.hpp"
#include "common/model/single_motor_cmd.hpp"

namespace ttl_driver
{

    constexpr float DXL_BUS_PROTOCOL_VERSION = 2.0;
    constexpr int DXL_FAIL_OPEN_PORT         = -4500;

    constexpr int DXL_FAIL_PORT_SET_BAUDRATE = -4501;
    constexpr int DXL_FAIL_SETUP_GPIO        = -4502;

    constexpr int DXL_SCAN_OK                = 0;
    constexpr int DXL_SCAN_MISSING_MOTOR     = -50;
    constexpr int DXL_SCAN_UNALLOWED_MOTOR   = -51;
    constexpr int DXL_WRONG_TYPE             = -52;

    /**
     * @brief The TtlDriver class
     */
    class TtlDriver : public common::model::IDriver
    {
        public:
            TtlDriver();
            virtual ~TtlDriver() override;

            //commands
            void addMotor(common::model::EMotorType type,
                          uint8_t id, bool isTool = false);

            int readSynchronizeCommand(common::model::SynchronizeMotorCmd cmd);
            int readSingleCommand(common::model::SingleMotorCmd cmd);
            void executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, uint32_t> > cmd_vec);

            int rebootMotors();

            int setLeds(int led, common::model::EMotorType type);

            int sendCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, int reg_address, int value, int byte_number);
            int readCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, int32_t reg_address, int &value, int byte_number);

            void readPositionStatus();
            void readHwStatus();

            int getAllIdsOnBus(std::vector<uint8_t> &id_list);

            //getters
            uint32_t getPosition(common::model::DxlMotorState& motor_state);
            int getLedState() const;

            std::vector<std::shared_ptr<common::model::DxlMotorState> > getMotorsStates() const;
            common::model::DxlMotorState getMotorState(uint8_t motor_id) const;

            std::vector<uint8_t> getRemovedMotorList() const;

            // IDriver Interface
            void removeMotor(uint8_t id) override;
            bool isConnectionOk() const override;

            int scanAndCheck() override;
            bool ping(uint8_t id) override;

            size_t getNbMotors() const override;
            void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const override;
            std::string getErrorMessage() const override;

        private:
            bool init() override;
            bool hasMotors() override;

            int setupCommunication();

            void checkRemovedMotors();

            int _syncWrite(int (AbstractMotorDriver::*syncWriteFunction)(const std::vector<uint8_t> &, const std::vector<uint32_t> &),
                                  const common::model::SynchronizeMotorCmd& cmd);

            int _singleWrite(int (AbstractMotorDriver::*singleWriteFunction)(uint8_t id, uint32_t), common::model::EMotorType dxl_type,
                                  const common::model::SingleMotorCmd& cmd);
        private:
            ros::NodeHandle _nh;

            std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
            std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

            std::string _device_name;
            int _uart_baudrate;

            std::vector<uint8_t> _all_motor_connected; //with all dxl motors connected (including the tool)
            std::vector<uint8_t> _removed_motor_id_list;

            std::map<uint8_t, std::shared_ptr<common::model::DxlMotorState> > _state_map;
            std::map<common::model::EMotorType, std::vector<uint8_t> > _ids_map;
            std::map<common::model::EMotorType, std::shared_ptr<AbstractMotorDriver> > _xdriver_map;

            // for hardware control
            bool _is_connection_ok;
            std::string _debug_error_message;

            int _hw_fail_counter_read;

            int _led_state;

            static constexpr int MAX_HW_FAILURE = 25;

    };

    //inline getters

    inline
    bool TtlDriver::isConnectionOk() const
    {
        return _is_connection_ok;
    }

    /**
     * @brief TtlDriver::getNbMotors
     * @return
     */
    inline
    size_t TtlDriver::getNbMotors() const
    {
        return _state_map.size();
    }

    /**
     * @brief TtlDriver::getRemovedMotorList
     * @return
     */
    inline
    std::vector<uint8_t> TtlDriver::getRemovedMotorList() const
    {
        return _removed_motor_id_list;
    }

    /**
     * @brief TtlDriver::getErrorMessage
     * @return
     */
    inline
    std::string TtlDriver::getErrorMessage() const
    {
        return _debug_error_message;
    }

    /**
     * @brief TtlDriver::getLedState
     * @return
     */
    inline
    int TtlDriver::getLedState() const
    {
        return _led_state;
    }

    /**
     * @brief TtlDriver::getBusState
     * @param connection_state
     * @param motor_id
     * @param debug_msg
     */
    inline
    void TtlDriver::getBusState(bool &connection_state, std::vector<uint8_t> &motor_id,
                                std::string &debug_msg) const
    {
        debug_msg = _debug_error_message;
        motor_id = _all_motor_connected;
        connection_state = isConnectionOk();
    }

    /**
     * @brief TtlDriver::hasMotors
     * @return
     */
    inline
    bool TtlDriver::hasMotors()
    {
        return _state_map.size() > 0;
    }

}

#endif // TTLDRIVER_HPP
