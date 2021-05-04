/*
    dxl_driver.hpp
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

#ifndef DXL_DRIVER_HPP
#define DXL_DRIVER_HPP

#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <algorithm>

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dynamixel_driver/DxlMotorCommand.h"
#include "niryo_robot_msgs/MotorHeader.h"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include "model/idriver.hpp"

//drivers
#include "dynamixel_driver/xl320_driver.hpp"
#include "dynamixel_driver/xl330_driver.hpp"
#include "dynamixel_driver/xl430_driver.hpp"
#include "dynamixel_driver/xc430_driver.hpp"

#include "model/dxl_motor_state.hpp"
#include "model/synchronize_motor_cmd.hpp"
#include "model/single_motor_cmd.hpp"

namespace DynamixelDriver
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
     * @brief The DxlDriver class
     */
    class DynamixelDriver : public common::model::IDriver
    {
        public:
            DynamixelDriver();
            virtual ~DynamixelDriver() override;


            //commands
            int readSynchronizeCommand(common::model::SynchronizeMotorCmd cmd);
            int readSingleCommand(common::model::SingleMotorCmd cmd);

            uint32_t getPosition(common::model::DxlMotorState& motor_state);

            void readPositionStatus();
            void readHwStatus();

            int setLeds(int led, common::model::EMotorType type);

            int scanAndCheck() override;

            int ping(common::model::DxlMotorState& targeted_dxl);
            int type_ping_id(uint8_t id, common::model::EMotorType type);

            int rebootMotors();

            int sendCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, int reg_address, int value, int byte_number);
            int readCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, int32_t reg_address, int &value, int byte_number);

            //getters
            std::vector<uint8_t> getRemovedMotorList() const;

            std::vector<common::model::DxlMotorState> getMotorsStates() const;
            common::model::DxlMotorState getMotorState(uint8_t motor_id) const;

            int getLedState() const;

            int getAllIdsOnBus(std::vector<uint8_t> &id_list);

            void addMotor(common::model::EMotorType type, uint8_t id, bool isTool = false);

            // IDriver Interface
            void removeMotor(uint8_t id) override;
            bool isConnectionOk() const override;
            size_t getNbMotors() const override;
            void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const override;
            std::string getErrorMessage() const override;

        private:
            bool init() override;
            bool hasMotors() override;

            int setupCommunication();
            void interpreteErrorState();

            void checkRemovedMotors();


            void fillPositionStatus();
            void fillVoltageStatus();
            void fillTemperatureStatus();
            void fillErrorStatus();

            void readAndFillState(
                int (XDriver::*syncReadFunction)(const std::vector<uint8_t> &, std::vector<uint32_t> &),
                void (common::model::DxlMotorState::*setFunction)(int));

            int _syncWrite(int (XDriver::*syncWriteFunction)(const std::vector<uint8_t> &,
                                                                    const std::vector<uint32_t> &),
                                  const common::model::SynchronizeMotorCmd& cmd);

            int _singleWrite(int (XDriver::*singleWriteFunction)(uint8_t id, uint32_t), common::model::EMotorType dxl_type,
                                  const common::model::SingleMotorCmd& cmd);
        private:
            ros::NodeHandle _nh;

            std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
            std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

            std::string _device_name;
            int _uart_baudrate;

            std::vector<uint8_t> _all_motor_connected; //with all dxl motors connected (including the tool)
            std::vector<uint8_t> _removed_motor_id_list;

            std::map<uint8_t, common::model::DxlMotorState> _state_map;
            std::map<common::model::EMotorType, std::vector<uint8_t> > _ids_map;
            std::map<common::model::EMotorType, std::shared_ptr<XDriver> > _xdriver_map;

            // for hardware control
            bool _is_connection_ok;
            std::string _debug_error_message;

            int _hw_fail_counter_read;

            int _led_state;

    };

    //inline getters

    inline
    bool DynamixelDriver::isConnectionOk() const
    {
        return _is_connection_ok;
    }

    inline
    std::vector<uint8_t> DynamixelDriver::getRemovedMotorList() const
    {
        return _removed_motor_id_list;
    }

    inline
    std::string DynamixelDriver::getErrorMessage() const
    {
        return _debug_error_message;
    }

    inline
    int DynamixelDriver::getLedState() const
    {
        return _led_state;
    }

    inline
    void DynamixelDriver::getBusState(bool &connection_state, std::vector<uint8_t> &motor_id,
                                std::string &debug_msg) const
    {
        debug_msg = _debug_error_message;
        motor_id = _all_motor_connected;
        connection_state = isConnectionOk();
    }

    inline
    bool DynamixelDriver::hasMotors()
    {
        return _state_map.size() > 0;
    }

}

#endif
