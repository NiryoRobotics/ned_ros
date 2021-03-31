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

//drivers
#include "dynamixel_driver/xl320_driver.hpp"
#include "dynamixel_driver/xl330_driver.hpp"
#include "dynamixel_driver/xl430_driver.hpp"
#include "dynamixel_driver/xc430_driver.hpp"

#include "dynamixel_driver/dxl_motor_state.hpp"
#include "dynamixel_driver/synchronize_motor_cmd.hpp"
#include "dynamixel_driver/single_motor_cmd.hpp"

#define DXL_BUS_PROTOCOL_VERSION 2.0

#define DXL_FAIL_OPEN_PORT -4500
#define DXL_FAIL_PORT_SET_BAUDRATE -4501
#define DXL_FAIL_SETUP_GPIO -4502

#define RADIAN_TO_DEGREE 57.295779513082320876798154814105

#define TIME_TO_WAIT_IF_BUSY 0.0005

#define DXL_SCAN_OK 0
#define DXL_SCAN_MISSING_MOTOR -50
#define DXL_SCAN_UNALLOWED_MOTOR -51
#define DXL_WRONG_TYPE -52

namespace DynamixelDriver
{

    /**
     * @brief The DxlDriver class
     */
    class DxlDriver
    {
        public:
            DxlDriver();
            virtual ~DxlDriver();

            void addDynamixel(uint8_t id, DxlMotorType type, bool isTool = false);
            void removeDynamixel(uint8_t id, DxlMotorType);

            //commands
            void executeJointTrajectoryCmd(std::vector<uint32_t> &cmd);

            void readSynchronizeCommand(SynchronizeMotorCmd cmd);
            void readSingleCommand(SingleMotorCmd cmd);

            uint32_t getPosition(DxlMotorState& motor_state);

            void readPositionStatus();
            void readHwStatus();

            int setTorqueEnable(DxlMotorState& targeted_dxl, uint32_t torque_enable);
            int setGoalPosition(DxlMotorState& targeted_dxl, uint32_t position);

            int setGoalTorque(DxlMotorState& targeted_dxl, uint32_t torque);
            int setGoalVelocity(DxlMotorState& targeted_dxl, uint32_t velocity);

            int setLeds(int led, DxlMotorType type = DxlMotorType::MOTOR_TYPE_XL330);

            int scanAndCheck();

            int ping(DxlMotorState& targeted_dxl);
            int type_ping_id(uint8_t id, DxlMotorType type);

            int rebootMotors();

            int sendCustomDxlCommand(DxlMotorType motor_type, uint8_t id, uint32_t reg_address, uint32_t value, uint32_t byte_number);
            int readCustomDxlCommand(DxlMotorType motor_type, uint8_t id, uint32_t reg_address, uint32_t &value, uint32_t byte_number);

            //tests
            bool isConnectionOk() const;

            //getters
            std::vector<uint8_t> getRemovedMotorList() const;
            std::vector<DxlMotorState> getMotorsState() const;
            int getLedState() const;
            std::string getErrorMessage() const;
            void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const;

            int getAllIdsOnDxlBus(std::vector<uint8_t> &id_list);

            //setters
        private:
            int init();
            int setupCommunication();

            // cc move to tool class
            DxlMotorType dxlMotorTypeFromString(std::string type) const;

            void interpreteErrorState();

            void checkRemovedMotors();

            bool hasMotors();

            std::vector<uint8_t> getArmMotors();

            void fillPositionStatus();
            void fillVoltageStatus();
            void fillTemperatureStatus();
            void fillErrorStatus();

            void readAndFillState(
                int (XDriver::*syncReadFunction)(const std::vector<uint8_t> &, std::vector<uint32_t> &),
                void (DxlMotorState::*setFunction)(uint32_t));

            void syncWriteTorqueEnable(const std::vector<uint8_t>& motor_list, const std::vector<uint32_t>& torque_enable);
            void syncWritePositionCommand(const std::vector<uint8_t>& motor_list, const std::vector<uint32_t>& position_list);
            void syncWriteVelocityCommand(const std::vector<uint8_t>& motor_list, const std::vector<uint32_t>& velocity_list);
            void syncWriteEffortCommand(const std::vector<uint8_t>& motor_list, const std::vector<uint32_t>& effort_list);

            void syncWriteCommand(int (XDriver::*syncWriteFunction)(const std::vector<uint8_t> &, const std::vector<uint32_t> &),
                                  const std::vector<uint8_t> &motor_list,
                                  const std::vector<uint32_t> &param_list);

        private:
            ros::NodeHandle _nh;

            std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
            std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

            std::string _device_name;
            int _uart_baudrate;

            std::vector<uint8_t> _all_motor_connected; //with all dxl motors connected (including the tool)
            std::vector<uint8_t> _removed_motor_id_list;

            std::map<int, DxlMotorState> _state_map;
            std::map<DxlMotorType, std::vector<uint8_t> > _ids_map;
            std::map<DxlMotorType, std::shared_ptr<XDriver> > _xdriver_map;

            // for hardware control
            bool _is_dxl_connection_ok;
            std::string _debug_error_message;

            int _hw_fail_counter_read;

            int _led_state;
    };

    //inline getters

    inline
    bool DxlDriver::isConnectionOk() const
    {
        return _is_dxl_connection_ok;
    }

    inline
    std::vector<uint8_t> DxlDriver::getRemovedMotorList() const
    {
        return _removed_motor_id_list;
    }

    inline
    std::vector<DxlMotorState> DxlDriver::getMotorsState() const
    {
        std::vector<DxlMotorState> states;
        for (auto it = _state_map.cbegin(); it != _state_map.cend(); ++it)
            states.push_back(it->second);

        return states;
    }

    inline
    std::string DxlDriver::getErrorMessage() const
    {
        return _debug_error_message;
    }

    inline
    int DxlDriver::getLedState() const
    {
        return _led_state;
    }

    inline
    void DxlDriver::getBusState(bool &connection_state, std::vector<uint8_t> &motor_id,
                                std::string &debug_msg) const
    {
        debug_msg = _debug_error_message;
        motor_id = _all_motor_connected;
        connection_state = isConnectionOk();
    }

    inline
    DxlMotorType DxlDriver::dxlMotorTypeFromString(std::string type) const
    {
        if("xl430" == type)
           return DxlMotorType::MOTOR_TYPE_XL430;
        else if("xc430" == type)
           return DxlMotorType::MOTOR_TYPE_XC430;
        else if("xl320" == type)
           return DxlMotorType::MOTOR_TYPE_XL320;
        else if("xl330" == type)
           return DxlMotorType::MOTOR_TYPE_XL330;

        return DxlMotorType::MOTOR_TYPE_UNKNOWN;
    }

    inline
    bool DxlDriver::hasMotors()
    {
        return _state_map.size() > 0;
    }
}

#endif
