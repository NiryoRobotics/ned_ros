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
    class DxlDriver
    {
        public:
            DxlDriver();
            virtual ~DxlDriver();

            void addDynamixel(uint8_t id, common::model::EMotorType type, bool isTool = false);
            void removeDynamixel(uint8_t id, common::model::EMotorType);

            //commands
            void executeJointTrajectoryCmd(std::vector<uint32_t> &cmd);

            void readSynchronizeCommand(common::model::SynchronizeMotorCmd cmd);
            void readSingleCommand(common::model::SingleMotorCmd cmd);

            uint32_t getPosition(common::model::DxlMotorState& motor_state);

            void readPositionStatus();
            void readHwStatus();

            int setTorqueEnable(common::model::DxlMotorState& targeted_dxl, uint32_t torque_enable);
            int setGoalPosition(common::model::DxlMotorState& targeted_dxl, uint32_t position);

            int setGoalTorque(common::model::DxlMotorState& targeted_dxl, uint32_t torque);
            int setGoalVelocity(common::model::DxlMotorState& targeted_dxl, uint32_t velocity);

            int setPGain(common::model::DxlMotorState& targeted_dxl, uint32_t gain);
            int setIGain(common::model::DxlMotorState& targeted_dxl, uint32_t gain);
            int setDGain(common::model::DxlMotorState& targeted_dxl, uint32_t gain);
            int setFF1Gain(common::model::DxlMotorState& targeted_dxl, uint32_t gain);
            int setFF2Gain(common::model::DxlMotorState& targeted_dxl, uint32_t gain);

            int setLeds(int led, common::model::EMotorType type);

            int scanAndCheck();

            int ping(common::model::DxlMotorState& targeted_dxl);
            int type_ping_id(uint8_t id, common::model::EMotorType type);

            int rebootMotors();

            int sendCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, uint32_t reg_address, uint32_t value, uint32_t byte_number);
            int readCustomDxlCommand(common::model::EMotorType motor_type, uint8_t id, uint32_t reg_address, uint32_t &value, uint32_t byte_number);

            //tests
            bool isConnectionOk() const;

            //getters
            std::vector<uint8_t> getRemovedMotorList() const;
            std::vector<common::model::DxlMotorState> getMotorsState() const;
            int getLedState() const;
            std::string getErrorMessage() const;
            void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const;

            int getAllIdsOnDxlBus(std::vector<uint8_t> &id_list);

        private:
            void init();
            int setupCommunication();
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
                void (common::model::DxlMotorState::*setFunction)(uint32_t));

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

            std::map<uint8_t, common::model::DxlMotorState> _state_map;
            std::map<common::model::EMotorType, std::vector<uint8_t> > _ids_map;
            std::map<common::model::EMotorType, std::shared_ptr<XDriver> > _xdriver_map;

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
    std::vector<common::model::DxlMotorState> DxlDriver::getMotorsState() const
    {
        std::vector<common::model::DxlMotorState> states;
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
    bool DxlDriver::hasMotors()
    {
        return _state_map.size() > 0;
    }


    //  Conversions

    /**
     *
     */
    template<typename T>
    uint32_t rad_pos_to_motor_pos(double position_rad)
    {
        static_assert (std::is_base_of<DynamixelDriver::XDriver, T>::value, "T must derive from class XDriver");
        static_assert(0 != T::TOTAL_ANGLE, "TOTAL_ANGLE of class T must be non null");

        uint32_t converted = T::MIDDLE_POSITION + (position_rad * RADIAN_TO_DEGREE * T::TOTAL_RANGE_POSITION) / T::TOTAL_ANGLE;
        return converted;
    }

    template<typename T>
    double motor_pos_to_rad_pos(uint32_t pos)
    {
        static_assert (std::is_base_of<DynamixelDriver::XDriver, T>::value, "T must derive from class XDriver");
        static_assert(0 != T::TOTAL_RANGE_POSITION, "TOTAL_ANGLE of class T must be non null");

        double converted = ((pos - T::MIDDLE_POSITION) * T::TOTAL_ANGLE) / (RADIAN_TO_DEGREE * T::TOTAL_RANGE_POSITION);

        return converted;
    }

}

#endif
