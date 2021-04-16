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

#ifndef DXL_DRIVER_CORE_HPP
#define DXL_DRIVER_CORE_HPP

//std
#include <memory>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>
#include <mutex>
#include <std_msgs/Int64MultiArray.h>

//ros
#include <ros/ros.h>

#include "dynamixel_driver/dxl_driver.hpp"
#include "dynamixel_driver/DxlArrayMotorHardwareStatus.h"
#include "dynamixel_driver/SendCustomDxlValue.h"
#include "dynamixel_driver/ReadCustomDxlValue.h"

#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include "model/dxl_motor_state.hpp"
#include "model/motor_type_enum.hpp"
#include "model/single_motor_cmd.hpp"
#include "model/synchronize_motor_cmd.hpp"



namespace DynamixelDriver
{
    class DynamixelDriverCore
    {
    public:

        DynamixelDriverCore();
        virtual ~DynamixelDriverCore();

        void startControlLoop();

        void setTrajectoryControllerCommands(std::vector<uint32_t>& cmd);

        void setDxlCommands(const common::model::SynchronizeMotorCmd &cmd);
        void addDxlCommandToQueue(const common::model::SingleMotorCmd &cmd);
        void addDxlCommandToQueue(const std::vector<common::model::SingleMotorCmd> &cmd);

        void addEndEffectorCommandToQueue(const common::model::SingleMotorCmd &cmd);
        void addEndEffectorCommandToQueue(const std::vector<common::model::SingleMotorCmd> &cmd);

        int ping_id(uint8_t id, common::model::EMotorType type);
        std::vector<uint8_t> scanTools();
        int setEndEffector(uint8_t id, common::model::EMotorType type);
        void unsetEndEffector(uint8_t id, common::model::EMotorType type);
        uint32_t getEndEffectorState(uint8_t id, common::model::EMotorType type);

        dynamixel_driver::DxlArrayMotorHardwareStatus getHwStatus() const;
        niryo_robot_msgs::BusState getDxlBusState() const;

        std::vector<common::model::DxlMotorState> getDxlStates() const;

        std::vector<uint8_t> getRemovedMotorList() const;
        
        int update_leds(void);

        void activeDebugMode(bool mode);
        int launchMotorsReport();
        int motorScanReport(uint8_t motor_id, common::model::EMotorType motor_type);
        int motorCmdReport(uint8_t motor_id, common::model::EMotorType motor_type);
        int rebootMotors();

    private:
        void init();
        void initParameters();
        void resetHardwareControlLoopRates();
        void controlLoop();
        void _executeCommand();

        //use other callbacks instead of executecommand
        bool callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
        bool callbackSendCustomDxlValue(dynamixel_driver::SendCustomDxlValue::Request &req, dynamixel_driver::SendCustomDxlValue::Response &res);
        bool callbackReadCustomDxlValue(dynamixel_driver::ReadCustomDxlValue::Request &req, dynamixel_driver::ReadCustomDxlValue::Response &res);

    private:
        ros::NodeHandle _nh;
        bool _control_loop_flag;
        bool _debug_flag;

        std::mutex _control_loop_mutex;

        std::thread _control_loop_thread;

        double _control_loop_frequency;
        double _write_frequency;
        double _read_data_frequency;            
        double _read_status_frequency;
        double _check_connection_frequency;
        double _check_end_effector_frequency;

        double _time_hw_data_last_write;
        double _time_hw_data_last_read;
        double _time_hw_status_last_read;
        double _time_check_connection_last_read;
        double _time_check_end_effector_last_read;

        std::unique_ptr<DxlDriver> _dynamixel;


        //set a queue of cmds ? need to create an interface then
        std::vector<uint32_t> _joint_trajectory_controller_cmd;
        common::model::SynchronizeMotorCmd _dxl_sync_cmds;
        std::queue<common::model::SingleMotorCmd> _dxl_single_cmds;

        std::queue<common::model::SingleMotorCmd> _end_effector_cmds;

        ros::ServiceServer _activate_leds_server;
        ros::ServiceServer _custom_cmd_server;
        ros::ServiceServer _custom_cmd_getter;

    };
} //DynamixelDriver
#endif
