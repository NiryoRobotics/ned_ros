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

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>
#include <mutex>

#include "dynamixel_driver/dxl_driver.hpp"
#include "dynamixel_driver/DxlArrayMotorHardwareStatus.h"
#include "dynamixel_driver/SendCustomDxlValue.h"

#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include <std_msgs/Int64MultiArray.h>

#define DXL_VOLTAGE_DIVISOR 10.0

namespace DynamixelDriver
{
    class DynamixelDriverCore
    {
    public:

        DynamixelDriverCore();

        void init();

        void initParameters();

        void startControlLoop();
        void resetHardwareControlLoopRates();
        void controlLoop();

        dynamixel_driver::DxlArrayMotorHardwareStatus getHwStatus();
        niryo_robot_msgs::BusState getDxlBusState();

        void setTrajectoryControllerCommands(std::vector<uint32_t>& cmd);

        std::vector<DxlMotorState>& getDxlStates();

        void setDxlCommands(SynchronizeMotorCmd &cmd);

        int ping_id(uint8_t id, DxlMotorType type);
        std::vector<uint8_t> scanTools();
        int setEndEffector(uint8_t id, DxlMotorType type);
        void unsetEndEffector(uint8_t id, DxlMotorType type);
        void setEndEffectorCommands(std::vector<SingleMotorCmd> &cmd);
        uint32_t getEndEffectorState(uint8_t id, DxlMotorType type);
        std::vector<int>& getRemovedMotorList(); 
        
        int update_leds(void);

        void activeDebugMode(bool mode);
        int launchMotorsReport();
        int motorScanReport(uint8_t motor_id, DxlMotorType motor_type);
        int motorCmdReport(uint8_t motor_id, DxlMotorType motor_type);
        int rebootMotors();
        bool rebootMotor(uint8_t motor_id, DxlMotorType motor_type);

    private:
        ros::NodeHandle _nh;
        bool _control_loop_flag;
        bool _debug_flag;

        std::mutex _control_loop_mutex;

        boost::shared_ptr<std::thread> _control_loop_thread;

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

        boost::shared_ptr<DxlDriver> _dynamixel;

        void _executeCommand();

        std::vector<uint32_t> _joint_trajectory_controller_cmd;
        boost::shared_ptr<SynchronizeMotorCmd> _dxl_cmd;
        std::vector<SingleMotorCmd> _end_effector_cmd;

        ros::ServiceServer _activate_leds_server;
        ros::ServiceServer _custom_cmd_server;
        bool callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
        bool callbackSendCustomDxlValue(dynamixel_driver::SendCustomDxlValue::Request &req, dynamixel_driver::SendCustomDxlValue::Response &res);
    };
}
#endif
