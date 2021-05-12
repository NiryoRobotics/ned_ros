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

//ros
#include <ros/ros.h>

#include "model/idriver_core.hpp"

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
    /**
     * @brief The DxlDriverCore class
     */
    class DxlDriverCore : public common::model::IDriverCore
    {
    public:

        DxlDriverCore();
        virtual ~DxlDriverCore() override;

        int setEndEffector(common::model::EMotorType type, uint8_t motor_id);
        void unsetEndEffector(uint8_t motor_id);

        void clearSingleCommandQueue();
        void clearEndEffectorCommandQueue();

        void setTrajectoryControllerCommands(const std::vector<std::pair<uint8_t, uint32_t> > &cmd);
        void setSyncCommand(const common::model::SynchronizeMotorCmd &cmd);
        void addSingleCommandToQueue(const common::model::SingleMotorCmd &cmd);
        void addSingleCommandToQueue(const std::vector<common::model::SingleMotorCmd> &cmd);

        void addEndEffectorCommandToQueue(const common::model::SingleMotorCmd &cmd);
        void addEndEffectorCommandToQueue(const std::vector<common::model::SingleMotorCmd> &cmd);

        //direct commands
        std::vector<uint8_t> scanTools();

        int update_leds(void);

        int rebootMotors();

        //getters
        std::vector<uint8_t> getRemovedMotorList() const;
        double getEndEffectorState(uint8_t id) const;

        dynamixel_driver::DxlArrayMotorHardwareStatus getHwStatus() const;

        std::vector<common::model::DxlMotorState> getDxlStates() const;
        common::model::DxlMotorState getDxlState(uint8_t motor_id) const;

        // IDriverCore interface
        void startControlLoop() override;

        void activeDebugMode(bool mode) override;

        bool isConnectionOk() const override;
        int launchMotorsReport() override;
        niryo_robot_msgs::BusState getBusState() const override;

    private:
        void init() override;
        void initParameters() override;
        void resetHardwareControlLoopRates() override;
        void controlLoop() override;
        void _executeCommand() override;

        int motorScanReport(uint8_t motor_id);
        int motorCmdReport(uint8_t motor_id, common::model::EMotorType motor_type);

        //use other callbacks instead of executecommand
        bool callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
        bool callbackSendCustomDxlValue(dynamixel_driver::SendCustomDxlValue::Request &req, dynamixel_driver::SendCustomDxlValue::Response &res);
        bool callbackReadCustomDxlValue(dynamixel_driver::ReadCustomDxlValue::Request &req, dynamixel_driver::ReadCustomDxlValue::Response &res);

    private:
        //common to stepper_driver_core. Put in abstract class ?
        ros::NodeHandle _nh;
        bool _control_loop_flag;
        bool _debug_flag;

        std::mutex _control_loop_mutex;
        std::thread _control_loop_thread;

        double _control_loop_frequency;

        double _delta_time_data_read;
        double _delta_time_write;

        double _time_hw_data_last_read;
        double _time_hw_data_last_write;

        double _time_check_connection_last_read;

        //specific to dxl
        double _delta_time_status_read;
        double _time_hw_status_last_read;

        double _time_check_end_effector_last_read;

        std::unique_ptr<DxlDriver> _dynamixel;

        std::vector<std::pair<uint8_t, uint32_t> > _joint_trajectory_cmd;

        common::model::SynchronizeMotorCmd _dxl_sync_cmds;
        std::queue<common::model::SingleMotorCmd> _dxl_single_cmds;
        std::queue<common::model::SingleMotorCmd> _end_effector_cmds;

        ros::ServiceServer _activate_leds_server;
        ros::ServiceServer _custom_cmd_server;
        ros::ServiceServer _custom_cmd_getter;

    private:
        static constexpr int QUEUE_OVERFLOW = 20;

    };

    /**
     * @brief DxlDriverCore::getDxlStates
     * @return
     */
    inline
    std::vector<common::model::DxlMotorState> DxlDriverCore::getDxlStates() const
    {
        return _dynamixel->getMotorsStates();
    }

    /**
     * @brief DxlDriverCore::isConnectionOk
     * @return
     */
    inline
    bool DxlDriverCore::isConnectionOk() const
    {
        return _dynamixel->isConnectionOk();
    }

    /**
     * @brief DxlDriverCore::getDxlState
     * @param motor_id
     * @return
     */
    inline
    common::model::DxlMotorState DxlDriverCore::getDxlState(uint8_t motor_id) const
    {
        return _dynamixel->getMotorState(motor_id);
    }

    /**
     * @brief DxlDriverCore::getRemovedMotorList
     * @return
     */
    inline
    std::vector<uint8_t> DxlDriverCore::getRemovedMotorList() const
    {
        return _dynamixel->getRemovedMotorList();
    }
} //DynamixelDriver

#endif
