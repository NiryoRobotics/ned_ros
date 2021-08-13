/*
ttl_interface_core.hpp
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

#ifndef TTL_INTERFACE_CORE_HPP
#define TTL_INTERFACE_CORE_HPP

// std
#include <memory>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>
#include <mutex>
#include <exception>

// ros
#include <ros/ros.h>

#include "common/model/i_driver_core.hpp"
#include "common/model/i_interface_core.hpp"

#include "ttl_driver/ttl_manager.hpp"
#include "ttl_driver/ArrayMotorHardwareStatus.h"
#include "ttl_driver/SendCustomValue.h"
#include "ttl_driver/ReadCustomValue.h"

#include "niryo_robot_msgs/BusState.h"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include "common/model/stepper_motor_state.hpp"
#include "common/model/dxl_motor_state.hpp"
#include "common/model/motor_type_enum.hpp"

#include "ttl_driver/abstract_motor_driver.hpp"

namespace ttl_driver
{
/**
 * @brief The TtlInterfaceCore class schedules and manager the communication in the TTL bus
 * its main purpose it to manage queues of commands for the TTL Motors
 */
class TtlInterfaceCore : public common::model::IDriverCore, public common::model::IInterfaceCore
{

    public:
        TtlInterfaceCore(ros::NodeHandle& nh);
        virtual ~TtlInterfaceCore() override;

        bool init(ros::NodeHandle& nh) override;

        int setEndEffector(common::model::EMotorType type, uint8_t motor_id);
        void unsetEndEffector(uint8_t motor_id);

        void clearSingleCommandQueue();
        void clearConveyorCommandQueue();

        bool setMotorPID(const std::shared_ptr<common::model::JointState>& motorState);
        void setTrajectoryControllerCommands(const std::vector<std::pair<uint8_t, uint32_t> > &cmd);

        void setSyncCommand(const std::shared_ptr<common::model::ISynchronizeMotorCmd>& cmd) override;

        // we have to use ISingleMotorCmd instead of AbstractTtlMotorCmd because this is pure virtual method in IDriverCore
        // IDriverCore used by Can and Ttl so AbstractTtlMotorCmd or AbstractCanMotorCmd can't be used in this case
        void addSingleCommandToQueue(const std::shared_ptr<common::model::ISingleMotorCmd>& cmd) override;
        void addSingleCommandToQueue(const std::vector<std::shared_ptr<common::model::ISingleMotorCmd> >& cmd) override;

        void addEndEffectorCommandToQueue(const std::shared_ptr<common::model::DxlSingleCmd> &cmd);
        void addEndEffectorCommandToQueue(const std::vector< std::shared_ptr<common::model::DxlSingleCmd> >& cmd);

        // conveyor control
        int setConveyor(uint8_t motor_id, uint8_t default_conveyor_id = 6) override;
        void unsetConveyor(uint8_t motor_id) override;

        // direct commands
        std::vector<uint8_t> scanTools();

        int update_leds(void);

        int rebootMotors();
        bool rebootMotor(uint8_t motor_id);

        // getters
        std::vector<uint8_t> getRemovedMotorList() const;
        double getEndEffectorState(uint8_t id) const;

        ttl_driver::ArrayMotorHardwareStatus getHwStatus() const;

        std::vector<std::shared_ptr<common::model::JointState> > getStates() const override;
        common::model::JointState getState(uint8_t motor_id) const override;

        // IDriverCore interface
        void startControlLoop() override;

        bool scanMotorId(uint8_t motor_to_find) override;

        void startCalibration() override;
        void resetCalibration() override ;
        bool isCalibrationInProgress() const override ;
        int32_t getCalibrationResult(uint8_t id) const override ;
        common::model::EStepperCalibrationStatus getCalibrationStatus() const override;

        void activeDebugMode(bool mode) override;

        bool isConnectionOk() const override;
        int launchMotorsReport() override;
        niryo_robot_msgs::BusState getBusState() const override;

    private:
        virtual void initParameters(ros::NodeHandle& nh) override;
        virtual void startServices(ros::NodeHandle& nh) override;
        virtual void startPublishers(ros::NodeHandle &nh) override;
        virtual void startSubscribers(ros::NodeHandle &nh) override;

        void resetHardwareControlLoopRates() override;
        void controlLoop() override;
        void _executeCommand() override;

        int motorScanReport(uint8_t motor_id);
        int motorCmdReport(uint8_t motor_id, common::model::EMotorType motor_type);

        // use other callbacks instead of executecommand
        bool _callbackActivateLeds(niryo_robot_msgs::SetInt::Request &req, niryo_robot_msgs::SetInt::Response &res);
        bool _callbackSendCustomValue(ttl_driver::SendCustomValue::Request &req, ttl_driver::SendCustomValue::Response &res);
        bool _callbackReadCustomValue(ttl_driver::ReadCustomValue::Request &req, ttl_driver::ReadCustomValue::Response &res);

    private:
        bool _control_loop_flag{false};
        bool _debug_flag{false};

        std::mutex _control_loop_mutex;
        std::thread _control_loop_thread;

        double _control_loop_frequency{0.0};

        double _delta_time_data_read{0.0};
        double _delta_time_write{0.0};

        double _time_hw_data_last_read{0.0};
        double _time_hw_data_last_write{0.0};

        double _time_check_connection_last_read{0.0};

        // specific to dxl
        double _delta_time_status_read{0.0};
        double _time_hw_status_last_read{0.0};

        double _time_check_end_effector_last_read{0.0};

        std::unique_ptr<TtlManager> _ttl_manager;

        std::vector<std::pair<uint8_t, uint32_t> > _joint_trajectory_cmd;

        // ttl cmds
        std::shared_ptr<common::model::AbstractTtlSynchronizeMotorCmd> _sync_cmds;
        std::queue<std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> > _single_cmds_queue;
        std::queue<std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> > _conveyor_cmds_queue;

        ros::ServiceServer _activate_leds_server;
        ros::ServiceServer _custom_cmd_server;
        ros::ServiceServer _custom_cmd_getter;

        static constexpr int QUEUE_OVERFLOW = 20;
        static constexpr double TTL_VOLTAGE_DIVISOR = 10.0;

        // IDriverCore interface
public:
        virtual common::model::EBusProtocol getBusProtocol() const override;
};

/**
 * @brief TtlInterfaceCore::isConnectionOk
 * @return
 */
inline
bool TtlInterfaceCore::isConnectionOk() const
{
    return _ttl_manager->isConnectionOk();
}

/**
 * @brief TtlInterfaceCore::getBusProtocol
 * @return
 */
inline
common::model::EBusProtocol
TtlInterfaceCore::getBusProtocol() const
{
    return common::model::EBusProtocol::TTL;
}

/**
 * @brief TtlInterfaceCore::getRemovedMotorList
 * @return
 */
inline
std::vector<uint8_t> TtlInterfaceCore::getRemovedMotorList() const
{
    return _ttl_manager->getRemovedMotorList();
}

} // ttl_driver

#endif // TTL_INTERFACE_CORE_HPP
