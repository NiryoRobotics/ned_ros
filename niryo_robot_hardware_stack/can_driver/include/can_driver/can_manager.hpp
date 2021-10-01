/*
    can_manager.hpp
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

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

// std
#include <cstdint>
#include <memory>
#include <functional>
#include <string>
#include <thread>
#include <map>
#include <mutex>

// ros
#include <ros/ros.h>
#include <vector>


// niryo
#include "common/model/i_bus_manager.hpp"
#include "common/model/stepper_motor_state.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"

#include "can_driver/StepperMotorCommand.h"
#include "can_driver/StepperCmd.h"

#include "abstract_can_driver.hpp"


namespace can_driver
{

/**
 * @brief The CanManager class
 */
class CanManager : public common::model::IBusManager
{
    public:

        CanManager(ros::NodeHandle& nh);
        virtual ~CanManager() override;

        // IBusManager Interface
        bool init(ros::NodeHandle& nh) override;

        void addHardwareComponent(const std::shared_ptr<common::model::AbstractHardwareState> &state) override;

        void removeHardwareComponent(uint8_t id) override;
        bool isConnectionOk() const override;

        int scanAndCheck() override;
        bool ping(uint8_t id) override;

        size_t getNbMotors() const override;
        void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const override;
        std::string getErrorMessage() const override;

        // commands
        int changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id);

        int writeSingleCommand(std::shared_ptr<common::model::AbstractCanSingleMotorCmd> cmd);
        void executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, int32_t> > cmd_vec);

        // read status
        void readStatus();

        //calibration
        void startCalibration() override;
        void resetCalibration() override;
        bool isCalibrationInProgress() const override;
        int32_t getCalibrationResult(uint8_t id) const override;
        common::model::EStepperCalibrationStatus getCalibrationStatus() const override;

        // getters
        int32_t getPosition(const common::model::JointState &motor_state) const;

        std::vector<std::shared_ptr<common::model::JointState> > getMotorsStates() const;
        std::shared_ptr<common::model::AbstractHardwareState> getHardwareState(uint8_t motor_id) const;

        std::vector<uint8_t> getRemovedMotorList() const override;
    private:
        int setupCommunication() override;
        void addHardwareDriver(common::model::EHardwareType hardware_type) override;

        void updateCurrentCalibrationStatus() override;

        void _verifyMotorTimeoutLoop();
        double getCurrentTimeout() const;

    private:
        bool _simulation_mode{false};
        std::shared_ptr<mcp_can_rpi::MCP_CAN> _mcp_can;

        std::vector<uint8_t> _all_motor_connected; // with all can motors connected (including the conveyor)
        std::vector<uint8_t> _removed_motor_id_list;

        // state of a component for a given id
        std::map<uint8_t, std::shared_ptr<common::model::AbstractHardwareState> > _state_map;
        // map of drivers for a given hardware type (xl, stepper, end effector)
        std::map<common::model::EHardwareType, std::shared_ptr<can_driver::AbstractCanDriver> > _driver_map;

        double _calibration_timeout{30.0};
        common::model::EStepperCalibrationStatus _calibration_status;

        // for hardware control
        bool _is_connection_ok{false};
        std::string _debug_error_message;

        std::mutex _stepper_timeout_mutex;
        std::thread _stepper_timeout_thread;
};

// inline getters

/**
 * @brief CanManager::isConnectionOk
 * @return
 */
inline
bool CanManager::isConnectionOk() const
{
  return _is_connection_ok;
}

/**
 * @brief CanManager::getNbMotors
 * @return
 */
inline
size_t CanManager::getNbMotors() const
{
    return _state_map.size();
}

/**
 * @brief TtlManager::getRemovedMotorList
 * @return
 */
inline
std::vector<uint8_t> CanManager::getRemovedMotorList() const
{
    return _removed_motor_id_list;
}

/**
 * @brief CanManager::getErrorMessage
 * @return
 */
inline
std::string CanManager::getErrorMessage() const
{
  return _debug_error_message;
}

/**
 * @brief CanManager::getCalibrationStatus
 * @return
 */
inline
common::model::EStepperCalibrationStatus
CanManager::getCalibrationStatus() const
{
    return _calibration_status;
}

/**
 * @brief CanManager::isCalibrationInProgress
 * @return
 */
inline
bool CanManager::isCalibrationInProgress() const {
    return common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS == _calibration_status;
}

} // namespace can_driver

#endif // CAN_MANAGER_H
