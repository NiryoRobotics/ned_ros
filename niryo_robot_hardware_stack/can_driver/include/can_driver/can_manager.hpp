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
#include "common/util/i_bus_manager.hpp"
#include "common/model/stepper_motor_state.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"

#include "can_driver/StepperMotorCommand.h"
#include "can_driver/StepperCmd.h"
#include "can_driver/fake_can_data.hpp"

#include "abstract_can_driver.hpp"
#include "ros/node_handle.h"


namespace can_driver
{

/**
 * @brief The CanManager class
 */
class CanManager : public common::util::IBusManager
{
public:
    CanManager() = delete;
    CanManager(ros::NodeHandle& nh);
    ~CanManager() override;

    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c21-if-you-define-or-delete-any-copy-move-or-destructor-function-define-or-delete-them-all
    CanManager( const CanManager& ) = delete;
    CanManager( CanManager&& ) = delete;
    CanManager& operator= ( CanManager && ) = delete;
    CanManager& operator= ( const CanManager& ) = delete;

    // IBusManager Interface
    bool init(ros::NodeHandle& nh) override;

    int addHardwareComponent(std::shared_ptr<common::model::AbstractHardwareState> &&state) override;

    void removeHardwareComponent(uint8_t id) override;
    bool isConnectionOk() const override;

    int scanAndCheck() override;
    bool ping(uint8_t id) override;

    size_t getNbMotors() const override;
    void getBusState(bool& connection_status, std::vector<uint8_t>& motor_list, std::string& error) const override;
    std::string getErrorMessage() const override;

    // commands
    int changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id);

    int writeSingleCommand(std::unique_ptr<common::model::AbstractCanSingleMotorCmd>&& cmd);
    void executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, int32_t> > cmd_vec);

    // read status
    void readStatus();

    //calibration
    void startCalibration() override;
    void resetCalibration() override;
    int32_t getCalibrationResult(uint8_t id) const override;
    common::model::EStepperCalibrationStatus getCalibrationStatus() const override;
    void setCalibrationStatus(const common::model::EStepperCalibrationStatus status);

    // getters
    int32_t getPosition(const common::model::JointState &motor_state) const;

    std::vector<std::shared_ptr<common::model::JointState> > getMotorsStates() const;
    std::shared_ptr<common::model::AbstractHardwareState> getHardwareState(uint8_t motor_id) const;

    std::vector<uint8_t> getRemovedMotorList() const override;
private:
    int setupCommunication() override;
    void addHardwareDriver(common::model::EHardwareType hardware_type) override;

    void updateCurrentCalibrationStatus();

    void _verifyMotorTimeoutLoop();
    double getCurrentTimeout() const;

    // config params using in fake driver
    void readFakeConfig(bool use_simu_conveyor);
    template<typename Reg>
    void retrieveFakeMotorData(const std::string& current_ns, std::map<uint8_t, Reg>& fake_params);

private:
    ros::NodeHandle _nh;
    std::shared_ptr<mcp_can_rpi::MCP_CAN> _mcp_can;
    std::shared_ptr<FakeCanData> _fake_data;

    std::vector<uint8_t> _all_motor_connected; // with all can motors connected (including the conveyor)
    std::vector<uint8_t> _removed_motor_id_list;

    // state of a component for a given id
    std::map<uint8_t, std::shared_ptr<common::model::AbstractHardwareState> > _state_map;
    // map of drivers for a given hardware type (xl, stepper, end effector)
    std::map<common::model::EHardwareType, std::shared_ptr<can_driver::AbstractCanDriver> > _driver_map;

    std::string _debug_error_message;

    // for hardware control
    std::mutex  _stepper_timeout_mutex;
    std::thread _stepper_timeout_thread;

    double _calibration_timeout{30.0};
    bool _isPing{false};
    static double constexpr _ping_timeout{5.0}; // prevent timeout check on motors when a ping failed

    common::model::EStepperCalibrationStatus _calibration_status{common::model::EStepperCalibrationStatus::UNINITIALIZED};

    bool _simulation_mode{false};
    bool _is_connection_ok{false};

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
 * @brief CanManager::getRemovedMotorList
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
 * @brief CanManager::setCalibrationStatus
 * @return
 */
inline
void CanManager::setCalibrationStatus(const common::model::EStepperCalibrationStatus status)
{
    _calibration_status = status;
}

/**
 * @brief CanManager::retrieveFakeMotorData get config for motors
 * @param current_ns
 * @param fake_params
 */
template<typename Reg>
void CanManager::retrieveFakeMotorData(const std::string& current_ns, std::map<uint8_t, Reg> &fake_params)
{
    std::vector<int> hw_ids;
    _nh.getParam(current_ns + "id", hw_ids);

    std::vector<int> hw_positions;
    _nh.getParam(current_ns + "position", hw_positions);
    assert(hw_ids.size() == hw_positions.size());

    std::vector<int> hw_temperatures;
    _nh.getParam(current_ns + "temperature", hw_temperatures);
    assert(hw_positions.size() == hw_temperatures.size());

     std::vector<double> hw_voltages;
    _nh.getParam(current_ns + "voltage", hw_voltages);
    assert(hw_temperatures.size() == hw_voltages.size());

    std::vector<int> hw_model_numbers;
    _nh.getParam(current_ns + "model_number", hw_model_numbers);
    assert(hw_voltages.size() == hw_model_numbers.size());

     std::vector<std::string> hw_firmwares;
    _nh.getParam(current_ns + "firmware", hw_firmwares);
    assert(hw_firmwares.size() == hw_firmwares.size());

    for (size_t i = 0; i < hw_ids.size(); i++)
    {
        Reg tmp;
        tmp.id = static_cast<uint8_t>(hw_ids.at(i));
        tmp.position = static_cast<int32_t>(hw_positions.at(i));
        tmp.temperature = static_cast<uint8_t>(hw_temperatures.at(i));
        tmp.voltage = hw_voltages.at(i);
        tmp.model_number = static_cast<uint16_t>(hw_model_numbers.at(i));
        tmp.firmware = hw_firmwares.at(i);
        fake_params.insert(std::make_pair(tmp.id, tmp));
    }
}

} // namespace can_driver

#endif // CAN_MANAGER_H
