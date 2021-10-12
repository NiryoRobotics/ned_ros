/*
calibration_interface.hpp
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

#ifndef CALIBRATION_INTERFACE_HPP
#define CALIBRATION_INTERFACE_HPP

#include <ros/ros.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <thread>

#include "common/model/joint_state.hpp"
#include "common/model/bus_protocol_enum.hpp"

#include "ttl_driver/ttl_interface_core.hpp"
#include "can_driver/can_interface_core.hpp"

#include "niryo_robot_msgs/CommandStatus.h"

namespace joints_interface
{

class CalibrationManager
{

public:
    CalibrationManager(ros::NodeHandle& nh,
                       std::vector<std::shared_ptr<common::model::JointState> > joint_list,
                       std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                       std::shared_ptr<can_driver::CanInterfaceCore> can_interface);

    virtual ~CalibrationManager();

    int startCalibration(int mode, std::string &result_message);

    bool CalibrationInprogress() const;

private:
    void initParameters(ros::NodeHandle& nh);

    common::model::EStepperCalibrationStatus autoCalibration();
    common::model::EStepperCalibrationStatus manualCalibration();

    std::shared_ptr<common::util::IDriverCore> getJointInterface(common::model::EBusProtocol proto);
    // tests
    bool canProcessManualCalibration(std::string &result_message);
    bool steppersConnected();

    // commands
    void setStepperCalibrationCommand(const std::shared_ptr<common::model::StepperMotorState>& pState,
                                      int32_t delay, int32_t calibration_direction, int32_t timeout);

    void setTorqueStepperMotor(const std::shared_ptr<common::model::JointState>& pState, bool status);

    void moveRobotBeforeCalibration();
    void moveSteppersToHome();
    void sendCalibrationToSteppers();
    void activateLearningMode(bool activated);

    // file operations
    bool setMotorsCalibrationOffsets(const std::vector<int>& motor_id_list, const std::vector<int> &steps_list);
    bool getMotorsCalibrationOffsets(std::vector<int> &motor_id_list, std::vector<int>& steps_list);

private:
    std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;
    std::shared_ptr<can_driver::CanInterfaceCore> _can_interface;

    std::vector<std::shared_ptr<common::model::JointState> > _joint_states_list;

    bool _calibration_in_progress{false};
    int _calibration_timeout{0};
    std::string _calibration_file_name;
    std::string _hardware_version;

    std::vector<int32_t> _motor_calibration_list;

    static constexpr int AUTO_CALIBRATION = 1;
    static constexpr int MANUAL_CALIBRATION = 2;

};

/**
 * @brief CalibrationManager::CalibrationInprogress
 * @return
 */
inline
bool CalibrationManager::CalibrationInprogress() const
{
  return _calibration_in_progress;
}

inline
std::shared_ptr<common::util::IDriverCore>
CalibrationManager::getJointInterface(common::model::EBusProtocol proto)
{
  if(common::model::EBusProtocol::CAN == proto)
    return _can_interface;
  else if(common::model::EBusProtocol::TTL == proto)
    return _ttl_interface;

  return nullptr;
}

} // JointsInterface
#endif
