/*
joint_hardware_interface.hpp
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

#ifndef JOINT_HARDWARE_INTERFACE_HPP
#define JOINT_HARDWARE_INTERFACE_HPP

// std
#include <memory>
#include <algorithm>

// ros
#include <ros/ros.h>

// niryo
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include "joints_interface/calibration_manager.hpp"
#include "can_driver/can_driver_core.hpp"
#include "ttl_driver/ttl_driver_core.hpp"

#include "common/model/joint_state.hpp"

namespace joints_interface
{

class JointHardwareInterface : public hardware_interface::RobotHW
{

public:
    JointHardwareInterface(ros::NodeHandle& rootnh, 
                           ros::NodeHandle& robot_hwnh,
                           std::shared_ptr<ttl_driver::TtlDriverCore> ttl_driver,
                           std::shared_ptr<can_driver::CanDriverCore> can_driver);

    void sendInitMotorsParams();
    int calibrateJoints(int mode, std::string &result_message);
    void deactivateLearningMode();
    void setNeedCalibration();
    void activateLearningMode();
    void synchronizeMotors(bool synchronize);

    void setCommandToCurrentPosition();

    bool needCalibration() const;
    bool isCalibrationInProgress() const;

    std::string jointIdToJointName(uint8_t id, common::model::EMotorType motor_type) const;
    const std::vector<std::shared_ptr<common::model::JointState> >& getJointsState() const;

    // RobotHW interface
public:
    bool init(ros::NodeHandle& rootnh, ros::NodeHandle &robot_hwnh) override;

    virtual void read(const ros::Time &/*time*/, const ros::Duration &/*period*/) override;
    virtual void write(const ros::Time &/*time*/, const ros::Duration &/*period*/) override;

private:
    ros::NodeHandle _nh;

    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::PositionJointInterface _joint_position_interface;

    std::shared_ptr<ttl_driver::TtlDriverCore> _ttl_driver_core;
    std::shared_ptr<can_driver::CanDriverCore> _can_driver_core;
    std::unique_ptr<CalibrationManager> _calibration_manager;

    std::map<uint8_t, std::string> _map_stepper_name;
    std::map<uint8_t, std::string> _map_dxl_name;

    std::vector<std::shared_ptr<common::model::JointState> > _joint_list;

    bool _learning_mode{true};
};

/**
 * @brief JointHardwareInterface::getJointsState
 * @return
 */
inline
bool JointHardwareInterface::isCalibrationInProgress() const
{
    return _calibration_manager->CalibrationInprogress();
}

/**
 * @brief JointHardwareInterface::getJointsState
 * @return
 */
inline
const std::vector<std::shared_ptr<common::model::JointState> >&
JointHardwareInterface::getJointsState() const
{
    return _joint_list;
}
} // JointsInterface

#endif
