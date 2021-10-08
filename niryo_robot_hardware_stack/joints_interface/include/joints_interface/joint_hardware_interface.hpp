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

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// niryo
#include "joints_interface/calibration_manager.hpp"
#include "can_driver/can_interface_core.hpp"
#include "ttl_driver/ttl_interface_core.hpp"
#include "common/model/joint_state.hpp"

namespace joints_interface
{

class JointHardwareInterface : public hardware_interface::RobotHW
{

    public:
        JointHardwareInterface(ros::NodeHandle& rootnh,
                               ros::NodeHandle& robot_hwnh,
                               std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                               std::shared_ptr<can_driver::CanInterfaceCore> can_interface);

        void sendInitMotorsParams();
        void setSteppersProfiles();
        int calibrateJoints(int mode, std::string &result_message);
        void setNeedCalibration();
        void activateLearningMode(bool activated);
        void synchronizeMotors(bool synchronize);

        void setCommandToCurrentPosition();

        bool needCalibration() const;
        bool isCalibrationInProgress() const;
        bool getLearningMode() const;
        std::string getHardwareVersion() const;

        const std::vector<std::shared_ptr<common::model::JointState> >& getJointsState() const;

        // RobotHW interface
    public:
        bool init(ros::NodeHandle& rootnh, ros::NodeHandle &robot_hwnh) override;

        virtual void read(const ros::Time &/*time*/, const ros::Duration &/*period*/) override;
        virtual void write(const ros::Time &/*time*/, const ros::Duration &/*period*/) override;

    private:
        void initParameters(ros::NodeHandle &nh);
        bool initStepper(ros::NodeHandle &robot_hwnh,
                         const std::shared_ptr<common::model::StepperMotorState>& stepperState,
                         const std::string& currentNamespace) const;
        bool initDxl(ros::NodeHandle &robot_hwnh,
                     const std::shared_ptr<common::model::DxlMotorState>& dxlState,
                     const std::string& currentNamespace) const;

    private:
        std::string _hardware_version;

        hardware_interface::JointStateInterface _joint_state_interface;
        hardware_interface::PositionJointInterface _joint_position_interface;

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;
        std::shared_ptr<can_driver::CanInterfaceCore> _can_interface;

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
 * @brief JointHardwareInterface::getHardwareVersion
 * @return
 */
inline
std::string JointHardwareInterface::getHardwareVersion() const
{
    return _hardware_version;
}

/**
 * @brief JointHardwareInterface::getLearningMode
 * @return
 */
inline
bool JointHardwareInterface::getLearningMode() const
{
    return _learning_mode;
}

/**
 * @brief JointHardwareInterface::getJointsState
 * @return
 */
inline
const std::vector<std::shared_ptr<common::model::JointState>> &
JointHardwareInterface::getJointsState() const
{
    return _joint_list;
}

} // JointsInterface

#endif
