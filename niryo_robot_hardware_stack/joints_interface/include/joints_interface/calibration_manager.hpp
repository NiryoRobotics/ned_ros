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
#include <cstdint>
#include <string>
#include <vector>
#include <thread>

#include "common/model/joint_state.hpp"
#include "common/model/bus_protocol_enum.hpp"

#include "ttl_driver/ttl_interface_core.hpp"
#include "can_driver/can_interface_core.hpp"

#include "niryo_robot_msgs/CommandStatus.h"
#include "joints_interface/FactoryCalibration.h"

namespace joints_interface
{

    class CalibrationManager
    {

    public:
        CalibrationManager(ros::NodeHandle &nh,
                           std::vector<std::shared_ptr<common::model::JointState>> joint_list,
                           std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                           std::shared_ptr<can_driver::CanInterfaceCore> can_interface);

        ~CalibrationManager() = default;

        // non copyable class
        CalibrationManager(const CalibrationManager &) = delete;
        CalibrationManager(CalibrationManager &&) = delete;

        CalibrationManager &operator=(CalibrationManager &&) = delete;
        CalibrationManager &operator=(const CalibrationManager &) = delete;

        int startCalibration(int mode, std::string &result_message);
        int startFactoryCalibration(FactoryCalibration::Request::_command_type command, FactoryCalibration::Request::_ids_type ids, std::string &result_message);

        common::model::EStepperCalibrationStatus getCalibrationStatus() const;

    private:
        void initParameters(ros::NodeHandle &nh);
        void initCalibrationParams(ros::NodeHandle &nh);

        common::model::EStepperCalibrationStatus autoCalibration();
        common::model::EStepperCalibrationStatus manualCalibration();

        // tests
        bool canProcessManualCalibration(std::string &result_message);
        bool steppersConnected();

        // commands
        void setTorqueStepperMotor(const std::shared_ptr<common::model::JointState> &pState, uint8_t percentage);

        void initVelocityProfiles();
        void resetVelocityProfiles();
        void moveRobotBeforeCalibration();
        void moveSteppersToHome();
        void sendCalibrationToSteppers();
        void activateTorque(bool activated);
        bool writeHomingAbsPosition();
        bool readHomingAbsPosition();

        // file operations
        bool saveCalibrationOffsetsToFile(const std::vector<int> &motor_id_list, const std::vector<int> &steps_list);
        bool readCalibrationOffsetsFromFile(std::vector<int> &motor_id_list, std::vector<int> &steps_list);
        bool saveHomingAbsPositionToFile();
        bool readHomingAbsPositionFromFile();

    private:
        struct CalibrationConfig
        {
            uint8_t stall_threshold{0};
            int8_t direction{0};
            int32_t delay{0};
            common::model::VelocityProfile profile;
        };

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;
        std::shared_ptr<can_driver::CanInterfaceCore> _can_interface;
        // one of the above interface, responsible for steppers
        std::shared_ptr<common::util::IDriverCore> _stepper_bus_interface;

        std::vector<std::shared_ptr<common::model::JointState>> _joint_states_list;
        std::map<uint8_t, CalibrationConfig> _calibration_params_map;

        int _calibration_timeout{0};
        bool _simulation_mode{false};

        std::string _calibration_file_name;
        std::string _homing_offset_file_name;
        std::string _hardware_version;

        static constexpr int AUTO_CALIBRATION = 1;
        static constexpr int MANUAL_CALIBRATION = 2;
    };

} // JointsInterface
#endif
