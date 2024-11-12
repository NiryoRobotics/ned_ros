/*
    conveyor_interface_core.cpp
    Copyright (C) 2017 Niryo
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

// c++
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// ros

// niryo
#include "common/model/bus_protocol_enum.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "niryo_robot_msgs/CommandStatus.h"

using ::std::shared_ptr;
using ::std::to_string;

using ::common::model::ConveyorState;
using ::common::model::EBusProtocol;
using ::common::model::EStepperCommandType;
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperSingleCmd;
using ::common::model::StepperTtlSingleCmd;

namespace conveyor_interface
{

/**
 * @brief ConveyorInterfaceCore::ConveyorInterfaceCore
 * @param nh
 * @param ttl_interface
 * @param can_interface
 */
ConveyorInterfaceCore::ConveyorInterfaceCore(ros::NodeHandle &nh, shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface, shared_ptr<can_driver::CanInterfaceCore> can_interface)
    : _ttl_interface(std::move(ttl_interface)), _can_interface(std::move(can_interface))

{
    ROS_DEBUG("ConveyorInterfaceCore::ConveyorInterfaceCore - ctor");

    init(nh);
}

/**
 * @brief ConveyorInterfaceCore::init
 * @param nh
 */
bool ConveyorInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("ConveyorInterfaceCore::init - Init parameters...");
    initParameters(nh);

    ROS_DEBUG("ConveyorInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("ConveyorInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("ConveyorInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief ConveyorInterfaceCore::initParameters
 * @param nh
 */
void ConveyorInterfaceCore::initParameters(ros::NodeHandle &nh)
{
    std::string typeStr;
    nh.getParam("type", typeStr);

    if (_can_interface && nh.hasParam("can"))
    {
        BusConfig canConfig(HardwareTypeEnum(typeStr.c_str()));

        int default_conveyor_id{CAN_DEFAULT_ID};
        std::vector<int> id_pool_list;

        nh.getParam("can/max_effort", canConfig.max_effort);
        nh.getParam("can/micro_steps", canConfig.micro_steps);
        nh.getParam("can/direction", canConfig.direction);
        nh.getParam("can/default_id", default_conveyor_id);

        canConfig.default_id = static_cast<uint8_t>(default_conveyor_id);

        nh.getParam("can/pool_id_list", id_pool_list);

        std::string pool_str{"["};

        for (auto const &id : id_pool_list)
        {
            canConfig.pool_id_list.insert(static_cast<uint8_t>(id));
            pool_str += to_string(id) + ",";
        }

        pool_str.pop_back();
        pool_str += "]";

        ROS_DEBUG("ConveyorInterfaceCore::initParameters - CAN - conveyor max effort : %f", canConfig.max_effort);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - CAN - conveyor max effort : %f", canConfig.micro_steps);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - CAN - default conveyor id : %d", default_conveyor_id);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - CAN - default pool id : %s", pool_str.c_str());

        canConfig.interface = _can_interface;
        _bus_config_map.insert(std::make_pair(EBusProtocol::CAN, canConfig));
    }

    if (_ttl_interface && nh.hasParam("ttl"))
    {
        BusConfig ttlConfig(HardwareTypeEnum(typeStr.c_str()));

        int default_conveyor_id{TTL_DEFAULT_ID};
        std::vector<int> id_pool_list;

        nh.getParam("ttl/default_id", default_conveyor_id);
        nh.getParam("ttl/pool_id_list", id_pool_list);
        nh.getParam("ttl/direction", ttlConfig.direction);

        ttlConfig.default_id = static_cast<uint8_t>(default_conveyor_id);

        std::string pool_str{"["};

        for (auto const &id : id_pool_list)
        {
            ttlConfig.pool_id_list.insert(static_cast<uint8_t>(id));
            pool_str += to_string(id) + ",";
        }

        pool_str.pop_back();
        pool_str += "]";

        ROS_DEBUG("ConveyorInterfaceCore::initParameters - TTL - default conveyor id : %d", default_conveyor_id);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - TTL - default pool id : %s", pool_str.c_str());

        ttlConfig.interface = _ttl_interface;
        _bus_config_map.insert(std::make_pair(EBusProtocol::TTL, ttlConfig));
    }

    double feedback_frequency{1.0};
    nh.getParam("publish_frequency", feedback_frequency);
    assert(feedback_frequency);
    _publish_feedback_duration = 1.0 / feedback_frequency;

    ROS_DEBUG("ConveyorInterfaceCore::initParameters - publish feedback frequency : %f", feedback_frequency);
}

/**
 * @brief ConveyorInterfaceCore::startServices
 * @param nh
 */
void ConveyorInterfaceCore::startServices(ros::NodeHandle &nh)
{
    _ping_and_set_stepper_server = nh.advertiseService("/niryo_robot/conveyor/ping_and_set_conveyor", &ConveyorInterfaceCore::_callbackPingAndSetConveyor, this);

    _control_conveyor_server = nh.advertiseService("/niryo_robot/conveyor/control_conveyor", &ConveyorInterfaceCore::_callbackControlConveyor, this);

    _get_hardware_id_server = nh.advertiseService("/niryo_robot/conveyor/get_hardware_id", &ConveyorInterfaceCore::_callbackGetHardwareId, this);
}

/**
 * @brief ConveyorInterfaceCore::startPublishers
 */
void ConveyorInterfaceCore::startPublishers(ros::NodeHandle &nh)
{
    _conveyors_feedback_publisher = nh.advertise<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 10, true);

    _publish_conveyors_feedback_timer = nh.createTimer(ros::Duration(_publish_feedback_duration), [this](const ros::TimerEvent&) { this->_publishConveyorsFeedback(); });
}

/**
 * @brief ConveyorInterfaceCore::startSubscribers
 * @param nh
 */
void ConveyorInterfaceCore::startSubscribers(ros::NodeHandle & /*nh*/) { ROS_DEBUG("ConveyorInterfaceCore::startSubscribers - no subscriber to start"); }

/**
 * @brief ConveyorInterfaceCore::addConveyor
 * @return
 * scans hw on its interfaces to try to find conveyors
 * If it find one, it changes its id using the available ids in the pool
 */
conveyor_interface::SetConveyor::Response ConveyorInterfaceCore::addConveyor()
{
    auto constexpr CONVEYOR_V2_HWID = "niryo/conveyor2";
    auto constexpr CONVEYOR_V3_HWID = "niryo/conveyor3";

    conveyor_interface::SetConveyor::Response res;
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    res.id = 0;

    for (auto &[protocol, bus] : _bus_config_map)
    {
        // if we still have available ids in the pool of ids
        if (bus.isValid())
        {
            // take first
            uint8_t conveyor_id = *bus.pool_id_list.begin();

            auto conveyor_state = std::make_shared<ConveyorState>(EHardwareType::NED3PRO_STEPPER, protocol, bus.default_id, bus.default_id, CONVEYOR_V3_HWID);

            // we want to check the model number for this conveyor
            // if it fails, we will recover with the old stepper hardware without checking the model number
            conveyor_state->setStrictModelNumber(true);

            int result = niryo_robot_msgs::CommandStatus::FAILURE;
            // Try 3 times
            for (int tries = 0; tries < 3; tries++)
            {
                ROS_INFO("ConveyorInterfaceCore::addConveyor - Try %d", tries);
                // add conveyor to interface management
                result = bus.interface->setConveyor(conveyor_state);
                ROS_INFO("ConveyorInterfaceCore::addConveyor - Set conveyor on id %d", conveyor_id);
                if (result == niryo_robot_msgs::CommandStatus::HARDWARE_NOT_SUPPORTED)
                {
                    // retry with the old stepper
                    conveyor_state = std::make_shared<ConveyorState>(EHardwareType::STEPPER, protocol, bus.default_id, bus.default_id, CONVEYOR_V2_HWID);
                    ROS_INFO("ConveyorInterfaceCore::addConveyor - Set NED3PRO conveyor on id %d", conveyor_id);

                    bus.type = EHardwareType::STEPPER;
                    result = bus.interface->setConveyor(conveyor_state);
                    ROS_INFO("ConveyorInterfaceCore::addConveyor - result %d", result);
                }

                // on success, we initialize the conveyor and go out of loop
                if (niryo_robot_msgs::CommandStatus::SUCCESS == result && niryo_robot_msgs::CommandStatus::SUCCESS == initHardware(conveyor_state))
                {
                    res.message = "Set new conveyor on id ";
                    res.message += to_string(conveyor_id);
                    res.message += " OK";
                    res.id = conveyor_id;
                    res.hardware_id = conveyor_state->getHardwareId();
                    res.status = static_cast<int16_t>(result);

                    ros::Duration(0.05).sleep();
                    ROS_INFO("ConveyorInterfaceCore::addConveyor - Set conveyor success");
                    break;
                }

                if (result != niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND)
                {
                    bus.interface->unsetConveyor(conveyor_state->getId(), bus.default_id);
                }

                ROS_DEBUG_COND(niryo_robot_msgs::CommandStatus::SUCCESS != result,
                               "ConveyorInterfaceCore::addConveyor - "
                               "Set conveyor failure, return : %d. Retrying (%d)...",
                               result, tries);
            }

            // on success we leave
            if (niryo_robot_msgs::CommandStatus::SUCCESS == res.status)
            {
                return res;
            }
            res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;
            res.message = "no conveyor found";
        }
        else
        {
            res.message = "no conveyor available";
            res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_LEFT;
        }
    }

    // on failure after three tries for both buses
    if (niryo_robot_msgs::CommandStatus::SUCCESS != res.status)
    {
        ROS_WARN("ConveyorInterfaceCore::addConveyor - Fail to set conveyor, message : %s, return : %d", res.message.c_str(), res.status);

        res.id = 0;
    }

    return res;
}

/**
 * @brief ConveyorInterfaceCore::initHardware
 * @param protocol
 * @param conveyor_state
 * @param new_id
 * @return
 */
int ConveyorInterfaceCore::initHardware(shared_ptr<ConveyorState> conveyor_state)
{
    ROS_DEBUG("ConveyorInterfaceCore::initHardware");

    if (!conveyor_state)
    {
        return niryo_robot_msgs::CommandStatus::FAILURE;
    }

    EBusProtocol protocol = conveyor_state->getBusProtocol();

    auto bus_config_it = _bus_config_map.find(protocol);
    if (bus_config_it == _bus_config_map.end())
    {
        ROS_ERROR("ConveyorInterfaceCore::initHardware : Conveyor bus not configured correctly");
        return niryo_robot_msgs::CommandStatus::FAILURE;
    }
    auto &bus_config = bus_config_it->second;

    // fail if no more id available
    if (!bus_config.isValid())
    {
        ROS_ERROR("ConveyorInterfaceCore::initHardware : No more id available");
        return niryo_robot_msgs::CommandStatus::FAILURE;
    }

    // change Id
    uint8_t new_id = *bus_config.pool_id_list.begin();
    auto result = bus_config.interface->changeId(conveyor_state->getHardwareType(), conveyor_state->getId(), new_id);

    if (niryo_robot_msgs::CommandStatus::SUCCESS != result)
    {
        ROS_ERROR("ConveyorInterfaceCore::initHardware: Unable to reboot motor : %d", result);
        return niryo_robot_msgs::CommandStatus::FAILURE;
    }

    // The conveyor id is updated, We can now finish the conveyor configuration

    // set some params for conveyor state
    conveyor_state->setDirection(static_cast<int8_t>(bus_config.direction));
    // specific to CAN
    if (EBusProtocol::CAN == protocol)
    {
        conveyor_state->setMaxEffort(bus_config.max_effort);
        conveyor_state->setMicroSteps(bus_config.micro_steps);
    }

    // add state to list of current connected ids
    conveyor_state->updateId(new_id);

    _conveyor_state_list.emplace_back(conveyor_state);

    // remove from pool
    bus_config.pool_id_list.erase(new_id);


    // Update the velocity profile
    auto hwType = conveyor_state->getHardwareType();
    if (hwType == EHardwareType::NED3PRO_STEPPER)
    {
        // Set operating mode
        constexpr auto CONVEYOR_NED3PRO_OPERATING_MODE = 1;
        bus_config.interface->addSingleCommandToQueue(
            std::make_unique<StepperTtlSingleCmd>(EStepperCommandType::CMD_TYPE_OPERATING_MODE, conveyor_state->getId(),
                                                                                                  std::initializer_list<uint32_t>({CONVEYOR_NED3PRO_OPERATING_MODE})));
        // Set velocity profile for the conveyor
        constexpr auto CONVEYOR_NED3PRO_ACCELERATION_PROFILE = 25;  // Arbitrary value, to fine tune
        ::common::model::VelocityProfile velocity_profile;
        velocity_profile.a_max = CONVEYOR_NED3PRO_ACCELERATION_PROFILE;
        auto interface = bus_config.interface;
        interface->addSingleCommandToQueue(
            std::make_unique<StepperTtlSingleCmd>(EStepperCommandType::CMD_TYPE_VELOCITY_PROFILE, conveyor_state->getId(), velocity_profile.to_list()));
        // TORQUE cmd
        constexpr auto CONVEYOR_TORQUE_PERCENT = 40;
        interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(EStepperCommandType::CMD_TYPE_TORQUE, conveyor_state->getId(),
                                                                                std::initializer_list<uint32_t>({static_cast<uint32_t>(CONVEYOR_TORQUE_PERCENT)})));
    }

    return result;
}

/**
 * @brief ConveyorInterfaceCore::removeConveyor
 * @param id
 * @return
 */
conveyor_interface::SetConveyor::Response ConveyorInterfaceCore::removeConveyor(uint8_t id)
{
    conveyor_interface::SetConveyor::Response res;

    // retrieve corresponding iterator in vector
    auto it = std::find_if(_conveyor_state_list.begin(), _conveyor_state_list.end(), [id](std::shared_ptr<ConveyorState> c) { return (c->getId() == id); });

    // if found
    if (_conveyor_state_list.end() == it)
    {
        ROS_INFO("Conveyor interface - Conveyor id %d not found", id);
        res.message = "Conveyor id " + to_string(id) + " not found";
        res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;
        return res;
    }

    auto conveyor_state = *it;

    EBusProtocol bus_proto = conveyor_state->getBusProtocol();

    auto bus_config_it = _bus_config_map.find(bus_proto);
    if (bus_config_it == _bus_config_map.end())
    {
        ROS_ERROR("Conveyor interface - Conveyor id %d not found", id);
        res.message = "Conveyor id " + to_string(id) + " has no protocol config";
        res.status = niryo_robot_msgs::CommandStatus::FAILURE;
        return res;
    }
    auto &bus_config = bus_config_it->second;

    // reinsert id in pool
    bus_config.pool_id_list.insert(id);

    // remove from states
    _conveyor_state_list.erase(it);

    // remove conveyor
    bus_config.interface->unsetConveyor(id, bus_config.default_id);

    res.message = "Remove conveyor id " + to_string(id);
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;

    return res;
}

/**
 * @brief ConveyorInterfaceCore::isInitialized
 * @return
 */
bool ConveyorInterfaceCore::isInitialized() { return !_bus_config_map.empty(); }

// *******************
//  callbacks
// *******************

/**
 * @brief ConveyorInterfaceCore::_callbackPingAndSetConveyor
 * @param req
 * @param res
 * @return
 */
bool ConveyorInterfaceCore::_callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req, conveyor_interface::SetConveyor::Response &res)
{
    if (!isCalibrationInProgress())
    {
        switch (req.cmd)
        {
        case conveyor_interface::SetConveyor::Request::ADD:
            res = addConveyor();
            break;

        case conveyor_interface::SetConveyor::Request::REMOVE:
        {
            std::lock_guard<std::mutex> lck(_state_map_mutex);
            res = removeConveyor(req.id);
            break;
        }
        default:
            break;
        }
        _publishConveyorsFeedback();
    }
    else
    {
        res.status = niryo_robot_msgs::CommandStatus::CALIBRATION_IN_PROGRESS;
        res.message = "Calibration in progress";
    }

    return true;
}

/**
 * @brief ConveyorInterfaceCore::_callbackControlConveyor
 * @param req
 * @param res
 * @return
 */
bool ConveyorInterfaceCore::_callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req, conveyor_interface::ControlConveyor::Response &res)
{
    if (req.speed < 0 || req.speed > 100)
    {
        res.message = "Speed value must be between 0 and 100";
        res.status = niryo_robot_msgs::CommandStatus::INVALID_PARAMETERS;
        return true;
    }

    std::lock_guard<std::mutex> lck(_state_map_mutex);

    // retrieve corresponding iterator in vector
    auto id = req.id;
    auto it = std::find_if(_conveyor_state_list.begin(), _conveyor_state_list.end(), [id](std::shared_ptr<ConveyorState> c) { return (c->getId() == id); });

    // if found
    if (it != _conveyor_state_list.end() && *it)
    {
        auto state = *it;
        EBusProtocol bus_proto = state->getBusProtocol();
        auto assembly_direction = state->getDirection();

        if (EBusProtocol::CAN == bus_proto)
        {
            _can_interface->addSingleCommandToQueue(std::make_unique<StepperSingleCmd>(
                EStepperCommandType::CMD_TYPE_CONVEYOR, req.id, std::initializer_list<int32_t>{req.control_on, req.speed, req.direction * assembly_direction}));
        }
        else if (EBusProtocol::TTL == bus_proto)
        {
            int32_t speed = req.speed;
            _ttl_interface->addSingleCommandToQueue(std::make_unique<StepperTtlSingleCmd>(
                EStepperCommandType::CMD_TYPE_CONVEYOR, req.id,
                std::initializer_list<uint32_t>{req.control_on, static_cast<uint32_t>(speed), static_cast<uint32_t>(req.direction * assembly_direction)}));
        }

        res.message = "Set command on conveyor id ";
        res.message += to_string(req.id);
        res.message += " is OK";
        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    else
    {
        res.message = "Conveyor id ";
        res.message += to_string(req.id);
        res.message += " is not set";
        res.status = niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID;
    }

    return true;
}

bool ConveyorInterfaceCore::_callbackGetHardwareId(conveyor_interface::GetHardwareId::Request &req, conveyor_interface::GetHardwareId::Response &res)
{
    std::lock_guard<std::mutex> lck(_state_map_mutex);

    // Conveyor lookup from id
    auto id = req.id;
    auto it = std::find_if(_conveyor_state_list.begin(), _conveyor_state_list.end(), [id](std::shared_ptr<ConveyorState> c) { return (c->getId() == id); });

    // Handle conveyor not found
    if (it == _conveyor_state_list.end())
    {
        res.message = "Conveyor id ";
        res.message += to_string(req.id);
        res.message += " is not set";
        res.status = niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID;
        return true;
    }

    // Conveyor found, get its hardware id
    auto conveyor = *it;
    res.hardware_id = conveyor->getHardwareId();
    res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    return true;
}

/**
 * @brief ConveyorInterfaceCore::_publishConveyorsFeedback
 */
void ConveyorInterfaceCore::_publishConveyorsFeedback()
{
    conveyor_interface::ConveyorFeedbackArray msg;
    conveyor_interface::ConveyorFeedback data;

    std::lock_guard<std::mutex> lck(_state_map_mutex);

    for (auto const &conveyor_state : _conveyor_state_list)
    {
        if (conveyor_state)
        {
            std::shared_ptr<common::util::IDriverCore> interface;
            if (conveyor_state->getBusProtocol() == EBusProtocol::CAN)
                interface = _can_interface;
            else
                interface = _ttl_interface;
            if (interface)
            {
                int cnt_scan_failed = 0;
                while (cnt_scan_failed < 3)
                {
                    if (!interface->scanMotorId(conveyor_state->getId()))
                    {
                        cnt_scan_failed++;
                        ros::Duration(0.1).sleep();
                    }
                    else
                        break;
                }
                if (cnt_scan_failed == 3)
                    removeConveyor(conveyor_state->getId());
                else
                {
                    data.conveyor_id = conveyor_state->getId();
                    data.running = conveyor_state->getState();
                    data.direction = static_cast<int8_t>(conveyor_state->getDirection() * conveyor_state->getGoalDirection());
                    data.speed = conveyor_state->getSpeed();
                    msg.conveyors.push_back(data);

                    ROS_DEBUG_THROTTLE(2.0, "ConveyorInterfaceCore::_publishConveyorsFeedback - Found a conveyor, publishing data : %s", conveyor_state->str().c_str());
                }
            }
        }
    }
    _conveyors_feedback_publisher.publish(msg);
}

/**
 * @brief ConveyorInterfaceCore::isCalibrationInProgress
 * @return
 */
bool ConveyorInterfaceCore::isCalibrationInProgress() const
{
    if (_can_interface)
        return common::model::EStepperCalibrationStatus::IN_PROGRESS == _can_interface->getCalibrationStatus();

    if (_ttl_interface)
        return common::model::EStepperCalibrationStatus::IN_PROGRESS == _ttl_interface->getCalibrationStatus();

    ROS_ERROR("ConveyorInterfaceCore::isCalibrationInProgress - No valid bus interface found");
    return false;
}

/**
 * @brief conveyor_interface::ConveyorInterfaceCore::getConveyorStates
 * @return
 */
std::vector<std::shared_ptr<common::model::ConveyorState>> conveyor_interface::ConveyorInterfaceCore::getConveyorStates() const { return _conveyor_state_list; }

}  // namespace conveyor_interface
