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
#include <memory>
#include <utility>
#include <vector>
#include <string>
#include <functional>

// ros

// niryo
#include "common/model/bus_protocol_enum.hpp"
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "niryo_robot_msgs/CommandStatus.h"


using ::std::shared_ptr;
using ::std::to_string;

using ::common::model::ConveyorState;
using ::common::model::EStepperCommandType;
using ::common::model::StepperSingleCmd;
using ::common::model::EBusProtocol;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperTtlSingleCmd;

namespace conveyor_interface
{

/**
 * @brief ConveyorInterfaceCore::ConveyorInterfaceCore
 * @param nh
 * @param ttl_interface
 * @param can_interface
 */
ConveyorInterfaceCore::ConveyorInterfaceCore(ros::NodeHandle& nh,
                                             std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface,
                                             std::shared_ptr<can_driver::CanInterfaceCore> can_interface) :
  _ttl_interface(std::move(ttl_interface)),
  _can_interface(std::move(can_interface))

{
    ROS_DEBUG("ConveyorInterfaceCore::ConveyorInterfaceCore - ctor");

    init(nh);
}

/**
 * @brief ConveyorInterfaceCore::init
 * @param nh
 */
bool ConveyorInterfaceCore::init(ros::NodeHandle& nh)
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
void ConveyorInterfaceCore::initParameters(ros::NodeHandle& nh)
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
        nh.getParam("can/default_id", default_conveyor_id);
        canConfig.default_id = static_cast<uint8_t>(default_conveyor_id);

        nh.getParam("can/pool_id_list", id_pool_list);

        std::string pool_str{"["};

        for (auto const& id : id_pool_list)
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
        ttlConfig.default_id = static_cast<uint8_t>(default_conveyor_id);

        std::string pool_str{"["};

        for (auto const& id : id_pool_list)
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
void ConveyorInterfaceCore::startServices(ros::NodeHandle& nh)
{
    _ping_and_set_stepper_server = nh.advertiseService("/niryo_robot/conveyor/ping_and_set_conveyor",
                                                        &ConveyorInterfaceCore::_callbackPingAndSetConveyor, this);

    _control_conveyor_server = nh.advertiseService("/niryo_robot/conveyor/control_conveyor",
                                                    &ConveyorInterfaceCore::_callbackControlConveyor, this);
}

/**
 * @brief ConveyorInterfaceCore::startPublishers
 */
void ConveyorInterfaceCore::startPublishers(ros::NodeHandle& nh)
{
    _conveyors_feedback_publisher = nh.advertise<conveyor_interface::ConveyorFeedbackArray>(
                                                    "/niryo_robot/conveyor/feedback", 10, true);

    _publish_conveyors_feedback_timer = nh.createTimer(ros::Duration(_publish_feedback_duration), &ConveyorInterfaceCore::_publishConveyorsFeedback, this);
}

/**
 * @brief ConveyorInterfaceCore::startSubscribers
 * @param nh
 */
void ConveyorInterfaceCore::startSubscribers(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("ConveyorInterfaceCore::startSubscribers - no subscriber to start");
}

/**
 * @brief ConveyorInterfaceCore::addConveyor
 * @return
 * scans hw on its interfaces to try to find conveyors
 * If it find one, it changes its id using the available ids in the pool
 */
conveyor_interface::SetConveyor::Response
ConveyorInterfaceCore::addConveyor()
{
    conveyor_interface::SetConveyor::Response res;
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;
    res.id = 0;

    for (auto& bus : _bus_config_map)
    {
        // if we still have available ids in the pool of ids
        if (bus.second.isValid())
        {
            // take last
            uint8_t conveyor_id = *bus.second.pool_id_list.begin();

            auto conveyor_state = std::make_shared<ConveyorState>(bus.second.type, bus.first, bus.second.default_id, bus.second.default_id);

            if (EBusProtocol::CAN == bus.first)
            {
                conveyor_state->setMaxEffort(bus.second.max_effort);
                conveyor_state->setMicroSteps(bus.second.micro_steps);
            }

            // Try 3 times
            for (int tries = 0; tries < 3; tries++)
            {
                // add conveyor to interface management
                int result = bus.second.interface->setConveyor(conveyor_state);

                if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
                {
                    // change Id
                    result = bus.second.interface->changeId(conveyor_state->getHardwareType(), conveyor_state->getId(), conveyor_id);

                    // on success, conveyor is set, we go out of loop
                    if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
                    {
                        // add state to list of current connected ids
                        conveyor_state->updateId(conveyor_id);
                        _state_map.insert(std::make_pair(conveyor_id, conveyor_state));
                        _order_insertion.push_back(conveyor_id);

                        // remove from pool
                        bus.second.pool_id_list.erase(bus.second.pool_id_list.begin());
                        res.message = "Set new conveyor on id ";
                        res.message += to_string(conveyor_id);
                        res.message += " OK";
                        res.id = conveyor_id;
                        res.status = static_cast<int16_t>(result);

                        ros::Duration(0.05).sleep();
                        ROS_INFO("ConveyorInterfaceCore::addConveyor - Set conveyor success");

                        break;
                    }
                    ros::Duration(0.01).sleep();
                }

                ROS_DEBUG_COND(niryo_robot_msgs::CommandStatus::SUCCESS != result, "ConveyorInterfaceCore::addConveyor - "
                        "Set conveyor failure, return : %d. Retrying (%d)...",
                        result, tries);

                bus.second.interface->unsetConveyor(conveyor_state->getId(), bus.second.default_id);
            }

            // on success we leave
            if (niryo_robot_msgs::CommandStatus::SUCCESS == res.status)
            {
                return res;
            }

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
        ROS_ERROR("ConveyorInterfaceCore::addConveyor - Fail to set conveyor, message : %s, return : %d",
                    res.message.c_str(),
                    res.status);

        res.id = 0;
    }

    return res;
}

/**
 * @brief ConveyorInterfaceCore::removeConveyor
 * @param id
 * @return
 */
conveyor_interface::SetConveyor::Response
ConveyorInterfaceCore::removeConveyor(uint8_t id)
{
    conveyor_interface::SetConveyor::Response res;

    if (_state_map.count(id) && _state_map.at(id))
    {
        EBusProtocol bus_proto = _state_map.at(id)->getBusProtocol();

        if (_bus_config_map.count(bus_proto))
        {
            _bus_config_map.at(bus_proto).pool_id_list.insert(id);

            // remove from states
            _state_map.erase(id);
            _order_insertion.erase(std::remove(_order_insertion.begin(), _order_insertion.end(), id), _order_insertion.end());

            // remove conveyor
            _bus_config_map.at(bus_proto).interface->unsetConveyor(id, _bus_config_map.at(bus_proto).default_id);

            res.message = "Remove conveyor id " + to_string(id);
            res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
        }
    }
    else
    {
        ROS_INFO("Conveyor interface - Conveyor id %d not found", id);
        res.message = "Conveyor id " + to_string(id) + " not found";
        res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;
    }

    return res;
}

/**
 * @brief ConveyorInterfaceCore::isInitialized
 * @return
 */
bool ConveyorInterfaceCore::isInitialized()
{
    return !_bus_config_map.empty();
}

// *******************
//  callbacks
// *******************

/**
 * @brief ConveyorInterfaceCore::_callbackPingAndSetConveyor
 * @param req
 * @param res
 * @return
 */
bool ConveyorInterfaceCore::_callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req,
                                                        conveyor_interface::SetConveyor::Response &res)
{
    if ((_ttl_interface && !_ttl_interface->isCalibrationInProgress()) ||
        (_can_interface && !_can_interface->isCalibrationInProgress()))
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
bool ConveyorInterfaceCore::_callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req,
                                                     conveyor_interface::ControlConveyor::Response &res)
{
    std::lock_guard<std::mutex> lck(_state_map_mutex);
    // if id found in list
    if (_state_map.count(req.id) && _state_map.at(req.id))
    {
        auto state = _state_map.at(req.id);
        EBusProtocol bus_proto = state->getBusProtocol();

        if (EBusProtocol::CAN == bus_proto)
        {
            _can_interface->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_CONVEYOR,
                                                                                       req.id, std::initializer_list<int32_t>{req.control_on,
                                                                                       req.speed, req.direction}));
        }
        else if (EBusProtocol::TTL == bus_proto)
        {
            _ttl_interface->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(EStepperCommandType::CMD_TYPE_CONVEYOR,
                                                                                          req.id, std::initializer_list<uint32_t>{req.control_on,
                                                                                          static_cast<uint32_t>(req.speed), static_cast<uint32_t>(req.direction)}));
        }

        res.message = "Set command on conveyor id ";
        res.message += to_string(req.id);
        res.message += " is OK";
        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
    }
    else
    {
        ROS_INFO("Conveyor interface - Conveyor id %d isn't set", req.id);
        res.message = "Conveyor id ";
        res.message += to_string(req.id);
        res.message += " is not set";
        res.status = niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID;
    }

    return true;
}

/**
 * @brief ConveyorInterfaceCore::_publishConveyorsFeedback
 */
void ConveyorInterfaceCore::_publishConveyorsFeedback(const ros::TimerEvent&)
{
    conveyor_interface::ConveyorFeedbackArray msg;
    conveyor_interface::ConveyorFeedback data;

    std::lock_guard<std::mutex> lck(_state_map_mutex);

    std::vector<uint8_t> tmp_ids = _order_insertion;
    for (auto id : tmp_ids)
    {
        auto conveyor_state = _state_map.at(id);
        if (conveyor_state)
        {
            std::shared_ptr<common::util::IDriverCore> interface;
            if (conveyor_state->getBusProtocol() == EBusProtocol::CAN)
                interface = _can_interface;
            else
                interface = _ttl_interface;
            // TODO(CC) put in ttl_manager
            if (interface)
            {
                int cnt_scan_failed = 0;
                while (cnt_scan_failed < 2)
                {
                    if (!interface->scanMotorId(conveyor_state->getId()))
                    {
                        cnt_scan_failed++;
                    }
                    else
                        break;
                }
                if (cnt_scan_failed == 2)
                    removeConveyor(conveyor_state->getId());
                else
                {
                    data.conveyor_id = conveyor_state->getId();
                    data.running = conveyor_state->getState();
                    data.direction = static_cast<int8_t>(conveyor_state->getDirection());
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
 * @brief conveyor_interface::ConveyorInterfaceCore::getConveyorStates
 * @return
 */
std::vector<std::shared_ptr<common::model::ConveyorState> >
conveyor_interface::ConveyorInterfaceCore::getConveyorStates() const
{
  std::vector<std::shared_ptr<ConveyorState> > states;
  for (const auto& it : _state_map)
  {
      states.push_back(it.second);
  }

  return states;
}

}  // namespace conveyor_interface
