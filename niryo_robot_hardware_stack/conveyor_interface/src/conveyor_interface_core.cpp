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
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperTtlSingleCmd;

namespace conveyor_interface
{

/**
 * @brief ConveyorInterfaceCore::ConveyorInterfaceCore
 * @param nh
 * @param conveyor_driver
 */
ConveyorInterfaceCore::ConveyorInterfaceCore(ros::NodeHandle& nh,
                                             shared_ptr<common::model::IDriverCore> conveyor_driver):
    _conveyor_driver(conveyor_driver)
{
    ROS_DEBUG("ConveyorInterfaceCore::ConveyorInterfaceCore - ctor");
    init(nh);
}

/**
 * @brief ConveyorInterfaceCore::~ConveyorInterfaceCore
 */
ConveyorInterfaceCore::~ConveyorInterfaceCore()
{
    if (_publish_conveyors_feedback_thread.joinable())
        _publish_conveyors_feedback_thread.join();
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
    std::vector<int> id_pool_list;

    double max_effort{0.0};
    double micro_steps{8.0};
    int default_conveyor_id{1};

    std::string type;
    nh.getParam("type", type);
    EHardwareType etype = HardwareTypeEnum(type.c_str());

    if (_conveyor_driver->getBusProtocol() == common::model::EBusProtocol::CAN)
    {
        nh.getParam("can/max_effort", max_effort);
        nh.getParam("can/micro_steps", micro_steps);
        nh.getParam("can/id", default_conveyor_id);
        nh.getParam("can/id_list", id_pool_list);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - conveyor max effort : %f", max_effort);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - conveyor max effort : %f", micro_steps);
        ROS_DEBUG("ConveyorInterfaceCore::initParameters - default conveyor id : %d", default_conveyor_id);
    }
    else if (_conveyor_driver->getBusProtocol() == common::model::EBusProtocol::TTL)
    {
        nh.getParam("ttl/id", default_conveyor_id);
        nh.getParam("ttl/id_list", id_pool_list);

        ROS_DEBUG("ConveyorInterfaceCore::initParameters - default conveyor id : %d", default_conveyor_id);
    }

    nh.getParam("publish_frequency", _publish_feedback_frequency);

    for (size_t i = 0; i < id_pool_list.size(); i++)
    {
        auto _conveyor_state = std::make_shared<ConveyorState>(etype, _conveyor_driver->getBusProtocol(),
                                                                default_conveyor_id);
        // update conveyor state with information
        _conveyor_state->initialize(static_cast<uint8_t>(default_conveyor_id),
                                                        max_effort,
                                                        micro_steps);
        _conveyor_states.push_back(_conveyor_state);
    }

    // debug - display info
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < id_pool_list.size(); ++i)
        ss << " id " << id_pool_list.at(i) << ",";

    std::string id_pool_list_string = ss.str();
    id_pool_list_string.pop_back();  // remove last ","
    id_pool_list_string += "]";

    ROS_DEBUG("ConveyorInterfaceCore::init - conveyor pool id list: %s ", id_pool_list_string.c_str());
    ROS_DEBUG("ConveyorInterfaceCore::initParameters - publish feedback frequency : %f", _publish_feedback_frequency);

    // initialize pool of possible id we can assign
    for (int const id : id_pool_list)
        _conveyor_pool_id_list.insert(static_cast<uint8_t>(id));
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
                                                    "/niryo_robot/conveyor/feedback", 10);
    _publish_conveyors_feedback_thread = std::thread(&ConveyorInterfaceCore::_publishConveyorsFeedback, this);
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
 */
conveyor_interface::SetConveyor::Response
ConveyorInterfaceCore::addConveyor()
{
    conveyor_interface::SetConveyor::Response res;
    res.status = niryo_robot_msgs::CommandStatus::FAILURE;

    // if we still have available ids in the pool of ids
    if (!_conveyor_pool_id_list.empty())
    {
        // take last
        uint8_t conveyor_id = *_conveyor_pool_id_list.begin();

        auto state = std::find_if(_conveyor_states.begin(), _conveyor_states.end(), [](const std::shared_ptr<ConveyorState> state){
                                                                                            if (state->getDefaultId() == state->getId())
                                                                                                return true;
                                                                                            return false;
                                                                                            });
        if (state != _conveyor_states.end() && *state)
        {
            auto conveyor_state = *state;

            // if new tool has been found
            if (conveyor_state->isValid())
            {
                // set and init conveyor
                if (_conveyor_driver->getBusProtocol() == EBusProtocol::CAN)
                {
                    conveyor_state->updateId(conveyor_id);
                }
                // Try 3 times
                for (int tries = 0; tries < 3; tries++)
                {
                    int result = _conveyor_driver->setConveyor(conveyor_state);

                    // change Id
                    if (result == niryo_robot_msgs::CommandStatus::SUCCESS && _conveyor_driver->getBusProtocol() == EBusProtocol::TTL)
                    {
                        conveyor_state->updateId(conveyor_id);
                        result = _conveyor_driver->changeId(conveyor_state->getHardwareType(), conveyor_state->getDefaultId(), conveyor_state->getId());
                    }

                    // on success, conveyor is set, we go out of loop
                    if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
                    {
                        // add id to list of current connected ids
                        _current_conveyor_id_list.push_back(conveyor_id);
                        // remove from pool
                        _conveyor_pool_id_list.erase(_conveyor_pool_id_list.begin());
                        res.message = "Set new conveyor on id ";
                        res.message += to_string(conveyor_id);
                        res.message += " OK";
                        res.id = conveyor_id;
                        res.status = static_cast<int16_t>(result);

                        ros::Duration(0.05).sleep();
                        ROS_INFO("ConveyorInterfaceCore::addConveyor - Set conveyor success");

                        break;
                    }
                    else
                    {
                        ROS_WARN("ConveyorInterfaceCore::addConveyor - "
                                "Set conveyor failure, return : %d. Retrying (%d)...",
                                result, tries);
                        _conveyor_driver->unsetConveyor(conveyor_state->getId());
                    }
                    ros::Duration(0.01).sleep();
                }

                // on failure after three tries
                if (niryo_robot_msgs::CommandStatus::SUCCESS != res.status)
                {
                    conveyor_state->updateId(conveyor_state->getDefaultId());
                    ROS_ERROR("ConveyorInterfaceCore::addConveyor - Fail to set conveyor, return : %d",
                                res.status);

                    res.id = 0;
                }
            }
            else  // no conveyor found, no conveyor set (it is not an error, the conveyor does not exists)
            {
                ROS_INFO("ConveyorInterfaceCore::addConveyor - No new conveyor state for new conveyor");

                res.id = 0;
                res.message = "No new conveyor state found";
            }
        }
        else
        {
            ROS_INFO("ConveyorInterfaceCore::addConveyor - No conveyor available");

            res.message = "no conveyor available";
            res.id = 0;
            res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_LEFT;
        }
    }
    else  // no conveyor available
    {
        ROS_INFO("ConveyorInterfaceCore::addConveyor - No conveyor available");

        res.message = "no conveyor available";
        res.id = 0;
        res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_LEFT;
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

    // find id in vector
    auto position = find(_current_conveyor_id_list.begin(), _current_conveyor_id_list.end(), id);

    // found
    if (position != _current_conveyor_id_list.end())
    {
        _conveyor_pool_id_list.insert(id);

        // remove from currently connected conveyors
        _current_conveyor_id_list.erase(position);
        // remove conveyor
        _conveyor_driver->unsetConveyor(id);
        // get back init state of conveyor
        auto state = std::find_if(_conveyor_states.begin(), _conveyor_states.end(), [id](const std::shared_ptr<ConveyorState> state){
                                                                                            if (state->getId() == id)
                                                                                                return true;
                                                                                            return false;
                                                                                            });
        (*state)->updateId((*state)->getDefaultId());

        res.message = "Remove conveyor id " + to_string(id);
        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
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
    return !_conveyor_pool_id_list.empty();
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
    if (!_conveyor_driver->isCalibrationInProgress())
    {
        switch (req.cmd)
        {
            case conveyor_interface::SetConveyor::Request::ADD:
                res = addConveyor();
                break;

            case conveyor_interface::SetConveyor::Request::REMOVE:
                res = removeConveyor(req.id);
                break;

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
    // if id found in list
    if (find(_current_conveyor_id_list.begin(), _current_conveyor_id_list.end() , req.id)
            != _current_conveyor_id_list.end())
    {
        auto state = std::find_if(_conveyor_states.begin(), _conveyor_states.end(), [req](const std::shared_ptr<ConveyorState> state){
                                                                                            if (state->getId() == req.id)
                                                                                                return true;
                                                                                            return false;
                                                                                            });
        if (*state && (*state)->getBusProtocol() == EBusProtocol::CAN)
        {
            _conveyor_driver->addSingleCommandToQueue(std::make_shared<StepperSingleCmd>(EStepperCommandType::CMD_TYPE_CONVEYOR,
                                                                            req.id, std::initializer_list<int32_t>{req.control_on,
                                                                            req.speed, req.direction}));
        }
        else if (*state && (*state)->getBusProtocol() == EBusProtocol::TTL)
        {
            _conveyor_driver->addSingleCommandToQueue(std::make_shared<StepperTtlSingleCmd>(EStepperCommandType::CMD_TYPE_CONVEYOR,
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
void ConveyorInterfaceCore::_publishConveyorsFeedback()
{
    ROS_DEBUG("ConveyorInterfaceCore::_publishConveyorsFeedback - start ros loop");
    ros::Rate publish_conveyor_feedback_rate = ros::Rate(_publish_feedback_frequency);

    while (ros::ok())
    {
        conveyor_interface::ConveyorFeedbackArray msg;
        conveyor_interface::ConveyorFeedback data;

        for (auto conveyor_state : _conveyor_states)
        {
            if (std::find(_current_conveyor_id_list.begin(), _current_conveyor_id_list.end(), conveyor_state->getId()) != _current_conveyor_id_list.end())
            {
                data.conveyor_id = conveyor_state->getId();
                data.running = conveyor_state->getState();

                // TODO(CC) implicit conversion loses integer precision
                data.direction = static_cast<int8_t>(conveyor_state->getDirection());
                data.speed = conveyor_state->getSpeed();
                msg.conveyors.push_back(data);

                ROS_DEBUG("ConveyorInterfaceCore::_publishConveyorsFeedback - Found a conveyor, publishing data : %s", conveyor_state->str().c_str());
            }
        }
        _conveyors_feedback_publisher.publish(msg);
        publish_conveyor_feedback_rate.sleep();
    }
}
}  // namespace conveyor_interface
