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
#include <vector>
#include <string>
#include <functional>

// ros

// niryo
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "common/model/conveyor_state.hpp"
#include "common/model/stepper_command_type_enum.hpp"


using ::std::shared_ptr;
using ::std::to_string;
using ::std::dynamic_pointer_cast;

using ::common::model::ConveyorState;
using ::common::model::StepperMotorCmd;
using ::common::model::EStepperCommandType;

namespace conveyor_interface
{

/**
 * @brief ConveyorInterfaceCore::ConveyorInterfaceCore
 * @param nh
 * @param stepper
 */
ConveyorInterfaceCore::ConveyorInterfaceCore(ros::NodeHandle& nh,
                                             shared_ptr<can_driver::CanDriverCore> can_driver):
    _can_driver(can_driver)
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

    _nh.getParam("/niryo_robot_hardware_interface/conveyor/max_effort", _conveyor_max_effort);
    _nh.getParam("/niryo_robot_hardware_interface/conveyor/id", _default_conveyor_id);
    _nh.getParam("/niryo_robot_hardware_interface/conveyor/id_list", id_pool_list);

    _nh.getParam("/niryo_robot_hardware_interface/conveyor/publish_frequency", _publish_feedback_frequency);

    ROS_DEBUG("ConveyorInterfaceCore::initParameters - conveyor max effort : %d", _conveyor_max_effort);
    ROS_DEBUG("ConveyorInterfaceCore::initParameters - default conveyor id : %d", _default_conveyor_id);

    // debug - display info
    std::ostringstream ss;
    ss << "[";
    for (size_t i = 0; i < id_pool_list.size(); ++i)
        ss << " id " << id_pool_list.at(i) << ",";

    std::string id_pool_list_string = ss.str();
    id_pool_list_string.pop_back();  // remove last ","
    id_pool_list_string += "]";

    ROS_DEBUG("ConveyorInterfaceCore::init - conveyor pool id list: %s ", id_pool_list_string.c_str());
    ROS_DEBUG("ConveyorInterfaceCore::initParameters - Publish_hw_status_frequency : %f", _publish_feedback_frequency);

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
    _ping_and_set_stepper_server = _nh.advertiseService("/niryo_robot/conveyor/ping_and_set_conveyor",
                                                        &ConveyorInterfaceCore::_callbackPingAndSetConveyor, this);

    _control_conveyor_server = _nh.advertiseService("/niryo_robot/conveyor/control_conveyor",
                                                    &ConveyorInterfaceCore::_callbackControlConveyor, this);
}

/**
 * @brief ConveyorInterfaceCore::startPublishers
 */
void ConveyorInterfaceCore::startPublishers(ros::NodeHandle& nh)
{
    _conveyors_feedback_publisher = _nh.advertise<conveyor_interface::ConveyorFeedbackArray>(
                                                    "/niryo_robot/conveyor/feedback", 10);
    _publish_conveyors_feedback_thread = std::thread(&ConveyorInterfaceCore::_publishConveyorsFeedback, this);
}

/**
 * @brief ConveyorInterfaceCore::startSubscribers
 * @param nh
 */
void ConveyorInterfaceCore::startSubscribers(ros::NodeHandle& nh)
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
    int result = niryo_robot_msgs::CommandStatus::FAILURE;

    // if we still have available ids in the pool of ids
    if (!_conveyor_pool_id_list.empty())
    {
        // take last
        uint8_t conveyor_id = *_conveyor_pool_id_list.begin();
        result = _can_driver->setConveyor(conveyor_id, static_cast<uint8_t>(_default_conveyor_id));

        if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
        {
            // add id to list of current connected ids
            _current_conveyor_id_list.push_back(conveyor_id);
            // remove from pool
            _conveyor_pool_id_list.erase(_conveyor_pool_id_list.begin());

            StepperMotorCmd cmd(EStepperCommandType::CMD_TYPE_MICRO_STEPS, conveyor_id, {8});
            cmd.setParams({8});
            _can_driver->addSingleCommandToQueue(cmd);

            cmd = StepperMotorCmd(EStepperCommandType::CMD_TYPE_MAX_EFFORT, conveyor_id, {_conveyor_max_effort});
            _can_driver->addSingleCommandToQueue(cmd);

            cmd = StepperMotorCmd(EStepperCommandType::CMD_TYPE_CONVEYOR, conveyor_id, {false, 0, -1});
            _can_driver->addSingleCommandToQueue(cmd);

            // CC why two times in a row ?
            _can_driver->addSingleCommandToQueue(cmd);
            res.status = niryo_robot_msgs::CommandStatus::SUCCESS;

            res.message = "Set new conveyor on id ";
            res.message += to_string(conveyor_id);
            res.message += " OK";
            res.id = conveyor_id;
        }
        else
        {
            ROS_INFO("Conveyor interface - No new conveyor found");

            res.status = static_cast<int16_t>(result);
            res.id = 0;
            res.message = "No new conveyor found";
        }
    }
    else
    {
        ROS_INFO("Conveyor interface - No conveyor available");

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
        _can_driver->unsetConveyor(id);
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
    if (!_can_driver->isCalibrationInProgress())
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

    return niryo_robot_msgs::CommandStatus::SUCCESS == res.status;
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
        StepperMotorCmd cmd(EStepperCommandType::CMD_TYPE_CONVEYOR,
                            req.id,
                            {req.control_on, req.speed, req.direction});

        res.message = "Set command on conveyor id ";
        res.message += to_string(req.id);
        res.message += " is OK";
        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
        _can_driver->addSingleCommandToQueue(cmd);
    }
    else
    {
        ROS_INFO("Conveyor interface - Conveyor id %d isn't set", req.id);
        res.message = "Conveyor id ";
        res.message += to_string(req.id);
        res.message += " is not set";
        res.status = niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID;
    }

    return niryo_robot_msgs::CommandStatus::SUCCESS == res.status;
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

        // CC to be checked
        for (auto sState : _can_driver->getStepperStates())
        {
            if (sState && sState->isConveyor())
            {
                auto cState = dynamic_pointer_cast<ConveyorState>(sState);
                data.conveyor_id = cState->getId();
                data.running = cState->getState();
                data.direction = cState->getDirection();
                data.speed = cState->getSpeed();
                msg.conveyors.push_back(data);

                ROS_DEBUG("ConveyorInterfaceCore::_publishConveyorsFeedback - Found a conveyor, publishing data : %s", cState->str().c_str());
            }
        }
        _conveyors_feedback_publisher.publish(msg);
        publish_conveyor_feedback_rate.sleep();
    }
}
}  // namespace conveyor_interface
