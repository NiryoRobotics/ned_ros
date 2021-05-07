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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//c++
#include <functional>

//ros

//niryo
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "model/conveyor_state.hpp"
#include "model/stepper_command_type_enum.hpp"

using namespace std;
using namespace common::model;

namespace ConveyorInterface {

    /**
     * @brief ConveyorInterfaceCore::ConveyorInterfaceCore
     * @param stepper
     */
    ConveyorInterfaceCore::ConveyorInterfaceCore(shared_ptr<StepperDriver::StepperDriverCore> stepper):
       _stepper(stepper)
    {
        init();

        ROS_DEBUG("ConveyorInterfaceCore::ConveyorInterfaceCore - ctor");
    }

    /**
     * @brief ConveyorInterfaceCore::~ConveyorInterfaceCore
     */
    ConveyorInterfaceCore::~ConveyorInterfaceCore()
    {
        if(_publish_conveyors_feedback_thread.joinable())
            _publish_conveyors_feedback_thread.join();
    }

    /**
     * @brief ConveyorInterfaceCore::init
     */
    void ConveyorInterfaceCore::init()
    {
        initParams();

        ROS_DEBUG("ConveyorInterfaceCore::init - Starting services...");
        startServices();

        ROS_DEBUG("ConveyorInterfaceCore::init - Starting publishers...");
        startPublishers();
    }

    /**
     * @brief ConveyorInterfaceCore::initParams
     */
    void ConveyorInterfaceCore::initParams()
    {
        _nh.getParam("/niryo_robot_hardware_interface/conveyor/max_effort", _conveyor_max_effort);
        _nh.getParam("/niryo_robot_hardware_interface/conveyor/id", _conveyor_id);
        _nh.getParam("/niryo_robot_hardware_interface/conveyor/id_list", _list_possible_conveyor_id);
        _nh.getParam("/niryo_robot_hardware_interface/conveyor/publish_frequency", _publish_feedback_frequency);

        ROS_DEBUG("ConveyorInterfaceCore::initParams - conveyor max effort : %d", _conveyor_max_effort);
        ROS_DEBUG("ConveyorInterfaceCore::initParams - Publish_hw_status_frequency : %f", _publish_feedback_frequency);
        _available_conveyor_list.push_back(static_cast<uint8_t>(_list_possible_conveyor_id.at(1)));
        _available_conveyor_list.push_back(static_cast<uint8_t>(_list_possible_conveyor_id.at(0)));
    }

    /**
     * @brief ConveyorInterfaceCore::startServices
     */
    void ConveyorInterfaceCore::startServices()
    {
        _ping_and_set_stepper_server = _nh.advertiseService("/niryo_robot/conveyor/ping_and_set_conveyor", &ConveyorInterfaceCore::_callbackPingAndSetConveyor, this);
        _control_conveyor_server = _nh.advertiseService("/niryo_robot/conveyor/control_conveyor", &ConveyorInterfaceCore::_callbackControlConveyor, this);
    }

    /**
     * @brief ConveyorInterfaceCore::startPublishers
     */
    void ConveyorInterfaceCore::startPublishers()
    {
        _conveyors_feedback_publisher = _nh.advertise<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 10);
        _publish_conveyors_feedback_thread = std::thread(&ConveyorInterfaceCore::_publishConveyorsFeedback, this);
    }

    //*******************
    //  callbacks
    //*******************

    /**
     * @brief ConveyorInterfaceCore::_callbackPingAndSetConveyor
     * @param req
     * @param res
     * @return
     */
    bool ConveyorInterfaceCore::_callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req, conveyor_interface::SetConveyor::Response &res)
    {
        string message = "";
        int result = niryo_robot_msgs::CommandStatus::FAILURE;

        if (!_stepper->isCalibrationInProgress())
        {
            switch(req.cmd)
            {
            case conveyor_interface::SetConveyor::Request::ADD:
            {
                if(!_available_conveyor_list.empty())
                {
                    uint8_t conveyor_id = _available_conveyor_list.back();
                    result = _stepper->setConveyor(conveyor_id);

                    if(niryo_robot_msgs::CommandStatus::SUCCESS == result)
                    {
                        _list_conveyor_id.push_back(conveyor_id);
                        _available_conveyor_list.pop_back();
                        ros::Duration(0.05).sleep();

                        // cc should we use stepper directly ?
                        StepperMotorCmd cmd(EStepperCommandType::CMD_TYPE_MICRO_STEPS, conveyor_id, {8});
                        cmd.setParams({8});
                        _stepper->addSingleCommandToQueue(cmd);
                        ros::Duration(0.05).sleep();

                        cmd = StepperMotorCmd(EStepperCommandType::CMD_TYPE_MAX_EFFORT, conveyor_id, {_conveyor_max_effort});
                        _stepper->addSingleCommandToQueue(cmd);
                        ros::Duration(0.1).sleep();

                        cmd = StepperMotorCmd(EStepperCommandType::CMD_TYPE_CONVEYOR, conveyor_id, {false, 0, -1});
                        _stepper->addSingleCommandToQueue(cmd);
                        ros::Duration(0.1).sleep();
                        // CC why two times in a row ?
                        _stepper->addSingleCommandToQueue(cmd);
                        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;

                        message = "Set new conveyor on id ";
                        message += to_string(conveyor_id);
                        message += " OK";
                        res.id = conveyor_id;
                    }
                    else
                    {
                        ROS_INFO("Conveyor interface - No new conveyor found");

                        res.status = static_cast<int16_t>(result);
                        res.id = 0;
                        message = "No new conveyor found";
                    }
                }
                else
                {
                    ROS_INFO("Conveyor interface - No conveyor available");

                    res.message = "no conveyor available";
                    res.id = 0;
                    res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_LEFT;
                    return true;
                }

            }
            break;

            case conveyor_interface::SetConveyor::Request::REMOVE:
            {
                bool conveyor_found = false;
                for(size_t i = 0; i < _list_conveyor_id.size(); i++)
                {
                    if(req.id == _list_conveyor_id.at(i))
                    {
                        conveyor_found = true;
                        _list_conveyor_id.erase( _list_conveyor_id.begin() + i);
                        _available_conveyor_list.push_back(req.id);
                        _stepper->unsetConveyor(req.id);
                        sort(_available_conveyor_list.begin(), _available_conveyor_list.end(), greater<int>());
                        message = "Remove conveyor id ";
                        message += to_string(req.id);
                        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
                        break;
                    }
                }

                if(!conveyor_found)
                {
                    ROS_INFO("Conveyor interface - Conveyor id %d not found", req.id);
                    message = "Conveyor id ";
                    message += to_string(req.id);
                    message += " not found";
                    res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;
                }
            }
            res.message = message;
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
    bool ConveyorInterfaceCore::_callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req, conveyor_interface::ControlConveyor::Response &res)
    {
        string message = "";
        auto conveyor_id = find(_list_conveyor_id.begin(), _list_conveyor_id.end() , req.id);
        if(conveyor_id != _list_conveyor_id.end())
        {
            StepperMotorCmd cmd(EStepperCommandType::CMD_TYPE_CONVEYOR,
                                req.id,
                                {req.control_on, req.speed, req.direction});

            message = "Set command on conveyor id ";
            message += to_string(req.id);
            message += " is OK";
            res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
            _stepper->addSingleCommandToQueue(cmd);
        }
        else
        {
            ROS_INFO("Conveyor interface - Conveyor id %d isn't set", req.id);
            message = "Conveyor id ";
            message += to_string(req.id);
            message += " is not set";
            res.status = niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID;
        }

        res.message = message;
        return true;
    }

    /**
     * @brief ConveyorInterfaceCore::_publishConveyorsFeedback
     */
    void ConveyorInterfaceCore::_publishConveyorsFeedback()
    {
        ros::Rate publish_conveyor_feedback_rate = ros::Rate(_publish_feedback_frequency);

        while (ros::ok()) {

            conveyor_interface::ConveyorFeedbackArray msg;
            conveyor_interface::ConveyorFeedback data;

            // CC to be checked
            for(auto& sState : _stepper->getStepperStates())
            {
                if(sState.isConveyor()) {
                    ConveyorState& cState = dynamic_cast<ConveyorState &>(sState);
                    data.conveyor_id = cState.getId();
                    data.running = cState.getState();
                    data.direction = static_cast<int8_t>(cState.getDirection());
                    data.speed = cState.getSpeed();
                    msg.conveyors.push_back(data);
                }
            }
            _conveyors_feedback_publisher.publish(msg);
            publish_conveyor_feedback_rate.sleep();
        }
    }
} //ConveyorInterface
