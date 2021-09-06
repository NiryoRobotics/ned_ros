/*
    end_effector_interface_core.cpp
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

// c++
#include <functional>
#include <string>
#include <vector>

// ros

// niryo
#include "end_effector_interface/end_effector_interface_core.hpp"
#include "common/model/end_effector_state.hpp"
#include "end_effector_interface/ButtonStatus.h"

using ::std::lock_guard;
using ::std::mutex;
using ::std::string;
using ::std::to_string;

using ::common::model::EndEffectorState;
using ::common::model::ButtonTypeEnum;
using ::common::model::EButtonType;

namespace end_effector_interface
{

/**
 * @brief EndEffectorInterfaceCore::EndEffectorInterfaceCore
 * @param nh
 * @param ttl_interface
 */
EndEffectorInterfaceCore::EndEffectorInterfaceCore(ros::NodeHandle& nh,
                                                   std::shared_ptr<ttl_driver::TtlInterfaceCore > ttl_interface):
    _ttl_interface(ttl_interface)
{
    ROS_DEBUG("EndEffectorInterfaceCore::ctor");

    init(nh);
}

/**
 * @brief EndEffectorInterfaceCore::~EndEffectorInterfaceCore
 */
EndEffectorInterfaceCore::~EndEffectorInterfaceCore()
{
}

/**
 * @brief EndEffectorInterfaceCore::init
 * @param nh
 * @return
 */
bool EndEffectorInterfaceCore::init(ros::NodeHandle &nh)
{
    ROS_DEBUG("EndEffectorInterfaceCore::init - Initializing parameters...");
    initParameters(nh);

    ROS_DEBUG("EndEffectorInterfaceCore::initEndEffectorHardware...");
    initEndEffectorHardware();

    ROS_DEBUG("EndEffectorInterfaceCore::init - Starting services...");
    startServices(nh);

    ROS_DEBUG("EndEffectorInterfaceCore::init - Starting publishers...");
    startPublishers(nh);

    ROS_DEBUG("EndEffectorInterfaceCore::init - Starting subscribers...");
    startSubscribers(nh);

    return true;
}

/**
 * @brief EndEffectorInterfaceCore::initParameters
 * @param nh
 */
void EndEffectorInterfaceCore::initParameters(ros::NodeHandle& nh)
{
    int id = -1;
    nh.getParam("end_effector_id", id);
    _id = static_cast<uint8_t>(id);

    nh.getParam("check_end_effector_status_frequency", _check_end_effector_status_frequency);

    ROS_DEBUG("EndEffectorInterfaceCore::initParameters - end effector id : %d", _id);
    ROS_DEBUG("EndEffectorInterfaceCore::initParameters - end effector status frequency : %f", _check_end_effector_status_frequency);

    // get buttons config

    uint8_t button_id = 1;
    while (nh.hasParam("button_"  + std::to_string(button_id) + "/type") &&
           nh.hasParam("button_"  + std::to_string(button_id) + "/name") &&
           nh.hasParam("button_"  + std::to_string(button_id) + "/config"))
    {
      std::string button_type = "";
      std::string button_name = "";
      nh.getParam("button_" + std::to_string(button_id) + "/type", button_type);
      auto eType = ButtonTypeEnum(button_type.c_str());

      nh.getParam("button_" + std::to_string(button_id) + "/name", button_name);

      button_id++;
    }
}

/**
 * @brief EndEffectorInterfaceCore::startServices
 * @param nh
 */
void EndEffectorInterfaceCore::startServices(ros::NodeHandle& /*nh*/)
{
    ROS_DEBUG("No services to start");
}

/**
 * @brief EndEffectorInterfaceCore::startPublishers
 * @param nh
 */
void EndEffectorInterfaceCore::startPublishers(ros::NodeHandle& nh)
{
  _free_drive_button_state_publisher = nh.advertise<end_effector_interface::ButtonStatus>(
                                          "free_drive_button_status", 10);

  _save_pos_button_state_publisher = nh.advertise<end_effector_interface::ButtonStatus>(
                                          "save_pos_button_status", 10);

  _custom_button_state_publisher = nh.advertise<end_effector_interface::ButtonStatus>(
                                          "custom_button_status", 10);

  _publish_buttons_state_thread = std::thread(&EndEffectorInterfaceCore::_publishButtonState, this);
}

/**
 * @brief EndEffectorInterfaceCore::startSubscribers
 * @param nh
 */
void EndEffectorInterfaceCore::startSubscribers(ros::NodeHandle& /*nh*/)
{
  ROS_DEBUG("No subscribers to start");
}

/**
 * @brief EndEffectorInterfaceCore::initEndEffectorHardware
 */
void EndEffectorInterfaceCore::initEndEffectorHardware()
{
  // init driver
  if (_end_effector_state.isValid())
  {
      int result = _ttl_interface->setEndEffector(_end_effector_state.getId());

      if (niryo_robot_msgs::CommandStatus::SUCCESS == result)
      {
          ROS_INFO("ToolsInterfaceCore::ctor - Set end effector success");
      }
      else
      {
          ROS_WARN("EndEffectorInterfaceCore::sendInitEndEffectorParams - "
                   "Set end effector failure, return : %d. Aborted...",
                   result);
      }
  }
}

/**
 * @brief EndEffectorInterfaceCore::_publishButtonState
 */
void EndEffectorInterfaceCore::_publishButtonState()
{
    ros::Rate check_status_rate = ros::Rate(_check_end_effector_status_frequency);
    ButtonStatus msg;

    while (ros::ok())
    {
        lock_guard<mutex> lck(_buttons_status_mutex);

        _end_effector_state = _ttl_interface->getEndEffectorState(_id);

        for (auto const& button : _end_effector_state.getButtonsStatus())
        {
            msg.action = static_cast<int>(button.action);
            switch (button.type)
            {
                case EButtonType::FREE_DRIVE_BUTTON:
                    _free_drive_button_state_publisher.publish(msg);
                  break;
                case EButtonType::SAVE_POSITION_BUTTON:
                    _save_pos_button_state_publisher.publish(msg);
                  break;
                case EButtonType::CUSTOM_BUTTON:
                    _custom_button_state_publisher.publish(msg);
                  break;
                default:
                  break;
            }
        }

        check_status_rate.sleep();
    }
}

}  // namespace end_effector_interface
