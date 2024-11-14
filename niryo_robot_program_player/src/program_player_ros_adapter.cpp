/*
program_player_ros_adapter.cpp
Copyright (C) 2024 Niryo
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

#include "niryo_robot_program_player/program_player_ros_adapter.hpp"

// STL
#include <vector>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <utility>

// ROS
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

// Niryo
#include "niryo_robot_program_player/program_player_enums.hpp"
#include "niryo_robot_msgs/Trigger.h"
#include "niryo_robot_programs_manager_v2/ExecuteProgramAction.h"
#include "niryo_robot_programs_manager_v2/ProgramList.h"
#include "niryo_robot_msgs/CommandStatus.h"
#include "niryo_robot_database/GetSettings.h"

#include "common/model/action_type_enum.hpp"

using ::common::model::EActionType;

namespace niryo_robot_program_player
{
ProgramPlayerROSAdapter::ProgramPlayerROSAdapter()
{
  ROS_DEBUG("ProgramPlayerROSAdapter::ctor");

  initParameters(_nh);
  startPublishers(_nh);
  startSubscribers(_nh);
  startServices(_nh);
  startActions(_nh);
}

void ProgramPlayerROSAdapter::initParameters(ros::NodeHandle& nh)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters");

  nh.getParam(_control_loop_frequency_param, _control_loop_frequency);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %f", _control_loop_frequency_param.c_str(),
            _control_loop_frequency);

  nh.getParam(_port_name_param, _port_name);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %s", _control_loop_frequency_param.c_str(),
            _port_name.c_str());

  nh.getParam(_stop_button_debounce_time_param, _stop_button_debounce_time);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %d", _stop_button_debounce_time_param.c_str(),
            _stop_button_debounce_time);

  nh.getParam(_niryo_studio_timeout_param, _niryo_studio_timeout);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %d", _niryo_studio_timeout_param.c_str(),
            _niryo_studio_timeout);
  _niryo_studio_timeout_chrono = std::chrono::seconds(_niryo_studio_timeout);

  nh.getParam(_baudrate_param, _baudrate);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %d", _baudrate_param.c_str(), _baudrate);

  nh.getParam(_lcd_box_id_param, _id);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %d", _lcd_box_id_param.c_str(), _id);

  nh.getParam(_screen_size_param, _screen_size);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %d", _screen_size_param.c_str(), _screen_size);

  nh.getParam(_program_number_delimiter_param, _program_number_delimiter);
  ROS_DEBUG("ProgramPlayerROSAdapter::initParameters - param: %s, value: %s", _program_number_delimiter_param.c_str(),
            _program_number_delimiter.c_str());
}

void ProgramPlayerROSAdapter::startPublishers(ros::NodeHandle& nh)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::startPublishers");
}

void ProgramPlayerROSAdapter::startSubscribers(ros::NodeHandle& nh)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::startSubscribers");

  _program_execution_result_subscriber =
      nh.subscribe(_program_execution_result_name, 1, &ProgramPlayerROSAdapter::_playingProgramCallback, this);

  _program_list_subscriber =
      nh.subscribe(_program_list_subscriber_name, 1, &ProgramPlayerROSAdapter::_programListCallback, this);

  _niryo_studio_connection_subscriber = nh.subscribe(_niryo_studio_connection_subscriber_name, 1,
                                                     &ProgramPlayerROSAdapter::_niryoStudioConnectionCallback, this);
}

void ProgramPlayerROSAdapter::startServices(ros::NodeHandle& nh)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::startServices");

  _stop_program_service_client = nh.serviceClient<niryo_robot_msgs::Trigger>(_stop_program_service_client_name);

  _database_settings_get_service_client =
      nh.serviceClient<niryo_robot_database::GetSettings>(_database_settings_get_service_client_name);
}

void ProgramPlayerROSAdapter::startActions(ros::NodeHandle& nh)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::startActions");

  _start_program_action_client =
      std::make_shared<actionlib::SimpleActionClient<niryo_robot_programs_manager_v2::ExecuteProgramAction>>(
          _start_program_action_client_name, true);
}

// *********************
//      HANDLERS
// *********************

bool ProgramPlayerROSAdapter::play(const std::string& program)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::playHandler");

  if (!_start_program_action_client->waitForServer(ros::Duration(0.1)))
  {
    ROS_ERROR("ProgramPlayerROSAdapter::playHandler - service not ready");

    return false;
  }

  niryo_robot_programs_manager_v2::ExecuteProgramGoal goal;
  goal.program_id = program;
  _start_program_action_client->sendGoal(goal);

  _program_state = ProgramExecutionState::PLAYING;

  return true;
}

bool ProgramPlayerROSAdapter::stop()
{
  ROS_DEBUG("ProgramPlayerROSAdapter::stop");

  if (!_stop_program_service_client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_ERROR("ProgramPlayerROSAdapter::stop - service not ready");

    return false;
  }

  niryo_robot_msgs::Trigger srv;
  if (!_stop_program_service_client.call(srv))
  {
    ROS_ERROR("ProgramPlayerROSAdapter::stop - Service call failed");

    return false;
  }

  return true;
}

// getter
ProgramExecutionState ProgramPlayerROSAdapter::getProgramState() const
{
  return _program_state;
}

std::map<std::string, std::string> const& ProgramPlayerROSAdapter::getProgramList() const
{
  return _program_list;
}

std::chrono::time_point<std::chrono::system_clock> ProgramPlayerROSAdapter::getNiryoStudioTimeStamp()
{
  return _niryo_studio_timestamp;
}

std::chrono::seconds ProgramPlayerROSAdapter::getNiryoStudioTimeOut() const
{
  return _niryo_studio_timeout_chrono;
}

std::string ProgramPlayerROSAdapter::getRobotName()
{
  if (!_database_settings_get_service_client.waitForExistence(ros::Duration(0.1)))
    return "NO NAME";

  niryo_robot_database::GetSettings srv;
  srv.request.name = "robot_name";
  _database_settings_get_service_client.call(srv);
  return srv.response.value;
}

int ProgramPlayerROSAdapter::getBaudrate() const
{
  return _baudrate;
}

int ProgramPlayerROSAdapter::getDisplaySpecs() const
{
  return _screen_size;
}

double ProgramPlayerROSAdapter::getLoopFrequency() const
{
  return _control_loop_frequency;
}

std::string ProgramPlayerROSAdapter::getPortName() const
{
  return _port_name;
}

int ProgramPlayerROSAdapter::getStopButtonDebounceTime() const
{
  return _stop_button_debounce_time;
}

int ProgramPlayerROSAdapter::getId() const
{
  return _id;
}

std::string ProgramPlayerROSAdapter::getDelimiter() const
{
  return _program_number_delimiter;
}

// *********************
//      CALLBACKS
// *********************

void ProgramPlayerROSAdapter::_programListCallback(const niryo_robot_programs_manager_v2::ProgramList& msg)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::_programListCallback");

  _program_list.clear();
  for (const auto& program : msg.programs)
  {
    _program_list.insert(std::make_pair(program.program_id, program.name));
    ROS_DEBUG("ProgramPlayerROSAdapter::_programListCallback - %s", program.name.c_str());
  }
}

void ProgramPlayerROSAdapter::_niryoStudioConnectionCallback(const std_msgs::Empty& msg)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::_niryoStudioConnectionCallback");

  _niryo_studio_timestamp = std::chrono::system_clock::now();
}

void ProgramPlayerROSAdapter::_playingProgramCallback(
    const niryo_robot_programs_manager_v2::ExecuteProgramActionResult& msg)
{
  ROS_DEBUG("ProgramPlayerROSAdapter::_databaseSettingsUpdateCallback");

  msg.result.status == niryo_robot_msgs::CommandStatus::SUCCESS ? _program_state = ProgramExecutionState::DONE :
                                                                  _program_state = ProgramExecutionState::ERROR;

  ROS_DEBUG("ProgramPlayerROSAdapter::_databaseSettingsUpdateCallback - status: %d, msg: %s", msg.result.status,
            msg.result.message.c_str());
}

}  // namespace niryo_robot_program_player
