/*
program_player_ros_adapter.hpp
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_ROS_ADAPTER_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_ROS_ADAPTER_HPP

// ros
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

// stl
#include <array>
#include <chrono>  // NOLINT
#include <map>
#include <memory>
#include <thread>  // NOLINT
#include <string>

// niryo
#include "niryo_robot_program_player/program_player_enums.hpp"
#include "niryo_robot_database/GetSettings.h"
#include "niryo_robot_programs_manager_v2/ExecuteProgramAction.h"
#include "niryo_robot_programs_manager_v2/ProgramList.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "niryo_robot_program_player/ButtonsStates.h"
#include "niryo_robot_program_player/DisplayMessage.h"
#include "niryo_robot_program_player/LedsStates.h"

namespace niryo_robot_program_player
{

// TODO(i.ambit) try to use the observer pattern instead of getters
class ProgramPlayerROSAdapter
{
private:
public:
  ProgramPlayerROSAdapter();

  ProgramExecutionState getProgramState() const;
  std::string getRobotName();
  std::map<std::string, std::string> const& getProgramList() const;
  std::chrono::time_point<std::chrono::system_clock> getNiryoStudioTimeStamp();
  std::chrono::seconds getNiryoStudioTimeOut() const;

  // TODO(i.ambit) create struc to encapsulate device specs
  // program player device specs
  int getBaudrate() const;
  int getDisplaySpecs() const;
  double getLoopFrequency() const;
  std::string getPortName() const;
  int getStopButtonDebounceTime() const;
  int getId() const;
  std::string getDelimiter() const;
  // program player device specs

  // handler functions
  bool play(const std::string& program);
  bool stop();

private:
  void initParameters(ros::NodeHandle& nh);
  void startPublishers(ros::NodeHandle& nh);
  void startSubscribers(ros::NodeHandle& nh);
  void startServices(ros::NodeHandle& nh);
  void startActions(ros::NodeHandle& nh);

  // callbacks
  void _programListCallback(const niryo_robot_programs_manager_v2::ProgramList& msg);
  void _niryoStudioConnectionCallback(const std_msgs::Empty& msg);
  void _playingProgramCallback(const niryo_robot_programs_manager_v2::ExecuteProgramActionResult& msg);

private:
  // ROS PARAMS

  const std::string _control_loop_frequency_param{ "/niryo_robot_program_player/control_loop_frequency" };
  double _control_loop_frequency{ 30 };

  const std::string _port_name_param{ "/niryo_robot_program_player/port_name" };
  std::string _port_name{ "" };

  const std::string _stop_button_debounce_time_param{ "/niryo_robot_program_player/stop_button_debounce_time" };
  int _stop_button_debounce_time{ 500 };

  const std::string _niryo_studio_timeout_param{ "/niryo_robot_program_player/niryo_studio_timeout" };
  int _niryo_studio_timeout{ 3 };

  const std::string _baudrate_param{ "/niryo_robot_program_player/baudrate" };
  int32_t _baudrate;

  const std::string _lcd_box_id_param{ "/niryo_robot_program_player/lcd_box_id" };
  int _id{ 10 };

  const std::string _screen_size_param{ "/niryo_robot_program_player/screen_size" };
  int32_t _screen_size{ 16 };

  const std::string _program_number_delimiter_param{ "/niryo_robot_program_player/program_number_delimiter" };
  std::string _program_number_delimiter{ "." };

  // ROS PUBLISHERS

  // ROS SUBSCRIBERS

  const std::string _program_execution_result_name{ "/niryo_robot_programs_manager_v2/execute_program/result" };
  ros::Subscriber _program_execution_result_subscriber;

  const std::string _program_list_subscriber_name{ "/niryo_robot_programs_manager_v2/program_list" };
  ros::Subscriber _program_list_subscriber;

  const std::string _niryo_studio_connection_subscriber_name{ "niryo_studio_connection" };
  ros::Subscriber _niryo_studio_connection_subscriber;

  // ROS SERVICES
  const std::string _stop_program_service_client_name{ "/niryo_robot_programs_manager_v2/stop_execution" };
  ros::ServiceClient _stop_program_service_client;

  const std::string _database_settings_get_service_client_name{ "/niryo_robot_database/settings/get" };
  ros::ServiceClient _database_settings_get_service_client;

  // ROS ACTIONS
  const std::string _start_program_action_client_name{ "/niryo_robot_programs_manager_v2/execute_program" };
  std::shared_ptr<actionlib::SimpleActionClient<niryo_robot_programs_manager_v2::ExecuteProgramAction>>
      _start_program_action_client;

private:
  // NS connection
  std::chrono::seconds _niryo_studio_timeout_chrono;
  std::chrono::time_point<std::chrono::system_clock> _niryo_studio_timestamp;

  // button box comm
  std::string _robot_name{ "NO NAME" };

  // programs manager
  std::map<std::string, std::string> _program_list;

  // program status
  ProgramExecutionState _program_state;

  ros::NodeHandle _nh;
};

}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_ROS_ADAPTER_HPP
