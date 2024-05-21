/*
    program_player_node.cpp
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

// ros
#include <ros/console.h>
#include <ros/ros.h>

// niryo
#include "niryo_robot_program_player/program_player.hpp"
#include "niryo_robot_program_player/program_player_states.hpp"
#include "niryo_robot_program_player/program_player_ros_adapter.hpp"
#include "niryo_robot_program_player/program_player_driver.hpp"
#include "niryo_robot_program_player/program_player_reg.hpp"

using namespace niryo_robot_program_player; // NOLINT

int main(int argc, char** argv)
{
  ros::init(argc, argv, "niryo_robot_program_player");

  ROS_DEBUG("Launching niryo_robot_program_player");

  ProgramPlayer<ProgramPlayerROSAdapter, ProgramPlayerDriver<ProgramPlayerReg>> program_player;
  program_player.startThread();

  ros::spin();

  ROS_INFO("niryo_robot_program_player - Shutdown node");

  return 0;
}
