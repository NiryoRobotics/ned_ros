/*
program_player_enums.hpp
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_ENUMS_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_ENUMS_HPP

namespace niryo_robot_program_player
{
enum class ProgramPlayerState
{
  IDLE,
  FAULT,
  NO_CONNECTION,
  CONNECTION,
  STOP,
  STOPPED,
  PLAY,
  PLAYING,
  UP,
  DOWN,
};

enum class Error
{
  NO_CONNECTION,
  NO_PROGRAM,
  NO_SERVER_AVAILABLE,
  FAILED_SERVICE_CALL,
  NIRYO_STUDIO_CONNECTED,
};

enum class ProgramExecutionState
{
  PLAYING,
  DONE,
  ERROR,
  NONE,
};

enum class Button
{
  PLAY,
  STOP,
  UP,
  DOWN,
  CUSTOM,
};

}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_ENUMS_HPP
