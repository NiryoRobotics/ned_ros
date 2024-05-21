/*
program_player_states_interfaces.hpp
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_STATES_INTERFACE_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_STATES_INTERFACE_HPP

// stl
#include <memory>

// niryo
#include "niryo_robot_program_player/program_player_enums.hpp"

namespace niryo_robot_program_player
{
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class ProgramPlayer;

// Interface for all states
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class IProgramPlayerState
{
public:
  virtual void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) = 0;

  virtual void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) = 0;

  virtual ProgramPlayerState getState() = 0;

  virtual ~IProgramPlayerState()
  {
  }
};
}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_STATES_INTERFACE_HPP
