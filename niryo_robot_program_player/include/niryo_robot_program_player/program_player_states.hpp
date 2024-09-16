/*
program_player_states.hpp
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_STATES_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_STATES_HPP

// stl
// TODO(m.boughias) implements time management in an adapter, so we can test it deterministically
#include <chrono>  // NOLINT
#include <map>
#include <memory>
#include <string>

// niryo
#include "niryo_robot_program_player/program_player_enums.hpp"
#include "niryo_robot_program_player/program_player_states_interface.hpp"
#include "common/model/action_type_enum.hpp"

namespace niryo_robot_program_player
{
// Declarations

// IDLE STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class IdleState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// FAULT STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class FaultState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// NO_CONNECTION STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class NoConnectionState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// CONNECTION STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class ConnectionState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// STOP STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class StopState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// STOPPED STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class StoppedState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// PLAY STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class PlayState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// PLAYING STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class PlayingState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// UP STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class UpState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// DOWN STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class DownState : public IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>
{
public:
  void tick(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  void execute(ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player) override;
  ProgramPlayerState getState() override;
};

// Definitions

// IDLE STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void IdleState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else if (program_player->buttons_state[Button::STOP] != common::model::EActionType::NO_ACTION)
  {
    // For safety issues, stop has higher priority in case several buttons are pressed simultaneously, however since no
    // program is launched it does nothing
  }

  else if (program_player->buttons_state[Button::PLAY] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::PLAY);

  else if (program_player->buttons_state[Button::UP] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::UP);

  else if (program_player->buttons_state[Button::DOWN] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::DOWN);

  else
    program_player->setState(ProgramPlayerState::IDLE);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void IdleState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  program_player->setDisplayMessage(program_player->getRobotName(), program_player->getCurrentProgram());
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState IdleState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::IDLE;
}

// FAULT STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void FaultState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else
    program_player->setState(ProgramPlayerState::IDLE);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void FaultState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  program_player->stop();

  if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED))
  {
    auto msg{ "CLIENT CONNECTED" };
    program_player->setDisplayMessage(program_player->getRobotName(), msg);
  }
  else if (program_player->hasError(Error::NO_PROGRAM))
  {
    auto msg{ "NO PROGRAM" };
    program_player->setDisplayMessage(program_player->getRobotName(), msg);
  }
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState FaultState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::FAULT;
}

// NO_CONNECTION STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void NoConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else
    program_player->setState(ProgramPlayerState::CONNECTION);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void NoConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->isConnectedPrevious())
    program_player->stop();

  program_player->initDriver();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState NoConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::NO_CONNECTION;
}

// CONNECTION STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else
    program_player->setState(ProgramPlayerState::IDLE);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (!program_player->isConnectedPrevious())
    program_player->stop();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState ConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::CONNECTION;
}

// STOP STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void StopState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else
    program_player->setState(ProgramPlayerState::STOPPED);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void StopState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  program_player->stop();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState StopState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::STOP;
}

// STOPPED STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void StoppedState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else if (program_player->buttons_state[Button::STOP] != common::model::EActionType::NO_ACTION)
  {
    // For safety issues, stop has higher priority in case several buttons are pressed simultaneously, however since no
    // program is launched it does nothing
  }

  else if (program_player->buttons_state[Button::PLAY] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::PLAY);

  else if (program_player->buttons_state[Button::UP] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::UP);

  else if (program_player->buttons_state[Button::DOWN] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::DOWN);

  else
    program_player->setState(ProgramPlayerState::STOPPED);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void StoppedState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  std::string line2 = "STOP:" + program_player->getCurrentProgram();
  program_player->setDisplayMessage(program_player->getRobotName(), line2);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState StoppedState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::STOPPED;
}

// PLAY STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void PlayState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else
    program_player->setState(ProgramPlayerState::PLAYING);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void PlayState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  program_player->play(program_player->getCurrentProgram());
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState PlayState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::PLAY;
}

// PLAYING STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void PlayingState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else if (program_player->buttons_state[Button::STOP] != common::model::EActionType::NO_ACTION)
    program_player->setState(ProgramPlayerState::STOP);

  // Case where running program raises an error due to its execution
  else if (program_player->getProgramState() == ProgramExecutionState::ERROR)
    program_player->setState(ProgramPlayerState::IDLE);

  else if (program_player->getProgramState() == ProgramExecutionState::DONE)
    program_player->setState(ProgramPlayerState::IDLE);

  else
    program_player->setState(ProgramPlayerState::PLAYING);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void PlayingState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  std::string line2 = "PLAY:" + program_player->getCurrentProgram();
  program_player->setDisplayMessage(program_player->getRobotName(), line2);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState PlayingState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::PLAYING;
}

// UP STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void UpState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else
    program_player->setState(ProgramPlayerState::IDLE);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void UpState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  auto program_list = program_player->getProgramList();

  if (program_list.empty())
  {
    return;
  }

  auto current_program = program_player->getCurrentProgram();
  auto program_list_iterator = program_list.find(current_program);

  if (program_list_iterator == program_list.cend() || program_list_iterator == --program_list.cend())
  {
    program_player->setCurrentProgram((program_list.begin())->first);
  }
  else
  {
    program_player->setCurrentProgram((++program_list_iterator)->first);
  }
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState UpState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::UP;
}

// DOWN STATE
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void DownState<ProgramPlayerAdapter, ProgramPlayerDriver>::tick(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  if (program_player->hasError(Error::NO_CONNECTION))
    program_player->setState(ProgramPlayerState::NO_CONNECTION);

  else if (program_player->hasError(Error::NIRYO_STUDIO_CONNECTED) || program_player->hasError(Error::NO_PROGRAM))
    program_player->setState(ProgramPlayerState::FAULT);

  else
    program_player->setState(ProgramPlayerState::IDLE);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void DownState<ProgramPlayerAdapter, ProgramPlayerDriver>::execute(
    ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>* program_player)
{
  auto program_list = program_player->getProgramList();

  if (!program_list.empty())
  {
    auto current_program = program_player->getCurrentProgram();
    auto program_list_iterator = program_list.find(current_program);

    if (program_list_iterator == program_list.cend() || program_list_iterator == program_list.cbegin())
      program_player->setCurrentProgram((--program_list.end())->first);
    else
      program_player->setCurrentProgram((--program_list_iterator)->first);
  }
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState DownState<ProgramPlayerAdapter, ProgramPlayerDriver>::getState()
{
  return ProgramPlayerState::DOWN;
}

}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_STATES_HPP
