/*
program_player.hpp
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_HPP

// stl
#include <chrono>  // NOLINT
#include <map>
#include <memory>
#include <string>
#include <filesystem>
#include <thread>  // NOLINT
#include <set>
#include <unordered_map>

// niryo
#include "niryo_robot_program_player/program_player_enums.hpp"
#include "niryo_robot_program_player/program_player_states.hpp"
#include "common/model/action_type_enum.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"  // included for return communication return enum

namespace niryo_robot_program_player
{
// forward declaration due to State design pattern
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class IProgramPlayerState;

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
class ProgramPlayer
{
  using StateType = IProgramPlayerState<ProgramPlayerAdapter, ProgramPlayerDriver>;

public:
  ProgramPlayer();
  ~ProgramPlayer();

  void startThread();
  void tick();
  void update(std::chrono::_V2::system_clock::time_point& next_read_time);

  void setState(ProgramPlayerState state);
  void play(const std::string& program);
  void stop();
  void initDriver();

  // getter
  common::model::EActionType getPlayStatus();
  common::model::EActionType getStopStatus();
  common::model::EActionType getUpStatus();
  common::model::EActionType getDownStatus();

  std::string getRobotName();
  std::string getCurrentProgram();
  std::map<std::string, std::string> const& getProgramList() const;
  ProgramExecutionState getProgramState() const;
  ProgramPlayerState getProgramPlayerState() const;
  std::chrono::duration<double> getNSTimeoutParam() const;
  std::chrono::time_point<std::chrono::steady_clock> getNSTimestamp() const;

  ProgramPlayerDriver& getDriver();
  ProgramPlayerAdapter& getAdapter();

  // setter
  void setCurrentProgram(const std::string& program);
  void setDisplayMessage(const std::string& line1, const std::string& line2);
  void hasError(const Error& error_type, bool value);

  // check
  bool hasError(const Error& error_type) const;
  bool isConnectedPrevious() const;

private:
  void controlLoop();

  void pingProgramPlayer();
  void checkConnectedClients();
  void checkProgramList();

  void display(std::string& line1, std::string& line2);

private:
  int _baudrate{ 1000000 };
  int _screen_size{ 16 };
  int _id{ 10 };
  std::string _program_delimiter{ "." };
  std::string _port_name;

  std::set<Error> _error;

  StateType* _current_state;

  // programs manager
  std::string _current_program{ "" };

  // display
  // TODO(i.ambit) add display effect
  std::string _line1{ "" };
  std::string _line2{ "" };

  ProgramPlayerDriver _program_player_driver;
  ProgramPlayerAdapter _program_player_adapter;

  std::thread _thread;
  bool _is_connected = false;
  bool _is_connected_previous = _is_connected;

  // States
  std::unordered_map<ProgramPlayerState, std::unique_ptr<StateType>> _states;

public:
  std::map<Button, common::model::EActionType> buttons_state{
    { Button::PLAY, common::model::EActionType::NO_ACTION },   { Button::STOP, common::model::EActionType::NO_ACTION },
    { Button::UP, common::model::EActionType::NO_ACTION },     { Button::DOWN, common::model::EActionType::NO_ACTION },
    { Button::CUSTOM, common::model::EActionType::NO_ACTION },
  };
};

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::ProgramPlayer()
{
  _id = _program_player_adapter.getId();

  // List of available states
  _states[ProgramPlayerState::IDLE] = std::make_unique<IdleState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::FAULT] = std::make_unique<FaultState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::NO_CONNECTION] =
      std::make_unique<NoConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::CONNECTION] =
      std::make_unique<ConnectionState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::STOP] = std::make_unique<StopState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::STOPPED] = std::make_unique<StoppedState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::PLAY] = std::make_unique<PlayState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::PLAYING] = std::make_unique<PlayingState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::UP] = std::make_unique<UpState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _states[ProgramPlayerState::DOWN] = std::make_unique<DownState<ProgramPlayerAdapter, ProgramPlayerDriver>>();
  _current_state = _states.at(ProgramPlayerState::NO_CONNECTION).get();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::~ProgramPlayer()
{
  if (_thread.joinable())
    _thread.join();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::startThread()
{
  _thread = std::thread(&ProgramPlayer::controlLoop, this);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::tick()
{
  checkConnectedClients();

  checkProgramList();

  // execute state machine step
  _current_state->tick(this);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::setState(ProgramPlayerState state)
{
  _current_state = _states.at(state).get();

  _current_state->execute(this);

  display(_line1, _line2);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::play(const std::string& program)
{
  _program_player_adapter.play(program);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::stop()
{
  _program_player_adapter.stop();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::initDriver()
{
  _baudrate = _program_player_adapter.getBaudrate();
  _port_name = _program_player_adapter.getPortName();

  if (std::filesystem::exists(_port_name))
    _program_player_driver.init(_baudrate, _port_name);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::controlLoop()
{
  double loop_frequency = _program_player_adapter.getLoopFrequency();

  auto current_time = std::chrono::system_clock::now();

  while (true)
  {
    auto control_loop_period_ms =
        std::chrono::system_clock::now() + std::chrono::milliseconds(static_cast<int>(1 / loop_frequency * 1000));

    update(current_time);

    std::this_thread::sleep_until(control_loop_period_ms);
  }
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::update(
    std::chrono::_V2::system_clock::time_point& next_read_time)
{
  auto current_time = std::chrono::system_clock::now();
  if (current_time >= next_read_time)
  {
    buttons_state[Button::PLAY] = getPlayStatus();
    buttons_state[Button::STOP] = getStopStatus();
    buttons_state[Button::UP] = getUpStatus();
    buttons_state[Button::DOWN] = getDownStatus();

    // if STOP has been pressed then don't read button inputs for X sec
    if (buttons_state[Button::STOP] != common::model::EActionType::NO_ACTION)
      next_read_time = current_time + std::chrono::milliseconds(_program_player_adapter.getStopButtonDebounceTime());
    else
      next_read_time = current_time;
  }

  pingProgramPlayer();

  tick();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::pingProgramPlayer()
{
  _is_connected_previous = _is_connected;
  _is_connected = _program_player_driver.ping(_id) == COMM_SUCCESS;

  hasError(Error::NO_CONNECTION, !_is_connected);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::checkConnectedClients()
{
  auto timeout = _program_player_adapter.getNiryoStudioTimeOut();
  auto timestamp = _program_player_adapter.getNiryoStudioTimeStamp();
  auto now = std::chrono::system_clock::now();
  auto duration = now - timestamp;

  auto duration_in_seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);

  hasError(Error::NIRYO_STUDIO_CONNECTED, duration_in_seconds < timeout);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::checkProgramList()
{
  auto program_list = getProgramList();
  hasError(Error::NO_PROGRAM, program_list.empty());
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::display(std::string& line1, std::string& line2)
{
  // TODO(i.ambit) fix effects
  _screen_size = _program_player_adapter.getDisplaySpecs();
  _program_delimiter = _program_player_adapter.getDelimiter();

  line1.resize(_screen_size, ' ');
  line2.resize(_screen_size, ' ');

  _program_player_driver.writeLCDLine1(_id, line1);
  _program_player_driver.writeLCDLine2(_id, line2);
}

// getter
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
common::model::EActionType ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getPlayStatus()
{
  common::model::EActionType action;
  if (_program_player_driver.readStateButtonPlay(_id, action) != COMM_SUCCESS)
  {
    hasError(Error::NO_CONNECTION, true);
    return common::model::EActionType::NO_ACTION;
  }

  hasError(Error::NO_CONNECTION, false);
  return action;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
common::model::EActionType ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getStopStatus()
{
  common::model::EActionType action;
  if (_program_player_driver.readStateButtonStop(_id, action) != COMM_SUCCESS)
  {
    hasError(Error::NO_CONNECTION, true);
    return common::model::EActionType::NO_ACTION;
  }

  hasError(Error::NO_CONNECTION, false);
  return action;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
common::model::EActionType ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getUpStatus()
{
  common::model::EActionType action;
  if (_program_player_driver.readStateButtonUp(_id, action) != COMM_SUCCESS)
  {
    hasError(Error::NO_CONNECTION, true);
    return common::model::EActionType::NO_ACTION;
  }

  hasError(Error::NO_CONNECTION, false);
  return action;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
common::model::EActionType ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getDownStatus()
{
  common::model::EActionType action;
  if (_program_player_driver.readStateButtonDown(_id, action) != COMM_SUCCESS)
  {
    hasError(Error::NO_CONNECTION, true);
    return common::model::EActionType::NO_ACTION;
  }

  hasError(Error::NO_CONNECTION, false);
  return action;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
std::string ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getRobotName()
{
  return _program_player_adapter.getRobotName();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
std::string ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getCurrentProgram()
{
  // Initialize _current_program value when list of programs is retrived
  auto program_list = _program_player_adapter.getProgramList();

  if (_current_program.empty() && !program_list.empty())
    _current_program = program_list.begin()->first;

  // current program has been deleted then show the first one in the list
  else if (program_list.find(_current_program) == program_list.end() && !program_list.empty())
    _current_program = "Program deleted";

  // no program available reset current program
  else if (program_list.empty())
    _current_program.clear();

  return _current_program;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
std::map<std::string, std::string> const&
ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getProgramList() const
{
  return _program_player_adapter.getProgramList();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramExecutionState ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getProgramState() const
{
  return _program_player_adapter.getProgramState();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerState ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getProgramPlayerState() const
{
  return _current_state->getState();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
std::chrono::duration<double> ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getNSTimeoutParam() const
{
  return _program_player_adapter.getNiryoStudioTimeOut();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
std::chrono::time_point<std::chrono::steady_clock>
ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getNSTimestamp() const
{
  return _program_player_adapter.getNiryoStudioTimeStamp();
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerDriver& ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getDriver()
{
  return _program_player_driver;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
ProgramPlayerAdapter& ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::getAdapter()
{
  return _program_player_adapter;
}

// setter
template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::setCurrentProgram(const std::string& program)
{
  auto program_list = _program_player_adapter.getProgramList();
  if (program_list.find(program) != program_list.cend())
  {
    _current_program = program;
  }
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::setDisplayMessage(const std::string& line1,
                                                                                 const std::string& line2)
{
  _line1 = line1;
  _line2 = line2;
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
void ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::hasError(const Error& error_type, bool value)
{
  if (value)
  {
    _error.insert(error_type);
  }
  else
  {
    _error.erase(error_type);
  }
}

// checks

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
bool ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::hasError(const Error& error_type) const
{
  return _error.count(error_type);
}

template <typename ProgramPlayerAdapter, typename ProgramPlayerDriver>
bool ProgramPlayer<ProgramPlayerAdapter, ProgramPlayerDriver>::isConnectedPrevious() const
{
  return _is_connected_previous;
}

}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_HPP
