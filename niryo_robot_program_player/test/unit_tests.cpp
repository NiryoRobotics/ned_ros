/*
    unit_tests.cpp
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

// STL
#include <cassert>
#include <gtest/gtest.h>
#include <memory>
#include <ros/console.h>
#include <string>
#include <utility>
#include <thread>  // NOLINT
#include <chrono>  // NOLINT
#include <map>

// ROS
#include <ros/ros.h>

// niryo
#include "niryo_robot_program_player/program_player_enums.hpp"
#include "niryo_robot_program_player/program_player.hpp"
#include "niryo_robot_program_player/program_player_states.hpp"

using namespace ::niryo_robot_program_player;  // NOLINT

class ProgramPlayerMockAdapter
{
public:
  std::string getRobotName()
  {
    return "ned2";
  }

  int getBaudrate() const
  {
    return _baudrate;
  }
  std::string getPortName() const
  {
    return _port_name;
  }
  int getDisplaySpecs() const
  {
    return _screen_size;
  }
  int getLoopFrequency() const
  {
    return _control_loop_frequency;
  }
  int getId() const
  {
    return _id;
  }
  std::string getDelimiter() const
  {
    return _program_number_delimiter;
  }

  void setProgramList(const std::map<std::string, std::string>& program_list)
  {
    _program_list = program_list;
  }

  std::map<std::string, std::string> const& getProgramList() const
  {
    return _program_list;
  }

  std::chrono::seconds getNiryoStudioTimeOut() const
  {
    return std::chrono::seconds(3);
  }

  void setNiryoStudioTimeStamp(const std::chrono::time_point<std::chrono::system_clock>& time_stamp)
  {
    _niryo_studio_timestamp = time_stamp;
  }

  std::chrono::time_point<std::chrono::system_clock> getNiryoStudioTimeStamp()
  {
    return _niryo_studio_timestamp;
  }

  int getStopButtonDebounceTime() const
  {
    return _stop_button_debounce_time;
  }

  void stop()
  {
  }

  void play(const std::string& program)
  {
  }

  void setProgramState(const ProgramExecutionState& program_state)
  {
    _program_state = program_state;
  }

  ProgramExecutionState getProgramState() const
  {
    return _program_state;
  }

private:
  ProgramExecutionState _program_state{ ProgramExecutionState::NONE };
  std::map<std::string, std::string> _program_list;
  std::chrono::time_point<std::chrono::system_clock> _niryo_studio_timestamp;
  int _baudrate{ 1000000 };
  int _id{ 10 };
  int _stop_button_debounce_time{ 500 };
  std::string _port_name{ "/dev/dummy_port" };
  int32_t _screen_size{ 16 };
  std::string _program_number_delimiter{ "." };
  double _control_loop_frequency{ 30 };
};

class ProgramPlayerMockDriver
{
public:
  void init(const int& baudrate, const std::string& port)
  {
  }

  void setPlayValue(const common::model::EActionType& value)
  {
    _play = value;
  }
  void setStopValue(const common::model::EActionType& value)
  {
    _stop = value;
  }
  void setUpValue(const common::model::EActionType& value)
  {
    _up = value;
  }
  void setDownValue(const common::model::EActionType& value)
  {
    _down = value;
  }
  void setCommunicationCode(const int& code)
  {
    _code = code;
  }

  int ping(uint8_t id)
  {
    return _code;
  }
  int readStateButtonPlay(uint8_t id, common::model::EActionType& action)
  {
    action = _play;
    return _code;
  }
  int readStateButtonStop(uint8_t id, common::model::EActionType& action)
  {
    action = _stop;
    return _code;
  }
  int readStateButtonUp(uint8_t id, common::model::EActionType& action)
  {
    action = _up;
    return _code;
  }
  int readStateButtonDown(uint8_t id, common::model::EActionType& action)
  {
    action = _down;
    return _code;
  }

  int writeLCDLine1(uint8_t id, const std::string& display_text)
  {
    _display_text_line1 = display_text;
    return _code;
  }
  int writeLCDLine2(uint8_t id, const std::string& display_text)
  {
    _display_text_line2 = display_text;
    return _code;
  }

public:
  // Spy functions
  std::string getDisplayTextLine1()
  {
    return _display_text_line1;
  }

  std::string getDisplayTextLine2()
  {
    return _display_text_line2;
  }

private:
  common::model::EActionType _play{ common::model::EActionType::NO_ACTION };
  common::model::EActionType _stop{ common::model::EActionType::NO_ACTION };
  common::model::EActionType _up{ common::model::EActionType::NO_ACTION };
  common::model::EActionType _down{ common::model::EActionType::NO_ACTION };
  int _code{ COMM_SUCCESS };
  std::string _display_text_line1;
  std::string _display_text_line2;
};

/******************************************************/
/************** Tests program player ******************/
/******************************************************/
/**
 * @brief The TtlManagerTestSuite class
 */
class ProgramPlayerTestSuite : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // reset button action values
    program_player = std::make_unique<ProgramPlayer<ProgramPlayerMockAdapter, ProgramPlayerMockDriver>>();

    ProgramPlayerMockDriver& driver = program_player->getDriver();
    driver.setPlayValue(common::model::EActionType::NO_ACTION);
    driver.setStopValue(common::model::EActionType::NO_ACTION);
    driver.setUpValue(common::model::EActionType::NO_ACTION);
    driver.setDownValue(common::model::EActionType::NO_ACTION);

    driver.setCommunicationCode(COMM_SUCCESS);

    program_list = { std::make_pair("unique-name1", "program1"), std::make_pair("unique-name2", "program2"),
                     std::make_pair("unique-name3", "program3") };
    ProgramPlayerMockAdapter& adapter = program_player->getAdapter();
    adapter.setProgramList(program_list);
    program_player->tick();
  }

  void TearDown() override
  {
    program_player = nullptr;
  }

  std::unique_ptr<ProgramPlayer<ProgramPlayerMockAdapter, ProgramPlayerMockDriver>> program_player;
  std::map<std::string, std::string> program_list;
};

// Get robot name
TEST_F(ProgramPlayerTestSuite, get_robot_name)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  ASSERT_EQ(program_player->getRobotName(), adapter.getRobotName());
}

TEST_F(ProgramPlayerTestSuite, no_connection_to_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::NO_CONNECTION);

  // Connected
  driver.setCommunicationCode(COMM_SUCCESS);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, connection_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::CONNECTION);

  // No connection
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, connection_to_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::CONNECTION);

  // Connected
  driver.setCommunicationCode(COMM_SUCCESS);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, idle_to_fault_with_ns_connected)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::IDLE);

  auto timestamp = std::chrono::system_clock::now();
  adapter.setNiryoStudioTimeStamp(timestamp);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, idle_to_fault_with_no_programs)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::IDLE);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, idle_to_play)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::IDLE);

  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);
}

TEST_F(ProgramPlayerTestSuite, play_to_playing)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::PLAY);

  driver.setPlayValue(common::model::EActionType::NO_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);
}

TEST_F(ProgramPlayerTestSuite, playing_to_stop)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::PLAYING);

  driver.setStopValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::STOP);
}

TEST_F(ProgramPlayerTestSuite, stop_to_stopped)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::STOP);

  driver.setStopValue(common::model::EActionType::NO_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::STOPPED);
}

TEST_F(ProgramPlayerTestSuite, stopped_to_play)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::STOPPED);

  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);
}

TEST_F(ProgramPlayerTestSuite, stopped_to_up)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::STOPPED);

  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::UP);
}

TEST_F(ProgramPlayerTestSuite, stopped_to_down)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::STOPPED);

  driver.setDownValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::DOWN);
}

TEST_F(ProgramPlayerTestSuite, idle_to_up)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::IDLE);

  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::UP);
}

TEST_F(ProgramPlayerTestSuite, up_to_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::UP);

  driver.setUpValue(common::model::EActionType::NO_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, idle_to_down)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::IDLE);

  driver.setDownValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::DOWN);
}

TEST_F(ProgramPlayerTestSuite, down_to_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::DOWN);

  driver.setDownValue(common::model::EActionType::NO_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, down_to_fault)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::DOWN);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, up_to_fault)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::UP);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, play_to_fault)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::PLAY);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, playing_to_fault)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::PLAYING);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, stop_to_fault)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::STOP);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, stopped_to_fault)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->setState(ProgramPlayerState::STOPPED);

  program_list.clear();
  adapter.setProgramList(program_list);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);
}

TEST_F(ProgramPlayerTestSuite, idle_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::IDLE);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, down_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::DOWN);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, up_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::UP);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, stop_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::STOP);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, stopped_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::STOPPED);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, playing_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::PLAYING);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, play_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::PLAY);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, fault_to_no_connection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::FAULT);

  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::NO_CONNECTION);
}

TEST_F(ProgramPlayerTestSuite, get_program_list)
{
  ASSERT_EQ(program_player->getProgramList(), program_list);
}

// Get current program
TEST_F(ProgramPlayerTestSuite, get_current_program)
{
  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);
}

// Up button Program selection
TEST_F(ProgramPlayerTestSuite, up_button_program_selection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::IDLE);

  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);

  // up button
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getCurrentProgram(), (++program_list.begin())->first);

  // idle
  current_time = std::chrono::system_clock::now();
  program_player->update(current_time);

  // up button twice, overflow list go to first program
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  current_time = std::chrono::system_clock::now();
  program_player->update(current_time);

  // idle
  current_time = std::chrono::system_clock::now();
  program_player->update(current_time);

  // up button twice, overflow list go to first program
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);
}

// Down button Program selection
TEST_F(ProgramPlayerTestSuite, down_button_program_selection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->setState(ProgramPlayerState::IDLE);

  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);

  // down button
  driver.setDownValue(common::model::EActionType::SINGLE_PUSH_ACTION);

  auto current_time = std::chrono::system_clock::now();
  program_player->update(current_time);
  ASSERT_EQ(program_player->getCurrentProgram(), (--program_list.end())->first);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
