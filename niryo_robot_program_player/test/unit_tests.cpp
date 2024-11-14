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

using namespace ::niryo_robot_program_player; // NOLINT

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

TEST_F(ProgramPlayerTestSuite, idle_stop_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // stop button
  driver.setStopValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::STOP);

  // idle
  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, idle_stop_fault_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // stop button
  driver.setStopValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::STOP);

  // fault
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

// IDLE -> FAULT -> IDLE
TEST_F(ProgramPlayerTestSuite, idle_fault_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // fault
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

// IDLE -> UP -> IDLE
TEST_F(ProgramPlayerTestSuite, idle_up_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // up button
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::UP);

  // idle
  driver.setUpValue(common::model::EActionType::NO_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_up_fault_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // up button
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::UP);

  // fault
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_down_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // down button
  driver.setDownValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::DOWN);

  // idle
  driver.setUpValue(common::model::EActionType::NO_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_down_fault_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // down button
  driver.setDownValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::DOWN);

  // fault
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_play_fault_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // play button
  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);

  // fault
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_play_playing_stop_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // stop button
  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);

  // playing
  driver.setCommunicationCode(COMM_SUCCESS);
  driver.setPlayValue(common::model::EActionType::NO_ACTION);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);

  // stop button
  driver.setStopValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::STOP);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_play_playing_fault_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // play button
  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);

  // playing
  driver.setPlayValue(common::model::EActionType::NO_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);

  // fault
  driver.setCommunicationCode(COMM_NOT_AVAILABLE);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_play_playing_done_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // play button
  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);

  // playing
  driver.setPlayValue(common::model::EActionType::NO_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);

  // done/idle
  adapter.setProgramState(ProgramExecutionState::DONE);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_play_playing_error_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // play button
  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);

  // playing
  driver.setPlayValue(common::model::EActionType::NO_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);

  // error/idle
  adapter.setProgramState(ProgramExecutionState::ERROR);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

TEST_F(ProgramPlayerTestSuite, transition_idle_play_playing_playing_done_idle)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // play button
  driver.setPlayValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAY);

  // playing
  driver.setPlayValue(common::model::EActionType::NO_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);

  // playing
  adapter.setProgramState(ProgramExecutionState::PLAYING);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::PLAYING);

  // done/idle
  adapter.setProgramState(ProgramExecutionState::DONE);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
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

  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);

  // up button
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getCurrentProgram(), (++program_list.begin())->first);

  // idle
  program_player->tick();

  // up button twice, overflow list go to first program
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();

  // idle
  program_player->tick();

  // up button twice, overflow list go to first program
  driver.setUpValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);
}

// Down button Program selection
TEST_F(ProgramPlayerTestSuite, down_button_program_selection)
{
  ProgramPlayerMockDriver& driver = program_player->getDriver();

  ASSERT_EQ(program_player->getCurrentProgram(), program_list.begin()->first);

  // down button
  driver.setDownValue(common::model::EActionType::SINGLE_PUSH_ACTION);
  driver.setCommunicationCode(COMM_SUCCESS);

  program_player->tick();
  ASSERT_EQ(program_player->getCurrentProgram(), (--program_list.end())->first);

  // idle
  program_player->tick();
}

// Get robot name
TEST_F(ProgramPlayerTestSuite, get_robot_name)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  ASSERT_EQ(program_player->getRobotName(), adapter.getRobotName());
}

// Niryo studio connected
TEST_F(ProgramPlayerTestSuite, test19)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // fault
  auto timestamp = std::chrono::system_clock::now();
  adapter.setNiryoStudioTimeStamp(timestamp);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  timestamp = std::chrono::system_clock::now() - std::chrono::seconds(5);
  adapter.setNiryoStudioTimeStamp(timestamp);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

// No program available
TEST_F(ProgramPlayerTestSuite, test20)
{
  ProgramPlayerMockAdapter& adapter = program_player->getAdapter();

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);

  // fault
  program_list.clear();
  adapter.setProgramList(program_list);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::FAULT);

  // idle
  program_list = { std::make_pair("unique-name1", "program1") };
  adapter.setProgramList(program_list);

  program_player->tick();
  ASSERT_EQ(program_player->getProgramPlayerState(), ProgramPlayerState::IDLE);
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
