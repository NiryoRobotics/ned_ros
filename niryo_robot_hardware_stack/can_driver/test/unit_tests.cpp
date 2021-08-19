/*
    can_driver_unit_tests.cpp
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

// Bring in my package's API, which is what I'm testing
#include "can_driver/can_manager.hpp"
#include "can_driver/can_interface_core.hpp"
#include "common/model/single_motor_cmd.hpp"
// Bring in gtest
#include <gtest/gtest.h>
#include <ros/console.h>

class CanInterfaceTestSuite : public ::testing::Test {
  protected:
    static void SetUpTestCase()
    {
      ros::NodeHandle nh("can_driver");
      can_interface = std::make_shared<can_driver::CanInterfaceCore>(nh);
    }

    static void TearDownTestCase()
    {
      ros::shutdown();
    }

    static std::shared_ptr<can_driver::CanInterfaceCore> can_interface;
};

std::shared_ptr<can_driver::CanInterfaceCore> CanInterfaceTestSuite::can_interface;
// Declare a test
TEST_F(CanInterfaceTestSuite, testConnection)
{
    EXPECT_TRUE(can_interface->isConnectionOk());
}

// Apply for NED1
class CanManagerTestSuite : public ::testing::Test {
  protected:
    static void SetUpTestCase()
    {
      ros::NodeHandle nh("can_driver");
      can_manager = std::make_shared<can_driver::CanManager>(nh);
      // check connections
      EXPECT_TRUE(can_manager->isConnectionOk());
      EXPECT_EQ(static_cast<int>(can_manager->getNbMotors()), 3);
      EXPECT_TRUE(can_manager->ping(1));
      EXPECT_TRUE(can_manager->ping(2));
      EXPECT_TRUE(can_manager->ping(3));
    }

    static void TearDownTestCase()
    {
      ros::shutdown();
    }

    static std::shared_ptr<can_driver::CanManager> can_manager;
};

std::shared_ptr<can_driver::CanManager> CanManagerTestSuite::can_manager;

TEST_F(CanManagerTestSuite, addAndRemoveMotor)
{
    EXPECT_THROW(can_manager->getPosition(10), std::out_of_range);
    can_manager->addMotor(10);
    EXPECT_EQ(can_manager->getPosition(10), 0);

    can_manager->removeMotor(10);
    EXPECT_THROW(can_manager->getPosition(10), std::out_of_range);
}

TEST_F(CanManagerTestSuite, testSingleCmds)
{
    std::shared_ptr<common::model::StepperSingleCmd> cmd_1 = std::make_shared<common::model::StepperSingleCmd>(
                                                                          common::model::EStepperCommandType::CMD_TYPE_TORQUE,
                                                                          2,
                                                                          std::initializer_list<int32_t>{1});
    EXPECT_EQ(can_manager->readSingleCommand(cmd_1), CAN_OK);
    ros::Duration(0.01).sleep();

    std::shared_ptr<common::model::StepperSingleCmd> cmd_2 = std::make_shared<common::model::StepperSingleCmd>(
                                                                          common::model::EStepperCommandType::CMD_TYPE_TORQUE,
                                                                          2,
                                                                          std::initializer_list<int32_t>{0});
    EXPECT_EQ(can_manager->readSingleCommand(cmd_2), CAN_OK);

    ros::Duration(0.01).sleep();

    // wrong id
    std::shared_ptr<common::model::StepperSingleCmd> cmd_3 = std::make_shared<common::model::StepperSingleCmd>(
                                                                            common::model::EStepperCommandType::CMD_TYPE_TORQUE,
                                                                            7,
                                                                            std::initializer_list<int32_t>{1});
    EXPECT_NE(can_manager->readSingleCommand(cmd_3), CAN_OK);
    ros::Duration(0.01).sleep();

    // wrong type cmd
    std::shared_ptr<common::model::StepperSingleCmd> cmd_4 = std::make_shared<common::model::StepperSingleCmd>(
                                                                            common::model::EStepperCommandType::CMD_TYPE_UNKNOWN,
                                                                            2,
                                                                            std::initializer_list<int32_t>{1});
    EXPECT_NE(can_manager->readSingleCommand(cmd_4), CAN_OK);
    ros::Duration(0.01).sleep();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "can_driver_unit_tests");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
