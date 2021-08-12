/*
    ttl_driver_unit_tests.cpp
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
#include "ttl_driver/ttl_interface_core.hpp"
#include "ttl_driver/ttl_manager.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Bring in gtest
#include <gtest/gtest.h>
#include <ros/console.h>

// Declare a test
class TtlInterfaceTestSuite : public ::testing::Test {
  protected:
    void SetUp() override
    {
      ros::NodeHandle nh;
      ttl_interface = std::make_unique<ttl_driver::TtlInterfaceCore>(nh);

      // check connections
      EXPECT_TRUE(ttl_interface->isConnectionOk());
      EXPECT_TRUE(ttl_interface->scanMotorId(2));
      EXPECT_TRUE(ttl_interface->scanMotorId(3));
      EXPECT_TRUE(ttl_interface->scanMotorId(6));
    }

    std::unique_ptr<ttl_driver::TtlInterfaceCore> ttl_interface;
};

TEST_F(TtlInterfaceTestSuite, testRebootMotors)
{
  EXPECT_EQ(ttl_interface->rebootMotors(), niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST_F(TtlInterfaceTestSuite, testRebootMotorsWrongID)
{
  EXPECT_NE(ttl_interface->rebootMotor(7), niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST_F(TtlInterfaceTestSuite, testCalibration)
{
  EXPECT_EQ(ttl_interface->getCalibrationStatus(), common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED);

  ttl_interface->startCalibration();
  
  EXPECT_EQ(ttl_interface->getCalibrationStatus(), common::model::EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS);

  ros::Duration(10.0).sleep();
  EXPECT_EQ(ttl_interface->getCalibrationStatus(), common::model::EStepperCalibrationStatus::CALIBRATION_OK);
}

class TtlManagerTestSuite : public ::testing::Test {
  protected:
    void SetUp() override
    {
      ros::NodeHandle nh;
      ttl_drv = std::make_shared<ttl_driver::TtlManager>(nh);

      // check connections
      EXPECT_TRUE(ttl_drv->ping(2));
      EXPECT_TRUE(ttl_drv->ping(3));
      EXPECT_TRUE(ttl_drv->ping(6));
    }

    std::shared_ptr<ttl_driver::TtlManager> ttl_drv;
};


// Test driver received cmd
TEST_F(TtlManagerTestSuite, testCmds)
{
  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_1 = std::make_shared<common::model::DxlSingleCmd>(
                                                                          common::model::EDxlCommandType::CMD_TYPE_TORQUE,
                                                                          2,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_EQ(ttl_drv->writeSingleCommand(cmd_1), COMM_SUCCESS);
  ros::Duration(0.01).sleep();

  // wrong id
  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_2 = std::make_shared<common::model::DxlSingleCmd>(
                                                                          common::model::EDxlCommandType::CMD_TYPE_TORQUE,
                                                                          7,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_EQ(ttl_drv->writeSingleCommand(cmd_2), COMM_TX_ERROR);
  ros::Duration(0.01).sleep();

  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_3 = std::make_shared<common::model::DxlSingleCmd>(
                                                                          common::model::EDxlCommandType::CMD_TYPE_UNKNOWN,
                                                                          2,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_NE(ttl_drv->writeSingleCommand(cmd_3), COMM_SUCCESS);
  ros::Duration(0.01).sleep();

  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_4 = std::make_shared<common::model::StepperTtlSingleCmd>(
                                                                          common::model::EStepperCommandType::CMD_TYPE_TORQUE,
                                                                          2,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_NE(ttl_drv->writeSingleCommand(cmd_4), COMM_SUCCESS);

  // sync cmd
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL430, 2, 1);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL430, 3, 1);

  EXPECT_EQ(ttl_drv->writeSynchronizeCommand(dynamixel_cmd), COMM_SUCCESS);
  ros::Duration(0.5).sleep();

  // wrong id
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL430, 5, 1);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL430, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd), COMM_SUCCESS);
  ros::Duration(0.5).sleep();

  // wrong motor type
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL320, 5, 1);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL320, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd), COMM_SUCCESS);

  // wrong cmd type
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_UNKNOWN);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL320, 5, 1);
  dynamixel_cmd->addMotorParam(common::model::EMotorType::XL320, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd), COMM_SUCCESS);
}

TEST_F(TtlManagerTestSuite, ScanTest)
{
  EXPECT_EQ(ttl_drv->scanAndCheck(), COMM_SUCCESS);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ttl_driver_unit_tests");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
