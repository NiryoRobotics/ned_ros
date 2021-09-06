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

#include <string>

// Declare a test

/******************************************************/
/************ Tests of ttl interface ******************/
/******************************************************/

class TtlInterfaceTestSuite : public ::testing::Test {
  protected:
    static void SetUpTestCase()
    {
      ros::NodeHandle nh("ttl_driver");
      ros::Duration(5.0).sleep();

      ttl_interface = std::make_shared<ttl_driver::TtlInterfaceCore>(nh);
      // check connections
      EXPECT_TRUE(ttl_interface->isConnectionOk());
      EXPECT_TRUE(ttl_interface->scanMotorId(2));
      EXPECT_TRUE(ttl_interface->scanMotorId(3));
      EXPECT_TRUE(ttl_interface->scanMotorId(6));
    }

    static void TearDownTestCase()
    {
      ros::shutdown();
    }

    static std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface;
};

std::shared_ptr<ttl_driver::TtlInterfaceCore> TtlInterfaceTestSuite::ttl_interface;

// Test reboot motors
TEST_F(TtlInterfaceTestSuite, testRebootMotors)
{
  int resutl = ttl_interface->rebootMotors();
  EXPECT_EQ(resutl, static_cast<int>(niryo_robot_msgs::CommandStatus::SUCCESS));
}

// Test reboot motor with wrong id
TEST_F(TtlInterfaceTestSuite, testRebootMotorsWrongID)
{
  bool result;
  result = ttl_interface->rebootMotor(20);
  EXPECT_FALSE(result);
}

class TtlManagerTestSuite : public ::testing::Test {
  protected:
    static void SetUpTestCase()
    {
      ros::NodeHandle nh("ttl_driver");
      ros::NodeHandle nh_private("~");
      nh_private.getParam("hardware_version", hw_version);

      ttl_drv = std::make_shared<ttl_driver::TtlManager>(nh);

      // check connections
      EXPECT_TRUE(ttl_drv->ping(2));
      EXPECT_TRUE(ttl_drv->ping(3));
      EXPECT_TRUE(ttl_drv->ping(6));
    }

    static std::string hw_version;
    static std::shared_ptr<ttl_driver::TtlManager> ttl_drv;
};

std::shared_ptr<ttl_driver::TtlManager> TtlManagerTestSuite::ttl_drv;
std::string TtlManagerTestSuite::hw_version;

/******************************************************/
/************** Tests of ttl manager ******************/
/******************************************************/

/*
* Theses tests is used to test NED v1
*/
// Test driver received cmd
TEST_F(TtlManagerTestSuite, testSingleCmds)
{
  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_1 = std::make_shared<common::model::DxlSingleCmd>(
                                                                          common::model::EDxlCommandType::CMD_TYPE_TORQUE,
                                                                          5,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_EQ(ttl_drv->writeSingleCommand(cmd_1), COMM_SUCCESS);
  ros::Duration(0.01).sleep();

  // wrong id
  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_2 = std::make_shared<common::model::DxlSingleCmd>(
                                                                          common::model::EDxlCommandType::CMD_TYPE_TORQUE,
                                                                          20,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_NE(ttl_drv->writeSingleCommand(cmd_2), COMM_SUCCESS);
  ros::Duration(0.01).sleep();

  // wrong type cmd
  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_3 = std::make_shared<common::model::DxlSingleCmd>(
                                                                          common::model::EDxlCommandType::CMD_TYPE_UNKNOWN,
                                                                          2,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_NE(ttl_drv->writeSingleCommand(cmd_3), COMM_SUCCESS);
  ros::Duration(0.01).sleep();

  // wrong type of cmd object
  std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> cmd_4 = std::make_shared<common::model::StepperTtlSingleCmd>(
                                                                          common::model::EStepperCommandType::CMD_TYPE_TORQUE,
                                                                          5,
                                                                          std::initializer_list<uint32_t>{1});
  EXPECT_NE(ttl_drv->writeSingleCommand(cmd_4), COMM_SUCCESS);
}

TEST_F(TtlManagerTestSuite, testSyncCmdsOnHW)
{
  // sync cmd
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_1 = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd_1->addMotorParam(common::model::EHardwareType::XL430, 2, 1);
  dynamixel_cmd_1->addMotorParam(common::model::EHardwareType::XL430, 3, 1);

  EXPECT_EQ(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_1), COMM_SUCCESS);
  ros::Duration(0.5).sleep();

  // sync cmd with different motor types
  if (hw_version == "NED")
  {
    // sync cmd with different motor types
    std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_2 = std::make_shared<common::model::DxlSyncCmd>(
                                                              common::model::EDxlCommandType::CMD_TYPE_TORQUE);
    dynamixel_cmd_2->addMotorParam(common::model::EHardwareType::XL430, 2, 1);
    dynamixel_cmd_2->addMotorParam(common::model::EHardwareType::XL320, 6, 1);

    EXPECT_EQ(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_2), COMM_SUCCESS);
    ros::Duration(0.5).sleep();
  }

  // redondant id
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_3 = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd_3->addMotorParam(common::model::EHardwareType::XL430, 3, 1);
  dynamixel_cmd_3->addMotorParam(common::model::EHardwareType::XL430, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_3), COMM_SUCCESS);

  // wrong cmd type
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_4 = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_UNKNOWN);
  dynamixel_cmd_4->addMotorParam(common::model::EHardwareType::XL320, 5, 1);
  dynamixel_cmd_4->addMotorParam(common::model::EHardwareType::XL320, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_4), COMM_SUCCESS);
}

TEST_F(TtlManagerTestSuite, testSyncCmdsOnFakeHW)
{
  // sync cmd
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_1 = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd_1->addMotorParam(common::model::EHardwareType::FAKE_DXL_MOTOR, 2, 1);
  dynamixel_cmd_1->addMotorParam(common::model::EHardwareType::FAKE_DXL_MOTOR, 3, 1);

  EXPECT_EQ(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_1), COMM_SUCCESS);
  ros::Duration(0.5).sleep();

  // redondant id
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_3 = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_TORQUE);
  dynamixel_cmd_3->addMotorParam(common::model::EHardwareType::FAKE_DXL_MOTOR, 3, 1);
  dynamixel_cmd_3->addMotorParam(common::model::EHardwareType::FAKE_DXL_MOTOR, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_3), COMM_SUCCESS);

  // wrong cmd type
  std::shared_ptr<common::model::DxlSyncCmd> dynamixel_cmd_4 = std::make_shared<common::model::DxlSyncCmd>(
                                                            common::model::EDxlCommandType::CMD_TYPE_UNKNOWN);
  dynamixel_cmd_4->addMotorParam(common::model::EHardwareType::FAKE_DXL_MOTOR, 5, 1);
  dynamixel_cmd_4->addMotorParam(common::model::EHardwareType::FAKE_DXL_MOTOR, 3, 1);

  EXPECT_NE(ttl_drv->writeSynchronizeCommand(dynamixel_cmd_4), COMM_SUCCESS);
}

// Test driver scan motors
TEST_F(TtlManagerTestSuite, scanTest)
{
  EXPECT_EQ(ttl_drv->scanAndCheck(), COMM_SUCCESS);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ttl_driver_unit_tests");

  std::string hardware_version;
  ros::NodeHandle nh_private("~");
  nh_private.getParam("hardware_version", hardware_version);
  if (hardware_version == "fake")
    testing::GTEST_FLAG(filter) = "-TtlManagerTestSuite.testSyncCmdsOnHW";
  else
    testing::GTEST_FLAG(filter) = "-TtlManagerTestSuite.testSyncCmdsOnFakeHW";
  return RUN_ALL_TESTS();
}
