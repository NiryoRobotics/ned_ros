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
#include "can_driver/can_interface_core.hpp"
#include "can_driver/can_manager.hpp"
#include "common/model/bus_protocol_enum.hpp"
#include "common/model/component_type_enum.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/stepper_motor_state.hpp"
// Bring in gtest
#include <gtest/gtest.h>
#include <memory>
#include <ros/console.h>
#include <string>
#include <utility>

using ::common::model::BusProtocolEnum;
using ::common::model::EBusProtocol;
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;
using ::common::model::StepperMotorState;
using ::std::string;
using ::std::to_string;

// Method add joints
void addJointToCanInterface(std::shared_ptr<can_driver::CanInterfaceCore> can_interface)
{
    size_t nb_joints = 0;

    ros::NodeHandle robot_hwnh("joints_interface");
    // retrieve nb joints with checking that the config param exists for both name and id
    while (robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/id") && robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/name") &&
           robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/type") && robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/bus"))
        nb_joints++;

    // connect and register joint state interface

    int currentIdStepper = 1;

    for (size_t j = 0; j < nb_joints; j++)
    {
        int joint_id_config = 0;
        string joint_name;
        string joint_type;
        string joint_bus;

        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/id", joint_id_config);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/name", joint_name);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/type", joint_type);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/bus", joint_bus);
        HardwareTypeEnum eType = HardwareTypeEnum(joint_type.c_str());
        BusProtocolEnum eBusProto = BusProtocolEnum(joint_bus.c_str());

        if (eType == EHardwareType::STEPPER || eType == EHardwareType::FAKE_STEPPER_MOTOR)
        {  // stepper
            std::string currentStepperNamespace = "steppers/stepper_" + to_string(currentIdStepper);

            auto stepperState = std::make_shared<StepperMotorState>(joint_name, eType, common::model::EComponentType::JOINT, eBusProto, static_cast<uint8_t>(joint_id_config));
            if (stepperState)
            {
                double offsetPos = 0.0;
                double gear_ratio = 1.0;
                int direction = 1;
                double max_effort = 0.0;

                robot_hwnh.getParam(currentStepperNamespace + "/offset_position", offsetPos);
                robot_hwnh.getParam(currentStepperNamespace + "/gear_ratio", gear_ratio);
                robot_hwnh.getParam(currentStepperNamespace + "/direction", direction);
                robot_hwnh.getParam(currentStepperNamespace + "/max_effort", max_effort);

                // add parameters
                stepperState->setOffsetPosition(offsetPos);
                stepperState->setGearRatio(gear_ratio);
                stepperState->setDirection(direction);
                stepperState->setMaxEffort(max_effort);

                if (eBusProto == EBusProtocol::CAN)
                    can_interface->addJoint(stepperState);

                currentIdStepper++;
            }
        }
    }  // end for (size_t j = 0; j < nb_joints; j++)
}

void addJointToCanManager(std::shared_ptr<can_driver::CanManager> can_drv)
{
    size_t nb_joints = 0;

    ros::NodeHandle robot_hwnh("joints_interface");
    // retrieve nb joints with checking that the config param exists for both name and id
    while (robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/id") && robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/name") &&
           robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/type") && robot_hwnh.hasParam("joint_" + to_string(nb_joints + 1) + "/bus"))
        nb_joints++;

    // connect and register joint state interface

    int currentIdStepper = 1;

    for (size_t j = 0; j < nb_joints; j++)
    {
        int joint_id_config = 0;
        string joint_name = "";
        string joint_type = "";
        string joint_bus = "";

        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/id", joint_id_config);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/name", joint_name);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/type", joint_type);
        robot_hwnh.getParam("joint_" + to_string(j + 1) + "/bus", joint_bus);
        HardwareTypeEnum eType = HardwareTypeEnum(joint_type.c_str());
        BusProtocolEnum eBusProto = BusProtocolEnum(joint_bus.c_str());

        if (eType == EHardwareType::STEPPER || eType == EHardwareType::FAKE_STEPPER_MOTOR)
        {  // stepper
            std::string currentStepperNamespace = "steppers/stepper_" + to_string(currentIdStepper);

            auto stepperState = std::make_shared<StepperMotorState>(joint_name, eType, common::model::EComponentType::JOINT, eBusProto, static_cast<uint8_t>(joint_id_config));
            if (stepperState)
            {
                double offsetPos = 0.0;
                double gear_ratio = 1.0;
                int direction = 1;
                double max_effort = 0.0;

                robot_hwnh.getParam(currentStepperNamespace + "/offset_position", offsetPos);
                robot_hwnh.getParam(currentStepperNamespace + "/gear_ratio", gear_ratio);
                robot_hwnh.getParam(currentStepperNamespace + "/direction", direction);
                robot_hwnh.getParam(currentStepperNamespace + "/max_effort", max_effort);

                // add parameters
                stepperState->setOffsetPosition(offsetPos);
                stepperState->setGearRatio(gear_ratio);
                stepperState->setDirection(direction);
                stepperState->setMaxEffort(max_effort);

                if (eBusProto == EBusProtocol::CAN)
                    can_drv->addHardwareComponent(stepperState);

                currentIdStepper++;
            }
        }
    }  // end for (size_t j = 0; j < nb_joints; j++)
}

class CanInterfaceTestSuite : public ::testing::Test
{
  protected:
    static void SetUpTestCase()
    {
        ros::NodeHandle nh("can_driver");
        can_interface = std::make_shared<can_driver::CanInterfaceCore>(nh);

        addJointToCanInterface(can_interface);

        ros::Duration(1.0).sleep();
    }

    static void TearDownTestCase() { ros::shutdown(); }

    static std::shared_ptr<can_driver::CanInterfaceCore> can_interface;
};

std::shared_ptr<can_driver::CanInterfaceCore> CanInterfaceTestSuite::can_interface;
// Declare a test
TEST_F(CanInterfaceTestSuite, testConnection) { EXPECT_TRUE(can_interface->isConnectionOk()); }

// Apply for NED1
class CanManagerTestSuite : public ::testing::Test
{
  protected:
    static void SetUpTestCase()
    {
        ros::NodeHandle nh("can_driver");
        can_manager = std::make_shared<can_driver::CanManager>(nh);

        addJointToCanManager(can_manager);
        // check connections
        EXPECT_TRUE(can_manager->isConnectionOk());
        EXPECT_EQ(static_cast<int>(can_manager->getNbMotors()), 3);
        EXPECT_TRUE(can_manager->ping(1));
        EXPECT_TRUE(can_manager->ping(2));
        EXPECT_TRUE(can_manager->ping(3));
    }

    static void TearDownTestCase() { ros::shutdown(); }

    static std::shared_ptr<can_driver::CanManager> can_manager;
};

std::shared_ptr<can_driver::CanManager> CanManagerTestSuite::can_manager;

TEST_F(CanManagerTestSuite, addAndRemoveMotor)
{
    common::model::StepperMotorState state;
    EXPECT_THROW(can_manager->getPosition(state), std::out_of_range);

    common::model::StepperMotorState new_state(common::model::EHardwareType::STEPPER, common::model::EComponentType::JOINT, common::model::EBusProtocol::CAN, 10);
    can_manager->addHardwareComponent(std::make_shared<common::model::StepperMotorState>(new_state));
    EXPECT_EQ(can_manager->getPosition(new_state), 0);

    can_manager->removeHardwareComponent(10);
    EXPECT_THROW(can_manager->getPosition(new_state), std::out_of_range);
}

TEST_F(CanManagerTestSuite, testSingleCmds)
{
    auto cmd_1 = std::make_unique<common::model::StepperSingleCmd>(common::model::EStepperCommandType::CMD_TYPE_TORQUE, 2, std::initializer_list<int32_t>{1});
    EXPECT_EQ(can_manager->writeSingleCommand(std::move(cmd_1)), CAN_OK);
    ros::Duration(0.01).sleep();

    auto cmd_2 = std::make_unique<common::model::StepperSingleCmd>(common::model::EStepperCommandType::CMD_TYPE_TORQUE, 2, std::initializer_list<int32_t>{0});
    EXPECT_EQ(can_manager->writeSingleCommand(std::move(cmd_2)), CAN_OK);

    ros::Duration(0.01).sleep();

    // wrong id
    auto cmd_3 = std::make_unique<common::model::StepperSingleCmd>(common::model::EStepperCommandType::CMD_TYPE_TORQUE, 7, std::initializer_list<int32_t>{1});
    EXPECT_NE(can_manager->writeSingleCommand(std::move(cmd_3)), CAN_OK);
    ros::Duration(0.01).sleep();

    // wrong type cmd
    auto cmd_4 = std::make_unique<common::model::StepperSingleCmd>(common::model::EStepperCommandType::CMD_TYPE_UNKNOWN, 2, std::initializer_list<int32_t>{1});
    EXPECT_NE(can_manager->writeSingleCommand(std::move(cmd_4)), CAN_OK);
    ros::Duration(0.01).sleep();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "can_driver_unit_tests");

    return RUN_ALL_TESTS();
}
