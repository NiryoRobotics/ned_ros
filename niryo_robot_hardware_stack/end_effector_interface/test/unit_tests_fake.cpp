/*
    end_effector_interface_unit_tests.cpp
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

#include <gtest/gtest.h>
#include <memory>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <string>

#include "common/model/tool_state.hpp"

#include "end_effector_interface/end_effector_interface_core.hpp"

#include "ttl_driver/ttl_interface_core.hpp"

class EndEffectorTestSuite : public ::testing::Test
{
  protected:
    static void SetUpTestCase()
    {
        ros::NodeHandle nh_ttl("ttl_driver");
        ros::NodeHandle nh("end_effector_interface");
        ros::Duration(10.0).sleep();

        ttl_interface = std::make_shared<ttl_driver::TtlInterfaceCore>(nh_ttl);

        ee_interface = std::make_shared<end_effector_interface::EndEffectorInterfaceCore>(nh, ttl_interface);
    }

    static void TearDownTestCase() { ros::shutdown(); }

    static std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface;
    static std::shared_ptr<end_effector_interface::EndEffectorInterfaceCore> ee_interface;
};

std::shared_ptr<ttl_driver::TtlInterfaceCore> EndEffectorTestSuite::ttl_interface;
std::shared_ptr<end_effector_interface::EndEffectorInterfaceCore> EndEffectorTestSuite::ee_interface;

TEST_F(EndEffectorTestSuite, config)
{
    ros::NodeHandle nh("end_effector_interface");
    int id{-1};
    std::string hw_type;

    nh.getParam("end_effector_id", id);
    nh.getParam("hardware_type", hw_type);

    ASSERT_EQ(id, 0);
    EXPECT_EQ(hw_type, "fake_end_effector");

    ASSERT_TRUE(nh.hasParam("button_2/type"));
    ASSERT_TRUE(nh.hasParam("button_1/type"));
    ASSERT_TRUE(nh.hasParam("button_0/type"));
}

TEST_F(EndEffectorTestSuite, getEndEffectorState)
{
    auto ee_state = ee_interface->getEndEffectorState();
    ASSERT_NE(ee_state, nullptr);
    ASSERT_EQ(ee_state->getId(), 0);
    ASSERT_EQ(ee_state->getHardwareType(), common::model::EHardwareType::FAKE_END_EFFECTOR);
}

TEST_F(EndEffectorTestSuite, getButtonState)
{
    auto ee_state = ee_interface->getEndEffectorState();
    ASSERT_NE(ee_state, nullptr);

    auto buttons = ee_state->getButtonsStatus();
    ASSERT_TRUE(buttons.at(0)->type != common::model::EButtonType::UNKNOWN);
    ASSERT_TRUE(buttons.at(1)->type != common::model::EButtonType::UNKNOWN);
    ASSERT_TRUE(buttons.at(2)->type != common::model::EButtonType::UNKNOWN);

    bool updated_status;
    for (int i = 0; i < 3; i++)
    {
        updated_status = !buttons.at(0)->actions.empty();
        if (updated_status)
            break;
    }
    ASSERT_TRUE(updated_status);
}

// Test service set Digital IO
TEST_F(EndEffectorTestSuite, rebootHW) { ASSERT_EQ(ee_interface->rebootHardware(), true); }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_interface_unit_tests");

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
