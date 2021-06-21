/*
    ttl_driver_integration_test.cpp
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

#include <memory>
#include <ros/ros.h>
#include <string>
#include <queue>
#include <functional>
#include <vector>

#include <ttl_driver/ttl_driver_core.hpp>
#include "common/model/dxl_command_type_enum.hpp"

class TtlDriverTest
{
    private:
        ros::NodeHandle nh;

        std::shared_ptr<ttl_driver::TtlDriverCore> _ttl_driver;

        std::vector<uint32_t> dxl_home_pose{2000, 2047 , 511};
        std::vector<uint32_t> dxl_pose_1{2047, 2047 , 511};
        std::vector<uint32_t> dxl_pose_2{2047, 2047 , 511};

        std::vector<common::model::DxlMotorState> states;

    public:
        TtlDriverTest()
        {
            ros::NodeHandle nodeHandle("~");

            states.emplace_back(common::model::DxlMotorState(common::model::EMotorType::XL430, 2));
            states.emplace_back(common::model::DxlMotorState(common::model::EMotorType::XL430, 3));
            states.emplace_back(common::model::DxlMotorState(common::model::EMotorType::XL320, 6));

            _ttl_driver.reset(new TtlDriver::TtlDriverCore());

            TestServiceActiveTorque();
            TestPublishPoseCmd();

            TestReceiveState();
        }

        void TestServiceActiveTorque()
        {
            ROS_INFO("active all arm motors");
            common::model::SynchronizeMotorCmd cmd_on(common::model::EDxlCommandType::CMD_TYPE_TORQUE);
            for (auto const& dState : states)
                cmd_on.addMotorParam(dState, 1);
            _ttl_driver->setSyncCommand(cmd_on);

            common::model::SynchronizeMotorCmd cmd_off(common::model::EDxlCommandType::CMD_TYPE_TORQUE);
            for (auto const& dState : states)
                cmd_off.addMotorParam(dState, 0);
            _ttl_driver->setSyncCommand(cmd_off);
        }

        void TestPublishPoseCmd()
        {
            ROS_INFO("move all arm motors");
            common::model::SynchronizeMotorCmd cmd(common::model::EDxlCommandType::CMD_TYPE_TORQUE);
            for (size_t i = 0; i < states.size(); ++i)
                cmd.addMotorParam(states.at(i), 1);
            _ttl_driver->setSyncCommand(cmd);
            ROS_INFO("Sending command 1");
            _ttl_driver->setSyncCommand(cmd);
            ros::Duration(3).sleep();

            cmd.reset();
            cmd.setType(common::model::EDxlCommandType::CMD_TYPE_POSITION);
            for (size_t i = 0; i < states.size(); ++i)
                cmd.addMotorParam(states.at(i), dxl_home_pose.at(i));

            ROS_INFO("Sending command 2");
            _ttl_driver->setSyncCommand(cmd);
            ros::Duration(3).sleep();

            cmd.clear();
            for (size_t i = 0; i < states.size(); ++i)
                cmd.addMotorParam(states.at(i), dxl_pose_1.at(i));

            ROS_INFO("Sending command 3");
            _ttl_driver->setSyncCommand(cmd);
            ros::Duration(3).sleep();


            cmd.clear();
            for (size_t i = 0; i < states.size(); ++i)
                cmd.addMotorParam(states.at(i), dxl_pose_2.at(i));

            ROS_INFO("Sending command 4");
            _ttl_driver->setSyncCommand(cmd);
            ros::Duration(3).sleep();

            cmd.reset();
            cmd.setType(common::model::EDxlCommandType::CMD_TYPE_TORQUE);
            for (auto const& dState : states)
                cmd.addMotorParam(dState, 0);
            ROS_INFO("Sending command 5");
            _ttl_driver->setSyncCommand(cmd);
        }

        void TestReceiveState()
        {
            ros::Duration r(0.1);
            std::vector<common::model::DxlMotorState> motor_state;
            for (int i = 0 ; i < 100 ; i++)
            {
                motor_state = _ttl_driver->getDxlStates();
                for (size_t j = 0 ; j < motor_state.size() ; j++)
                {
                    std::cout << " " << motor_state.at(j).getId()
                              << "    " << motor_state.at(j).getPositionState() << std::endl;
                }
                std::cout << " "  << std::endl;
                r.sleep();
            }
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ttl_driver_test");

    ROS_DEBUG("Launching ttl_driver_test");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    TtlDriverTest test;

    ros::waitForShutdown();

    ROS_INFO("shutdown node");
}