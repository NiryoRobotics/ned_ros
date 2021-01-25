/*
    conveyor_test.cpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "conveyor_interface/conveyor_interface_core.hpp"

#include "stepper_driver/stepper_driver_core.hpp"

boost::shared_ptr<ConveyorInterfaceCore> conveyor_interface_bite;

void TestConveyor(ros::NodeHandle nh)
{
    ros::ServiceClient set_conveyor_client = nh.serviceClient<conveyor_interface::SetConveyor>("niryo_robot/tools/ping_and_set_conveyor");
    ros::ServiceClient control_conveyor_client = nh.serviceClient<conveyor_interface::ControlConveyor>("niryo_robot/tools/control_conveyor");
    ros::ServiceClient change_conveyor_client = nh.serviceClient<conveyor_interface::UpdateConveyorId>("niryo_robot/tools/update_conveyor_id");

    conveyor_interface::SetConveyor set_conveyor_srv;
    conveyor_interface::ControlConveyor control_conveyor_srv;
    conveyor_interface::UpdateConveyorId update_conveyor_srv;

    ROS_INFO("Conveyor Test - Set Conveyor");
    set_conveyor_srv.request.id = 6;
    set_conveyor_srv.request.activate = true;
    if(set_conveyor_client.call(set_conveyor_srv))
    {
        ROS_INFO("Conveyor Test -  Server return");
    }
    ros::Duration(5).sleep();

    ROS_INFO("Conveyor Test - Control Conveyor step 1");
    control_conveyor_srv.request.id = 6;
    control_conveyor_srv.request.control_on = true;
    control_conveyor_srv.request.speed = 75;
    control_conveyor_srv.request.direction = 1;
    if(control_conveyor_client.call(control_conveyor_srv))
    {
        ROS_INFO("Conveyor Test - Server return");
    }
    ros::Duration(5).sleep();

    ROS_INFO("Conveyor Test - Control Conveyor step 2");
    control_conveyor_srv.request.id = 6;
    control_conveyor_srv.request.control_on = true;
    control_conveyor_srv.request.speed = 75;
    control_conveyor_srv.request.direction = -1;
    if(control_conveyor_client.call(control_conveyor_srv))
    {
        ROS_INFO("Conveyor Test - Server return");
    }
    ros::Duration(5).sleep();

    ROS_INFO("Control Conveyor step 3");
    control_conveyor_srv.request.id = 6;
    control_conveyor_srv.request.control_on = false;
    control_conveyor_srv.request.speed = 75;
    control_conveyor_srv.request.direction = -1;
    if(control_conveyor_client.call(control_conveyor_srv))
    {
        ROS_INFO("Conveyor Test - Server return");
    }
    ros::Duration(5).sleep();

    ROS_INFO("Conveyor Test - Change Conveyor ID");
    update_conveyor_srv.request.old_id = 6;
    update_conveyor_srv.request.new_id = 7;
    if(change_conveyor_client.call(update_conveyor_srv))
    {
        ROS_INFO("Conveyor Test - Server return");
    }    
    ros::Duration(5).sleep();

    ROS_INFO("Conveyor Test - Control Conveyor step 2");
    control_conveyor_srv.request.id = 7;
    control_conveyor_srv.request.control_on = true;
    control_conveyor_srv.request.speed = 75;
    control_conveyor_srv.request.direction = -1;
    if(control_conveyor_client.call(control_conveyor_srv))
    {
        ROS_INFO("Conveyor Test - Server return");
    }
    ros::Duration(5).sleep();

    ROS_INFO("Conveyor Test - Control Conveyor step 3");
    control_conveyor_srv.request.id = 7;
    control_conveyor_srv.request.control_on = false;
    control_conveyor_srv.request.speed = 75;
    control_conveyor_srv.request.direction = -1;
    if(control_conveyor_client.call(control_conveyor_srv))
    {
        ROS_INFO("Conveyor Test - Server return");
    }
    ros::Duration(5).sleep();
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "conveyor_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;

    boost::shared_ptr<StepperDriver::StepperDriverCore> stepper(new StepperDriver::StepperDriverCore());
    ros::Duration(1).sleep();
    conveyor_interface_bite.reset(new ConveyorInterfaceCore(stepper));
    ros::Duration(1).sleep();
    TestConveyor(nh);
    ros::waitForShutdown();
    
    ROS_INFO("Conveyor Test - Shutdown node");
}
