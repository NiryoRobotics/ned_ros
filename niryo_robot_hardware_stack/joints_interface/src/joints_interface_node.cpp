#include <ros/ros.h>
#include <ros/console.h>
#include "joints_interface/joints_interface_core.hpp"
#include "dynamixel_driver/dxl_driver_core.hpp"
#include "stepper_driver/stepper_driver_core.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "joints_interface_node");
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel(new DynamixelDriver::DynamixelDriverCore());
    ros::Duration(1).sleep();

    boost::shared_ptr<StepperDriver::StepperDriverCore> stepper(new StepperDriver::StepperDriverCore());
    ros::Duration(1).sleep();

    boost::shared_ptr<JointsInterfaceCore> joints(new JointsInterfaceCore(dynamixel, stepper));
    ros::waitForShutdown();
    
    ROS_INFO("Joints Interface - Shutdown node");
}