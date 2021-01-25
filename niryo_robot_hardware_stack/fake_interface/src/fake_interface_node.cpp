#include <ros/ros.h>
#include "fake_interface/fake_interface_core.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "fake_interface_node");
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    FakeInterfaceCore nd();
    ros::waitForShutdown();
    
    ROS_INFO("Fake Joint Interface - Shutdown node");
}