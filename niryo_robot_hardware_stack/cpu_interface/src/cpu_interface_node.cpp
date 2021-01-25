#include <ros/ros.h>
#include <thread> 
#include "cpu_interface/cpu_interface_core.hpp"

boost::shared_ptr<CpuInterfaceCore> cpu;

void readTemperature()
{
    ros::Rate read_rpi_diagnostics_rate = ros::Rate(1);
    int temperature = 0;
    while (ros::ok())
    {
        temperature = cpu->getCpuTemperature();
        ROS_INFO("cpu temperature = %d", temperature);
        read_rpi_diagnostics_rate.sleep();
    }
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "cpu_interface_node");
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    cpu.reset(new CpuInterfaceCore());

    std::thread readTempThread(readTemperature);
    readTempThread.join();
    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}