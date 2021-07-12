#ifndef _JOINTS_DRIVER_HPP
#define _JOINTS_DRIVER_HPP

#include <ros/ros.h>

namespace joint_driver
{
    class JointDriver {
    public:
        JointDriver(ros::NodeHandle nh);
        ~JointDriver();

    private:
        ros::NodeHandle _nh;

        void init();
    };
}   // joint_driver

#endif // _JOINTS_DRIVER_HPP