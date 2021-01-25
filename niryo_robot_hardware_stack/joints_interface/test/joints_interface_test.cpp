#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int64MultiArray.h>
// #include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "niryo_robot_msgs/SetBool.h"
#include "niryo_robot_msgs/SetInt.h"

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

class JointsInterfaceTest
{

private:
    ros::NodeHandle nh;
    ros::ServiceClient learnig_mode_client;
    ros::ServiceClient calibrate_motor_client;

    ros::Publisher reset_stepper_publisher;

    TrajClient *traj_client_;

    std::vector<double> pose_start{0.2, 0.0, 0.25, 0.0, 1.57, 0};
    std::vector<double> pick_pose{0.2, 0.1, 0.14, 0.0, 1.57, 0};
    std::vector<double> place_pose{0.2, -0.1, 0.14, 0.0, 1.57, 0};

public:
    JointsInterfaceTest()
    {

        reset_stepper_publisher = nh.advertise<std_msgs::Empty>("/niryo_robot/joints_interface/steppers_reset_controller", 1000);
        learnig_mode_client = nh.serviceClient<niryo_robot_msgs::SetBool>("niryo_robot/learning_mode/activate");
        calibrate_motor_client = nh.serviceClient<niryo_robot_msgs::SetInt>("/niryo_robot/joints_interface/calibrate_motors");

        traj_client_ = new TrajClient("/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory", true);

        ROS_INFO(" Joint interface - Test - Calibrate motors");
        ros::Duration(1).sleep();
        niryo_robot_msgs::SetInt calibrate_srv;
        calibrate_srv.request.value = 0;
        if (calibrate_motor_client.call(calibrate_srv))
        {
            ROS_INFO(" Joint interface - Test - Calibrate motor service return");
        }
        ros::Duration(10).sleep();

        ROS_INFO(" Joint interface - Test - Deactivate Learning Mode");
        niryo_robot_msgs::SetBool learning_mode_srv;
        learning_mode_srv.request.value = false;
        if (learnig_mode_client.call(learning_mode_srv))
        {
            ROS_INFO(" Joint interface - Test - Learning mode Server return");
        }
        ros::Duration(10).sleep();

        // ROS_INFO(" Joint interface - Test - Activate Learning Mode");
        // learning_mode_srv.request.value = true;
        // if(learnig_mode_client.call(learning_mode_srv))
        // {
        //     ROS_INFO(" Joint interface - Test - Learning mode Server return");
        // }
        // ros::Duration(10).sleep();

        // ROS_INFO(" Joint interface - Test - Deactivate Learning Mode");
        // learning_mode_srv.request.value = false;
        // if(learnig_mode_client.call(learning_mode_srv))
        // {
        //     ROS_INFO(" Joint interface - Test - Learning mode Server return");
        // }
        // ros::Duration(10).sleep();

        ROS_INFO(" Joint interface - Test - Send trajectory");
        while (!traj_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO(" Joint interface - Test - Waiting for the joint_trajectory_action server");
        }

        ROS_INFO(" Joint interface - Test - Reset Controller");
        std_msgs::Empty reset_controller_topic;
        reset_stepper_publisher.publish(reset_controller_topic);
        ros::Duration(0.05).sleep();
        startTrajectory(armExtensionTrajectory2());
        // Wait for trajectory completion
        while (!getState().isDone() && ros::ok())
        {
            usleep(50000);
        }
        // ros::Rate(10).sleep();

        // startTrajectory(armExtensionTrajectory2());
        // // Wait for trajectory completion
        // while(!getState().isDone() && ros::ok())
        // {
        //     usleep(50000);
        // }
    }

    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
    {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        traj_client_->sendGoal(goal);
    }

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
    {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("joint_1");
        goal.trajectory.joint_names.push_back("joint_2");
        goal.trajectory.joint_names.push_back("joint_3");
        goal.trajectory.joint_names.push_back("joint_4");
        goal.trajectory.joint_names.push_back("joint_5");
        goal.trajectory.joint_names.push_back("joint_6");
        ;

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(2);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(6);
        goal.trajectory.points[ind].positions[0] = 0.0;
        goal.trajectory.points[ind].positions[1] = 0.0;
        goal.trajectory.points[ind].positions[2] = 0.0;
        goal.trajectory.points[ind].positions[3] = 0.0;
        goal.trajectory.points[ind].positions[4] = 0.0;
        goal.trajectory.points[ind].positions[5] = 0.0;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(6);
        for (size_t j = 0; j < 6; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Second trajectory point
        // Positions
        ind += 1;
        goal.trajectory.points[ind].positions.resize(6);
        goal.trajectory.points[ind].positions[0] = 1.6;
        goal.trajectory.points[ind].positions[1] = 0.0;
        goal.trajectory.points[ind].positions[2] = 0.0;
        goal.trajectory.points[ind].positions[3] = 0.0;
        goal.trajectory.points[ind].positions[4] = 0.0;
        goal.trajectory.points[ind].positions[5] = 0.0;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(6);
        for (size_t j = 0; j < 6; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 2 seconds after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

        //we are done; return the goal
        return goal;
    }

    control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory2()
    {
        //our goal variable
        control_msgs::FollowJointTrajectoryGoal goal;

        // First, the joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("joint_1");
        goal.trajectory.joint_names.push_back("joint_2");
        goal.trajectory.joint_names.push_back("joint_3");
        goal.trajectory.joint_names.push_back("joint_4");
        goal.trajectory.joint_names.push_back("joint_5");
        goal.trajectory.joint_names.push_back("joint_6");

        // We will have two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(6);
        goal.trajectory.points[ind].positions[0] = 1.6;
        goal.trajectory.points[ind].positions[1] = 0.0;
        goal.trajectory.points[ind].positions[2] = 0.0;
        goal.trajectory.points[ind].positions[3] = 0.0;
        goal.trajectory.points[ind].positions[4] = 0.0;
        goal.trajectory.points[ind].positions[5] = 0.0;
        // goal.trajectory.points[ind].positions.resize(6);
        // goal.trajectory.points[ind].positions[0] = 1.6;
        // goal.trajectory.points[ind].positions[1] = 0.0;
        // goal.trajectory.points[ind].positions[2] = 1.6;
        // goal.trajectory.points[ind].positions[3] = -1.5;
        // goal.trajectory.points[ind].positions[4] = -1.3;
        // goal.trajectory.points[ind].positions[5] = -1;
        // Velocities
        goal.trajectory.points[ind].velocities.resize(6);
        for (size_t j = 0; j < 6; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // To be reached 1 second after starting along the trajectory
        goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

        // Second trajectory point
        // Positions
        // ind += 1;
        // goal.trajectory.points[ind].positions.resize(6);
        // goal.trajectory.points[ind].positions[0] = -0.3;
        // goal.trajectory.points[ind].positions[1] = 0.2;
        // goal.trajectory.points[ind].positions[2] = -0.1;
        // goal.trajectory.points[ind].positions[3] = -1.2;
        // goal.trajectory.points[ind].positions[4] = 1.5;
        // goal.trajectory.points[ind].positions[5] = -0.3;
        // // Velocities
        // goal.trajectory.points[ind].velocities.resize(6);
        // for (size_t j = 0; j < 6; ++j)
        // {
        //     goal.trajectory.points[ind].velocities[j] = 0.0;
        // }
        // // To be reached 2 seconds after starting along the trajectory
        // goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

        //we are done; return the goal
        return goal;
    }

    actionlib::SimpleClientGoalState getState()
    {
        return traj_client_->getState();
    }

    void test1()
    {
        ROS_INFO(" Joint interface - Test -  Deactivate Learning Mod ");
        ros::Rate pause_1(1);
        ros::Rate pause_2(2);
        activeFreeDriveMod(false);
        pause_2.sleep();

        ROS_INFO(" Joint interface - Test -  Activate Learning Mod ");
        activeFreeDriveMod(true);
        pause_2.sleep();

        ROS_INFO(" Joint interface - Test -  Activate Learning Mod ");
        activeFreeDriveMod(true);
        pause_2.sleep();

        ROS_INFO(" Joint interface - Test -  Deactivate Learning Mod ");
        activeFreeDriveMod(false);
        pause_2.sleep();

        ROS_INFO(" Joint interface - Test -  Deactivate Learning Mod ");
        activeFreeDriveMod(false);
        pause_2.sleep();

        ROS_INFO(" Joint interface - Test -  Activate Learning Mod ");
        activeFreeDriveMod(true);
        pause_2.sleep();
    }

    void activeFreeDriveMod(bool active)
    {
        niryo_robot_msgs::SetBool msg;
        msg.request.value = active;
        if (learnig_mode_client.call(msg))
        {
            ROS_INFO(" Joint interface - Test - Free Drive Server return ");
        }
        else
        {
            ROS_INFO(" Joint interface - Test - Free Drive error");
        }
    }

    void test2()
    {
        ROS_INFO(" Joint interface - Test -  Calibrate motors ");
        niryo_robot_msgs::SetInt msg;
        msg.request.value = 0;
        if (calibrate_motor_client.call(msg))
        {
            ROS_INFO(" Joint interface - Test - Calibrate motor Server return ");
        }
        else
        {
            ROS_INFO(" Joint interface - Test - Calibrate motor error");
        }
    }

    // void test3()
    // {
    //     activeFreeDriveMod(false);
    //     ROS_INFO(" Joint interface - Test -  Move P ");
    //     moveP(place_pose);

    // }

    // void moveP(const std::vector<double> &pose)
    // {
    //     geometry_msgs::Pose target_pose;
    //     Eigen::Quaterniond quaternion_matrix;

    //     target_pose.position.x = pose[0];
    //     target_pose.position.y = pose[1];
    //     target_pose.position.z = pose[2];
    //     quaternion_matrix =
    //         Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitZ())
    //         * Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY())
    //         * Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitX());
    //     target_pose.orientation.x = quaternion_matrix.x();
    //     target_pose.orientation.y = quaternion_matrix.y();
    //     target_pose.orientation.z = quaternion_matrix.z();
    //     target_pose.orientation.w = quaternion_matrix.w();

    //     move_group->setPoseTarget(target_pose);
    //     move_group->plan(computed_plan);

    //     std_msgs::Empty empty_msg;
    //     reset_stepper_publisher.publish(empty_msg);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(50));

    //     move_group->move();

    // }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stepper_driver_test");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    JointsInterfaceTest test;

    ros::waitForShutdown();

    ROS_INFO(" Joint interface - Test - shutdown node");
}
