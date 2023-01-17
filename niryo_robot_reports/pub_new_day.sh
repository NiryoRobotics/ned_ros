#!/usr/bin/env bash

source /opt/ros/noetic/setup.bash
source /home/niryo/catkin_ws/install/release/setup.bash

# wait for node initialization
get_param="rosparam get /niryo_robot_reports/initialized"
$get_param > /dev/null 2>&1
until [[ $? -eq 0 ]] ; do
  sleep 2s
  $get_param > /dev/null 2>&1
done;

rostopic pub --once /niryo_robot_reports/new_day std_msgs/Empty "{}"
