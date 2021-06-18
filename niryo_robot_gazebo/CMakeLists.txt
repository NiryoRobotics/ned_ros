message("\n## Reading niryo_robot_gazebo/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_gazebo)

find_package(catkin COMPONENTS
  gazebo_ros
)

find_package(catkin REQUIRED)

catkin_package()
