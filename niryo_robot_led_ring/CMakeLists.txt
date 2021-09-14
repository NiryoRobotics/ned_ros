cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_led_ring)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  niryo_robot_msgs
  visualization_msgs
)

add_message_files(
  FILES
  LedRingStatus.msg
  LedRingAnimation.msg
  LedRingCurrentState.msg
)

add_service_files(
  DIRECTORY 
    srv
  FILES
    LedUser.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  niryo_robot_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  niryo_robot_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES niryo_robot_led_ring
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

