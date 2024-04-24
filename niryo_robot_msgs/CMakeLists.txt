cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_msgs)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  BasicObject.msg
  BasicObjectArray.msg
  BusState.msg
  CommandStatus.msg
  HardwareStatus.msg
  MotorHeader.msg
  ObjectPose.msg
  RPY.msg
  RobotState.msg
  SoftwareVersion.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  AdvertiseShutdown.srv
  GetBool.srv
  GetInt.srv
  GetNameDescriptionList.srv
  GetString.srv
  GetStringList.srv
  Ping.srv
  SetBool.srv
  GetFloatList.srv
  SetFloatList.srv
  SetFloat.srv
  SetInt.srv
  SetString.srv
  Trigger.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
##   Doc   ##
#############

#############
## Testing ##
#############

#############
## Install ##
#############

# no scripts to install

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    # no static code analysis on external libs
    # create empty target (needed for catkin build --make-args roslint)
    message(STATUS "no lint analysis for this packet. Create empty target for conveniency")
    roslint_cpp(empty) # hack to create an empty roslint target
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()
