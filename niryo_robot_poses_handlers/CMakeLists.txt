cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_poses_handlers)

find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  message_generation
  moveit_msgs
  niryo_robot_msgs
)

add_message_files(
  FILES
  NiryoPose.msg
  Trajectory.msg
  Workspace.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetPose.srv
  GetTargetPose.srv
  GetTrajectory.srv
  GetWorkspaceRatio.srv
  GetWorkspaceRobotPoses.srv
  ManagePose.srv
  ManageTrajectory.srv
  ManageWorkspace.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
  moveit_msgs
  niryo_robot_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  moveit_msgs
  moveit_msgs
  niryo_robot_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
    PROGRAMS scripts/poses_handlers_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(launch/rpi_poses_handlers.test)
  add_rostest(launch/simulation_poses_handlers.test)
endif()
