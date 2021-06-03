cmake_minimum_required(VERSION 3.0.2)
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

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############
find_package(rostest REQUIRED)

if (${ARCHITECTURE} MATCHES "^(arm.*|aarch64.*|arm64.*)$")
  add_rostest(test/rpi_poses_handlers.test)
else()
  add_rostest(test/simulation_poses_handlers.test)
endif()