message("\n## Reading niryo_robot_poses_handlers/CMakeLists.txt...")

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

# enable python module support
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

execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )
    
if (${ARCHITECTURE} MATCHES "^(arm.*|aarch64.*|arm64.*)$")
  add_rostest(test/rpi_poses_handlers.test)
else()
  add_rostest(test/simulation_poses_handlers.test)
endif()

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/transform_handler.py
    scripts/poses_handlers_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation
install(
  DIRECTORY grips/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/grips
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_python()
endif()
