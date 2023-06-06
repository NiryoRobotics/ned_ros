cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_poses_handlers)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

#retrieve architecture
execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )

find_package(catkin REQUIRED
  COMPONENTS
  geometry_msgs
  message_generation
  moveit_msgs
  niryo_robot_msgs
)

add_message_files(
  FILES
  DynamicFrame.msg
  NiryoPose.msg
  Workspace.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetDynamicFrame.srv
  GetPose.srv
  GetTargetPose.srv
  GetTransformPose.srv
  GetWorkspaceMatrixPoses.srv
  GetWorkspacePoints.srv
  GetWorkspaceRatio.srv
  GetWorkspaceRobotPoses.srv
  ManageDynamicFrame.srv
  ManagePose.srv
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

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
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
##   Doc   ##
#############

find_program(rosdoc_lite_FOUND rosdoc_lite)
if(rosdoc_lite_FOUND)
    add_custom_target (${PROJECT_NAME}_doc ALL
        rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
    )
else()
  message(WARNING "rosdoc_lite not found. Skipping documentation building")
endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  if ("${ARCHITECTURE}" MATCHES "^(arm.*|aarch64.*|arm64.*)$")
    add_rostest(test/rpi_poses_handlers.test)
  else()
    add_rostest(test/simulation_poses_handlers.test)
  endif()

  ##########################
  ## Static code analysis ##
  ##########################
  find_package(roslint QUIET)

  if(roslint_FOUND)
      set(ROSLINT_PYTHON_OPTS "1>&2")
      roslint_python()
  else()
      message(WARNING "roslint not found. Skipping roslint target building")
  endif()
endif()

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
    scripts/poses_handlers_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation
install(
  DIRECTORY grips/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/grips
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

if (rosdoc_lite_FOUND)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()
