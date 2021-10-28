cmake_minimum_required(VERSION 3.0.2)
project(joints_interface)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")


#retrieve architecture
execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )

## c++ options
## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall  -Werror -Os)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
      actionlib
      can_driver
      common
      controller_manager
      hardware_interface
      niryo_robot_msgs
      roscpp
      sensor_msgs
      std_msgs
      ttl_driver
      dynamic_reconfigure
)

find_package(Boost REQUIRED
    COMPONENTS
        system
)

generate_dynamic_reconfigure_options(
  cfg/steppers_config.cfg
)

###############################################
# Declare ROS messages, services and actions ##
###############################################

###################################
## catkin specific configuration ##
###################################

## INCLUDE_DIRS: if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        ${PROJECT_NAME}_core
    CATKIN_DEPENDS
        actionlib
        can_driver
        common
        controller_manager
        hardware_interface
        niryo_robot_msgs
        roscpp
        ttl_driver
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Declare libs and execs
add_library(${PROJECT_NAME}
    src/calibration_manager.cpp
)

add_library(${PROJECT_NAME}_core
    src/joint_hardware_interface.cpp
    src/joints_interface_core.cpp
)

add_executable(${PROJECT_NAME}_node
    src/joints_interface_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
    niryo_robot_msgs_generate_messages_cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_core
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
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
    add_rostest_gtest(${PROJECT_NAME}_integration_tests
      test/service_client.test
      test/service_client.cpp
    )
  else()
    add_rostest_gtest(${PROJECT_NAME}_integration_tests
    test/service_client_fake.test
    test/service_client.cpp
    )
  endif()
  
  if(TARGET ${PROJECT_NAME}_integration_tests)
    target_link_libraries(
      ${PROJECT_NAME}_integration_tests
      ${PROJECT_NAME}_core
    )
  endif()

  ##########################
  ## Static code analysis ##
  ##########################
  find_package(roslint QUIET)

  if(roslint_FOUND)
      roslint_cpp()
  else()
      message(WARNING "roslint not found. Skipping roslint target building")
  endif()
endif()

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (rosdoc_lite_FOUND)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
