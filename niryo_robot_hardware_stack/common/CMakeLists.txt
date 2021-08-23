cmake_minimum_required(VERSION 3.0.2)
project(common)
message("\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")


## c++ options
## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall  -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
      niryo_robot_msgs
      roscpp
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
    CATKIN_DEPENDS
        niryo_robot_msgs
        roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## Declare libs and execs
add_library(${PROJECT_NAME}
    src/model/abstract_hardware_state.cpp
    src/model/abstract_motor_state.cpp
    src/model/bus_protocol_enum.cpp
    src/model/button_type_enum.cpp
    src/model/conveyor_state.cpp
    src/model/dxl_command_type_enum.cpp
    src/model/dxl_motor_state.cpp
    src/model/end_effector_state.cpp
    src/model/end_effector_command_type_enum.cpp
    src/model/joint_state.cpp
    src/model/hardware_type_enum.cpp
    src/model/stepper_calibration_status_enum.cpp
    src/model/stepper_command_type_enum.cpp
    src/model/stepper_motor_state.cpp
    src/model/tool_state.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
  catkin_add_gtest(
    ${PROJECT_NAME}_unit_tests
    test/unit_tests.cpp
  )
  if(TARGET ${PROJECT_NAME}_unit_tests)
    target_link_libraries(
      ${PROJECT_NAME}_unit_tests
      ${PROJECT_NAME}
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
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

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
