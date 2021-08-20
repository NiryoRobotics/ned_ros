cmake_minimum_required(VERSION 2.8.3)
project(joints_interface)
message("\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        controller_manager
        dynamixel_driver
        hardware_interface
        niryo_robot_msgs
        roscpp
        sensor_msgs
        std_msgs
        stepper_driver

)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        ${PROJECT_NAME}
        ${PROJECT_NAME}_core
    CATKIN_DEPENDS
        actionlib
        controller_manager
        dynamixel_driver
        hardware_interface
        niryo_robot_msgs
        roscpp
        stepper_driver
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

add_library(${PROJECT_NAME}
    src/calibration_interface.cpp
    src/joint_state.cpp
    src/motor_offset_file_handler.cpp
)

add_library(${PROJECT_NAME}_core
    src/JointHardwareInterface.cpp
    src/joints_interface_core.cpp
)

add_executable(${PROJECT_NAME}_node
    src/joints_interface_node.cpp
)

# add_executable(${PROJECT_NAME}_test
#   test/joints_interface_test.cpp
# )

## Add dependencies to exported targets, like ROS msgs or srvs

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME} 
    niryo_robot_msgs_generate_messages_cpp
)

## Specify libraries to link executable targets against

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_core
  ${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)

# target_link_libraries(${PROJECT_NAME}_test
#   ${PROJECT_NAME}_core
# )

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
  TARGETS ${PROJECT_NAME}_core
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



##########################
## Static code analysis ##
##########################

find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_cpp()
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()
