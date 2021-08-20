cmake_minimum_required(VERSION 2.8.3)
project(niryo_robot_debug)
message("\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  mcp_can_rpi
  niryo_robot_msgs
  roscpp
  rospy
  std_msgs
)
 
find_package(Boost REQUIRED COMPONENTS system program_options)

catkin_package(
  CATKIN_DEPENDS 
    dynamixel_sdk
    mcp_can_rpi
    niryo_robot_msgs
    roscpp
    rospy
    std_msgs
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

add_executable(dxl_debug_tools
    src/dxl_debug.cpp
    src/dxl_tools.cpp
)


## Specify libraries to link executable targets against
execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

# wiringPi should be installed only on a Raspberry Pi board
if(CMAKE_CROSSCOMPILING)
    message(STATUS "Cross compiling : wiringPi library is required")
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        -lwiringPi -lrt -lcrypt
    )
else()

    if ("${ARCHITECTURE}" MATCHES "^(arm.*|aarch64.*|arm64.*)$")
        message(STATUS "wiringdxl_debug_toolsPi library is required for ${PROJECT_NAME} (ARM processor)")
        target_link_libraries(dxl_debug_tools
            ${catkin_LIBRARIES}
            ${Boost_LIBRARIES}
            -lwiringPi -lrt -lcrypt
        )
    else()
        message(STATUS "wiringPi library not required")
        target_link_libraries(dxl_debug_tools
            ${catkin_LIBRARIES}
            ${Boost_LIBRARIES}
        )
    endif()
endif()

#############
## Install ##
#############

install(
  TARGETS dxl_debug_tools
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
    PROGRAMS scripts/send_custom_dxl_value.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)