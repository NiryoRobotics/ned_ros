message("\n## Reading niryo_robot_python_ros_wrapper/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_python_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
  niryo_robot_arm_commander
  niryo_robot_tools_commander
)

# enable python module support
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  niryo_robot_msgs
  niryo_robot_arm_commander
  niryo_robot_tools_commander
)

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
    add_rostest(test/rpi_python_wrapper.test)
else()
    add_rostest(test/simulation_python_wrapper.test)
endif()


#############
## Install ##
#############

# no script to install

# Mark other files for installation
install(
  DIRECTORY examples/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()