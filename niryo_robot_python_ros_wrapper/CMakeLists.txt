cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_python_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
)
catkin_python_setup()

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
    add_rostest(test/rpi_python_wrapper.test)
else()
    add_rostest(test/simulation_python_wrapper.test)
endif()