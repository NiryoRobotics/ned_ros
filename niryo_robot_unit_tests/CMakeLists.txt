cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_unit_tests)
message("\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED COMPONENTS
  rospy
  rostest
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_python()
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()
