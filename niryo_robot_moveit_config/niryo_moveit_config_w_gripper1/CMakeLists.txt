cmake_minimum_required(VERSION 3.0.2)
project(niryo_moveit_config_w_gripper1)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED
    COMPONENTS
    niryo_robot_description
)

catkin_package()

#############
##   Doc   ##
#############

#############
## Testing ##
#############

#############
## Install ##
#############

# no scripts to install

install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    # no static code analysis on external libs
    # create empty target (needed for catkin build --make-args roslint)
    message(STATUS "no lint analysis for this packet. Create empty target for conveniency")
    roslint_cpp(empty) # hack to create an empty roslint target
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()
