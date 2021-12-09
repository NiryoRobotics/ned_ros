cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_description)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED)

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

# Mark other files for installation

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(
  DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
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
