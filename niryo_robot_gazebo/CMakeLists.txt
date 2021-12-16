cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_gazebo)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

# no dependency on gazebo because not present in niryo one
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
  DIRECTORY demo/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/demo
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media
)

install(
  DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(
  DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
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
