cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_bringup)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED)

catkin_package()

#############
##   Doc   ##
#############

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

  ##########################
  ## Static code analysis ##
  ##########################
  find_package(roslint QUIET)

  if(roslint_FOUND)
      roslint_cpp(empty)
  else()
      message(WARNING "roslint not found. Skipping roslint target building")
  endif()
endif()

#############
## Install ##
#############

# Mark other files for installation
install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
    DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS
)
