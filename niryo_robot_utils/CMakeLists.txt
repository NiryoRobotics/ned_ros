cmake_minimum_required(VERSION 3.0.2)
project(niryo_robot_utils)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED COMPONENTS
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

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
      set(ROSLINT_PYTHON_OPTS "1>&2")
      roslint_python()
  else()
      message(WARNING "roslint not found. Skipping roslint target building")
  endif()
endif()
# no testings : external lib


#############
## Install ##
#############

# no scripts to install

