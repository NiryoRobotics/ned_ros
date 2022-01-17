cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_sdk)
message( STATUS "\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

set(HARDWARE_VERSION "ned2" CACHE STRING "Robot model target")
# for preprocessor directives
if ("${HARDWARE_VERSION}" STREQUAL "one")
    add_definitions(-DNIRYO_ONE)
elseif ("${HARDWARE_VERSION}" STREQUAL "ned")
    add_definitions(-DNIRYO_NED)
elseif ("${HARDWARE_VERSION}" STREQUAL "ned2")
    add_definitions(-DNIRYO_NED2)
endif()

#retrieve architecture
execute_process( COMMAND
            uname -m COMMAND tr -d '\n'
        OUTPUT_VARIABLE
            ARCHITECTURE
    )

## c++ options
## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
      serial
)

###############################################
# Declare ROS messages, services and actions ##
###############################################

###################################
## catkin specific configuration ##
###################################

## INCLUDE_DIRS: if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    serial
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

## Declare libs and execs
add_library(${PROJECT_NAME}
  src/group_bulk_read.cpp
  src/group_bulk_write.cpp
  src/group_sync_read.cpp
  src/group_sync_write.cpp
  src/packet_handler.cpp
  src/port_handler.cpp
  src/port_handler_linux.cpp
  src/protocol1_packet_handler.cpp
  src/protocol2_packet_handler.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against

# wiringPi should be installed only on a Raspberry Pi board (for ned and one only)
if ("${HARDWARE_VERSION}" STREQUAL "ned" OR "${HARDWARE_VERSION}" STREQUAL "one")
    if(CMAKE_CROSSCOMPILING)
        message(STATUS "Cross compiling : wiringPi library is required")
        target_link_libraries(${PROJECT_NAME}
            ${catkin_LIBRARIES}
            -lwiringPi -lrt -lcrypt
        )
    else()
        if ("${ARCHITECTURE}" MATCHES "^(arm.*|aarch64.*|arm64.*)$")
            message(STATUS "wiringPi library is required - arm processor")
            target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
                -lwiringPi -lrt -lcrypt
            )
        else()
            message(STATUS "wiringPi library not required")
            target_link_libraries(${PROJECT_NAME}
                ${catkin_LIBRARIES}
            )
        endif()
    endif()
endif()

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
# no testings : external lib

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
