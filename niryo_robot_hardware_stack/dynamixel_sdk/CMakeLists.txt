cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)
message("\n## Reading ${PROJECT_NAME}/CMakeLists.txt...")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
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

add_dependencies(${PROJECT_NAME} 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

#target_link_libraries(${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    -lwiringPi
#)

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


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


