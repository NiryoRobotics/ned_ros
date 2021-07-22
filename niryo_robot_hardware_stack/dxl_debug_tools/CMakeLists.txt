message("\n## Reading dxl_debug_tools/CMakeLists.txt...")

cmake_minimum_required(VERSION 3.0.2)
project(dxl_debug_tools)

option(BUILD_DOCS "Build documentation" OFF)
option(BUILD_UNIT_TESTS "Build the unit tests" OFF)

## c++ options
## Compile as C++14, supported in ROS Melodic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall  -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
      dynamixel_sdk
)
 
find_package(Boost REQUIRED
    COMPONENTS
        program_options
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
  CATKIN_DEPENDS 
    dynamixel_sdk
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Declare libs and execs
add_library(${PROJECT_NAME}_core
    src/dxl_tools.cpp)

add_executable(${PROJECT_NAME}
    src/dxl_debug.cpp)

target_link_libraries(${PROJECT_NAME} 
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

#############
##   Doc   ##
#############

if(BUILD_DOCS)
  find_program(rosdoc_lite_FOUND rosdoc_lite)
  if(rosdoc_lite_FOUND)
      add_custom_target (${PROJECT_NAME}_doc ALL
          rosdoc_lite -o ${CMAKE_CURRENT_BINARY_DIR}/doc ${CMAKE_CURRENT_LIST_DIR}
      )
  else()
    message(WARNING "rosdoc_lite not found. Skipping documentation building")
  endif()

endif(BUILD_DOCS)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  if(BUILD_UNIT_TESTS)
    catkin_add_gtest(
      ${PROJECT_NAME}_unit_tests
      test/dxl_tools_unit_tests.cpp
    )
    if(TARGET ${PROJECT_NAME}_unit_tests)
      target_link_libraries(${PROJECT_NAME}_unit_tests
          ${catkin_LIBRARIES}
          ${PROJECT_NAME}_core
      )
    endif()
  endif(BUILD_UNIT_TESTS)

endif(CATKIN_ENABLE_TESTING)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS 
    scripts/send_custom_dxl_value.py
    scripts/read_custom_dxl_value.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark other files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/doc/)
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
  )
endif()

##########################
## Static code analysis ##
##########################
find_package(roslint QUIET)

if(roslint_FOUND)
    roslint_cpp()
else()
    message(WARNING "roslint not found. Skipping roslint target building")
endif()
