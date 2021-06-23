#File raspberrytoolchain.cmake for ROS and system packages to cross compile.
SET(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

SET(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Below call is necessary to avoid non-RT problem.
SET(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

SET(RASPBERRY_ROOT_PATH /home/ccocq/Workspace/catkin_ws_niryo_ned/arm64_raspberry)
SET(RASPBERRY_MELODIC_PATH ${RASPBERRY_ROOT_PATH}/opt/ros/melodic)

SET(CMAKE_FIND_ROOT_PATH ${RASPBERRY_ROOT_PATH})

#If you have installed cross compiler to somewhere else, please specify that path.
#SET(COMPILER_ROOT /usr/aarch64-linux-gnu) 

#Have to set this one to BOTH, to allow CMake to find rospack
#This set of variables controls whether the CMAKE_FIND_ROOT_PATH and CMAKE_SYSROOT are used for find_xxx() operations.
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NONE)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
#SET(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

#SET(CMAKE_PREFIX_PATH ${RASPBERRY_MELODIC_PATH}) 

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
#SET(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
#SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)

include_directories(${RASPBERRY_ROOT_PATH}/usr/include ${RASPBERRY_ROOT_PATH}/usr/local/include)
SET(LD_LIBRARY_PATH ${RASPBERRY_MELODIC_PATH}/lib ${RASPBERRY_ROOT_PATH}/lib/aarch64-linux-gnu ${RASPBERRY_ROOT_PATH}/usr/local/lib ${RASPBERRY_ROOT_PATH}/usr/lib)
