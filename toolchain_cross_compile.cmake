#File raspberrytoolchain.cmake for ROS and system packages to cross compile.
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
# Below call is necessary to avoid non-RT problem.
SET(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

SET(RASPBERRY_ROOT_PATH /home/ccocq/Workspace/catkin_ws_niryo_ned/arm64_raspberry)
SET(RASPBERRY_MELODIC_PATH ${RASPBERRY_ROOT_PATH}/opt/ros/melodic)

SET(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)


SET(CMAKE_FIND_ROOT_PATH ${RASPBERRY_ROOT_PATH})

#Have to set this one to BOTH, to allow CMake to find rospack
#This set of variables controls whether the CMAKE_FIND_ROOT_PATH and CMAKE_SYSROOT are used for find_xxx() operations.
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

include_directories(${RASPBERRY_ROOT_PATH}/usr/include ${RASPBERRY_ROOT_PATH}/usr/local/include)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)
SET(CMAKE_CXX_LINK_FLAGS "${CMAKE_CXX_LINK_FLAGS} --sysroot=${RASPBERRY_ROOT_PATH}" CACHE INTERNAL "" FORCE)

SET(LD_LIBRARY_PATH ${RASPBERRY_MELODIC_PATH}/lib ${RASPBERRY_ROOT_PATH}/lib/aarch64-linux-gnu ${RASPBERRY_ROOT_PATH}/usr/local/lib ${RASPBERRY_ROOT_PATH}/usr/lib)
