cmake_minimum_required(VERSION 3.0.2)
project(comm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -ggdb")
# set(CMAKE_FIND_DEBUG_MODE TRUE)    # cmake find_package() 搜索路径输出

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread signals)
find_package(Protobuf REQUIRED)
###########
## Build ##
###########
# message(FATAL_ERROR "${PROTOBUF_INCLUDE_DIRS}: " ${PROTOBUF_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_library(client_comm_lib STATIC
    client/socket_comm.cpp
    proto/pose.pb.cc
    proto/obs.pb.cc
    proto/control_cmd.pb.cc
)

target_link_libraries(client_comm_lib ${Protobuf_LIBRARIES})
target_include_directories(client_comm_lib PUBLIC
    ${PROTOBUF_INCLUDE_DIRS}
)

#############
## INSTALL ##
#############
install(TARGETS client_comm_lib
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
