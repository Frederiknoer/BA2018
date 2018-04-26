cmake_minimum_required(VERSION 3.9)
project(vis_test)

set(CMAKE_CXX_STANDARD 11)

#-------------------------------
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DEPENDENCIES realsense2 ${Boost_LIBRARIES})

add_executable(vis_test main.cpp PclPlane.cpp PclPlane.h rsCam.cpp rsCam.h Algorithms.cpp Algorithms.h)
target_link_libraries (vis_test ${PCL_LIBRARIES} ${DEPENDENCIES})

