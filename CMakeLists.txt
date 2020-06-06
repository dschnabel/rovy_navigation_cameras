cmake_minimum_required(VERSION 2.8.3)
project(rovy_navigation_cameras)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf2_geometry_msgs tf cv_bridge image_transport)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rovy_navigation_cameras src/rovy_navigation_cameras.cpp)
target_link_libraries(rovy_navigation_cameras ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-Wformat -Wformat-security -Wall -Wno-psabi -lrealsense2 ${CMAKE_CXX_FLAGS}")