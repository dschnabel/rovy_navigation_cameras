cmake_minimum_required(VERSION 2.8.3)
project(rovy_navigation_cameras)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp tf2_geometry_msgs tf cv_bridge image_transport)

include_directories(include ${catkin_INCLUDE_DIRS})

##executable##
catkin_package()
add_executable(rovy_navigation_cameras src/rovy_navigation_cameras.cpp)
##############


##library##
#catkin_package(INCLUDE_DIRS include
#    CATKIN_DEPENDS roscpp tf2_geometry_msgs tf cv_bridge image_transport
#    LIBRARIES rovy_navigation_cameras)
#add_library(rovy_navigation_cameras src/rovy_navigation_cameras.cpp)
#install(TARGETS rovy_navigation_cameras
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
###########

target_link_libraries(rovy_navigation_cameras ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-Wformat -Wformat-security -Wall -Wno-psabi -lrealsense2 ${CMAKE_CXX_FLAGS}")
