cmake_minimum_required(VERSION 2.8.3)
project(snap_cam)

# check if this is being built as a ROS package
if (CATKIN_DEVEL_PREFIX)
    set(IS_CATKIN 1)
    message(STATUS "Building this package with catkin")
else()
    set(IS_CATKIN 0)
    message(STATUS "Building this package without catkin")
endif()

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -Wno-format-security")

find_package(OpenCV REQUIRED)

if(IS_CATKIN)
    ## Find catkin macros and libraries
    ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
    ## is used, also find other catkin packages
    find_package(catkin REQUIRED COMPONENTS
      cv_bridge
      image_transport
      klt_feature_tracker
      sensor_msgs
      roscpp
      rospy
      sensor_msgs
      std_msgs
      mavlink
    )

    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES snap_cam
    #  CATKIN_DEPENDS catkin_lib
      DEPENDS yaml-cpp
    )
endif()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	./libcamera/inc
	./libcamera/src
	)

find_library(camera_LIBRARIES NAMES camera PATHS /usr/lib)

if (NOT camera_LIBRARIES)
    message(FATAL_ERROR "Could not find camera libraries")
endif()

## Declare a C++ library
add_library(snap_cam
  src/SnapCam.cpp
)
target_link_libraries(snap_cam
  ${OpenCV_LIBS}
  ${camera_LIBRARIES}
)

add_executable(optical_flow src/optical_flow.cpp)
target_link_libraries(optical_flow yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS} snap_cam)

# add the ROS executables
if(IS_CATKIN)
    add_executable(snap_cam_publisher src/nodes/snap_cam_publisher_node.cpp)
    target_link_libraries(snap_cam_publisher yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS} snap_cam)
endif()
