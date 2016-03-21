cmake_minimum_required(VERSION 2.8.3)
project(snap_cam)

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

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	./libcamera/inc
	./libcamera/src
	)

find_library(camera_LIBRARIES NAMES camera PATHS /usr/lib)

if(camera_LIBRARIES)
    #found
    # add the snap_cam executable
    add_executable(snap_cam_publisher_node src/nodes/snap_cam_publisher_node.cpp src/snap_cam_publisher.cpp)
    target_link_libraries(snap_cam_publisher_node ${catkin_LIBRARIES} ${camera_LIBRARIES} ${OpenCV_LIBS})
endif()

# add the snap_cam executable
add_executable(optical_flow_node src/nodes/optical_flow_node.cpp)
target_link_libraries(optical_flow_node yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBS})
