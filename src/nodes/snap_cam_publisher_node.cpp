/*
 * snap_cam_publisher_node.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Christoph
 */

#include <ros/ros.h>

#include "snap_cam_publisher.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "snap_cam_image_publisher");

  CameraTest camera_test;

  ros::spin();
  return 0;
}
