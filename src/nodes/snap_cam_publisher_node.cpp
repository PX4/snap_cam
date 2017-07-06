/*
* optical_flow.cpp
*
*  Created on: Mar 16, 2016
*      Author: Nicolas
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <typeinfo>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "SnapCam.h"

image_transport::Publisher image_pub;


void imageCallback(const cv::Mat &img, uint64_t time_stamp)
{
	// convert OpenCV image to ROS message
	cv_bridge::CvImage cvi;
	cvi.header.stamp = ros::Time::now();
	cvi.header.frame_id = "image";
	cvi.image = img;

	if (img.channels() == 1) { // optical flow
		cvi.encoding = "mono8";

	} else { // highres
		cvi.encoding = "bgr8";
	}

	sensor_msgs::Image im;
	cvi.toImageMsg(im);
	image_pub.publish(im);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "snap_cam_publisher");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);

	std::string topic_name;

	if (!nh.getParam("topic_name", topic_name)) {
		topic_name = "image";
		ROS_WARN("No topic name parameter provided. Defaulting to: %s.", topic_name.c_str());
	}

	image_pub = it.advertise(topic_name.c_str(), 1);

	std::string res;

	if (!nh.getParam("camera_resolution", res)) {
		res = "VGA";
		ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
	}

	std::string camera_type;

	if (!nh.getParam("camera_type", camera_type)) {
		camera_type = "highres";
		ROS_WARN("No camera type parameter provided. Defaulting to %s.", camera_type.c_str());
	}

	int camera_fps;

	if (!nh.getParam("camera_fps", camera_fps)) {
		camera_fps = 30;
		ROS_WARN("No camera fps idx parameter provided. Defaulting to %d.", camera_fps);
	}

	int exposure;

	if (!nh.getParam("exposure", exposure)) {
		exposure = 100;
		ROS_WARN("No exposure parameter provided. Defaulting to %d.", exposure);
	}

	int camera_gain;

	if (!nh.getParam("gain", camera_gain)) {
		camera_gain = 0;
		ROS_WARN("No gain parameter provided. Defaulting to %d.", camera_gain);
	}

	bool auto_exposure;

	if (!nh.getParam("auto_exposure", auto_exposure)) {
		auto_exposure = false;
		ROS_WARN("Defaulting to no auto exposure");
	}

	CamConfig cfg;

	if (camera_type == "highres") {
		cfg.func = CAM_FUNC_HIRES;

	} else if (camera_type == "optflow") {
		cfg.func = CAM_FUNC_OPTIC_FLOW;

	} else {
		ROS_ERROR("Invalid camera type %s. Defaulting to highres.", camera_type.c_str());
		cfg.func = CAM_FUNC_HIRES;
	}

	if (res == "4k") {
		cfg.pSize = CameraSizes::UHDSize();

	} else if (res == "1080p") {
		cfg.pSize = CameraSizes::FHDSize();

	} else if (res == "720p") {
		cfg.pSize = CameraSizes::HDSize();

	} else if (res == "VGA") {
		cfg.pSize = CameraSizes::VGASize();

	} else if (res == "QVGA") {
		cfg.pSize = CameraSizes::QVGASize();

	} else if (res == "stereoVGA") {
		cfg.pSize = CameraSizes::stereoVGASize();

	} else if (res == "stereoQVGA") {
		cfg.pSize = CameraSizes::stereoQVGASize();

	} else {
		ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
		cfg.pSize = CameraSizes::stereoVGASize();
	}

	cfg.fps = camera_fps;
	cfg.exposureValue = exposure;
	cfg.gainValue = camera_gain;

	SnapCam cam(cfg);
	cam.setListener(imageCallback);
	if (auto_exposure) {
		ROS_INFO("Using auto exposure");
		cam.setAutoExposure(auto_exposure);
	}

	ros::spin();

	return 0;
}
