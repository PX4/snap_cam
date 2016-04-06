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

#include "SnapCam.h"

ros::Publisher pub;

void imageCallback(const cv::Mat& img)
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
    pub.publish(im);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snap_cam_publisher");
    ros::NodeHandle nh("~");

    std::string topic_name;

    if (!nh.getParam("topic_name", topic_name)) {
        topic_name = "image";
        ROS_WARN("No topic name parameter provided. Defaulting to: %s.", topic_name.c_str());
    }

    pub = nh.advertise<sensor_msgs::Image>(topic_name.c_str(), 1);

    std::string res;
    if (!nh.getParam("resolution", res)) {
        res = "VGA";
        ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
    }

    std::string camera_type;
    if (!nh.getParam("camera_type", camera_type)) {
        camera_type = "highres";
        ROS_WARN("No camera type parameter provided. Defaulting to %s.", camera_type.c_str() );
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

    SnapCam cam(cfg);
    cam.setListener(imageCallback);

    ros::spin();

    return 0;
}
