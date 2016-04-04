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

// supported image sizes TODO its not ideal that these have to be redefined here (originally defined in SnapCam.cpp)
ImageSize FourKSize(4096,2160);
ImageSize UHDSize(3840,2160);
ImageSize FHDSize(1920,1080);
ImageSize HDSize(1280,720);
ImageSize VGASize(640,480);
ImageSize stereoVGASize(1280, 480);
ImageSize QVGASize(320,240);
ImageSize stereoQVGASize(640,240);

void imageCallback(const cv::Mat& img)
{
    // convert OpenCV image to ROS message
    cv_bridge::CvImage cvi;
    cvi.header.stamp = ros::Time::now();
    cvi.header.frame_id = "image";
    cvi.encoding = "bgr8";
    cvi.image = img;

    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    pub.publish(im);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "snap_cam_publisher");
    ros::NodeHandle nh("~");

    std::string topic_name;

    if (nh.getParam("topic_name", topic_name)) {
        ROS_INFO("Got topic name: %s", topic_name.c_str());
    } else {
        std::string default_topicName = "image";
        ROS_INFO("Could not get topic_name: default topic name: %s", default_topicName.c_str());
        topic_name = default_topicName;
    }

    pub = nh.advertise<sensor_msgs::Image>(topic_name.c_str(), 1);

    std::string res;
    if (!nh.getParam("resolution", res)) {
        res = "VGA";
        ROS_WARN("Could not get topic_name. Defaulting to %s", res.c_str());
    }

    CamConfig cfg;
    cfg.func = CAM_FUNC_OPTIC_FLOW; // TODO get from ROS parameters
    if (res == "4k") {
        cfg.pSize = UHDSize;
    } else if (res == "1080p") {
        cfg.pSize = FHDSize;
    } else if (res == "720p") {
        cfg.pSize = HDSize;
    } else if (res == "VGA") {
        cfg.pSize = VGASize;
    } else if (res == "QVGA") {
        cfg.pSize = QVGASize;
    } else if (res == "stereoVGA") {
        cfg.pSize = stereoVGASize;
    } else if (res == "stereoQVGA") {
        cfg.pSize = stereoQVGASize;
    } else {
        printf("Invalid resolution specification %s. Defaulting to VGA\n", res.c_str());
        cfg.pSize = stereoVGASize;
    }

    SnapCam cam(cfg);
    cam.setListener(imageCallback);

    ros::spin();
}
