/*
* optical_flow.cpp
*
*  Created on: Mar 16, 2016
*      Author: Christoph
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
#include <math.h>
#include <typeinfo>
#include <time.h>

#include <opencv2/highgui/highgui.hpp>

#include "trackFeatures.h"
#include "calib_yaml_interface.h"

#include <mavlink/v1.0/mavlink_types.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "SnapCam.h"

#define UDP_PORT    14556
#define MAVLINK_MSG_ID_OPTICAL_FLOW_RAD 106

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

// supported image sizes TODO its not ideal that these have to be redefined here (originally defined in SnapCam.cpp)
ImageSize FourKSize(4096,2160);
ImageSize UHDSize(3840,2160);
ImageSize FHDSize(1920,1080);
ImageSize HDSize(1280,720);
ImageSize VGASize(640,480);
ImageSize stereoVGASize(1280, 480);
ImageSize QVGASize(320,240);
ImageSize stereoQVGASize(640,240);

//decalare variables
std::vector<cv::Point2f> features_current;
std::vector<int> updateVector;
cv::Mat_<cv::Point2f> out_features_previous;

double img_timestamp_prev;

CameraParameters cameraParams = {};

int _fd;
struct sockaddr_in _srcaddr;
socklen_t _addrlen;
unsigned char _buf[65535];

void loadCustomCameraCalibration(const std::string calib_path) {
    // load a camera calibration defined in the launch script
    try {
        YAML::Node YamlNode = YAML::LoadFile(calib_path);
        if (YamlNode.IsNull()) {
            printf("Failed to open camera calibration %s\n", calib_path.c_str());
            exit(-1);
        }
        cameraParams = parseYaml(YamlNode);
    } catch (YAML::BadFile &e) {
        printf("Failed to open camera calibration %s\nException: %s\n", calib_path.c_str(), e.what());
        exit(-1);
    }
}

void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID)
{
    component_ID = 0;
    uint8_t payload_len = mavlink_message_lengths[msgid];
    unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    /* header */
    buf[0] = MAVLINK_STX;
    buf[1] = payload_len;
    /* no idea which numbers should be here*/
    buf[2] = 100;
    buf[3] = 0;
    buf[4] = component_ID;
    buf[5] = msgid;

    /* payload */
    memcpy(&buf[MAVLINK_NUM_HEADER_BYTES], msg, payload_len);

    /* checksum */
    uint16_t checksum;
    crc_init(&checksum);
    crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
    crc_accumulate(mavlink_message_crcs[msgid], &checksum);

    buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
    buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

    ssize_t len = sendto(_fd, buf, packet_len, 0, (struct sockaddr *)&_srcaddr, _addrlen);

    if (len <= 0) {
        printf("Failed sending mavlink message\n");
    }
}

void sendOptFlowMessage (double timestamp, double dt, double flow_x, double flow_y, double quality) {
    //create and send optical flow mavlink message to px4
    mavlink_optical_flow_rad_t sensor_msg;

    sensor_msg.time_usec = timestamp * 1000000.0;
    sensor_msg.sensor_id = 0;//?
    sensor_msg.integration_time_us = dt;
    sensor_msg.integrated_x = flow_y;
    sensor_msg.integrated_y = - flow_x;
    sensor_msg.integrated_xgyro = 0.0;
    sensor_msg.integrated_ygyro = 0.0;
    sensor_msg.integrated_zgyro = 0.0;
    sensor_msg.temperature = 0.0;
    sensor_msg.quality = quality;
    sensor_msg.time_delta_distance_us = 0.0;
    sensor_msg.distance = 0.0;
    send_mavlink_message(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &sensor_msg, 200);
}

void calcOptFlow(const cv::Mat& Image, double img_timestamp)
{
    std::vector<cv::Point2f> useless;
    int meancount = 0;

    double pixel_flow_x_integral = 0.0;
    double pixel_flow_y_integral = 0.0;

    trackFeatures(Image, Image, features_current, useless, updateVector, 0);

    cv::Mat_<float> cam_matrix(3,3);
    cam_matrix <<   cameraParams.CameraMatrix[0][0], cameraParams.CameraMatrix[0][1], cameraParams.CameraMatrix[0][2],
    cameraParams.CameraMatrix[1][0], cameraParams.CameraMatrix[1][1], cameraParams.CameraMatrix[1][2],
    cameraParams.CameraMatrix[2][0], cameraParams.CameraMatrix[2][1], cameraParams.CameraMatrix[2][2];
    cv::Mat_<float> distortion(1,4);
    distortion <<   cameraParams.RadialDistortion[0], cameraParams.RadialDistortion[1], 0, 0, cameraParams.RadialDistortion[2];

    int npoints = updateVector.size();
    cv::Mat_<cv::Point2f> out_features_current(1,npoints);

    cv::undistortPoints(features_current, out_features_current, cam_matrix, distortion);

    //cv::undistortPoints returns normalized coordinates... -> convert
    for (int i = 0; i < npoints; i++) {
        out_features_current(i).x = out_features_current(i).x * cameraParams.CameraMatrix[0][0] + cameraParams.CameraMatrix[0][2];
        out_features_current(i).y = out_features_current(i).y * cameraParams.CameraMatrix[1][1] + cameraParams.CameraMatrix[1][2];
    }

    if (!out_features_current.empty() && !out_features_previous.empty()) {
        //calc diff
        for (int i = 0; i < updateVector.size(); i++) {

            if (updateVector[i] == 1) {
                pixel_flow_x_integral += out_features_current(i).x - out_features_previous(i).x;
                pixel_flow_y_integral += out_features_current(i).y - out_features_previous(i).y;
                meancount++;
            }
            if (updateVector[i] == 2)
            updateVector[i] = 1;
            if (updateVector[i] == 0)
            updateVector[i] = 2;
        }
    }

    if (meancount) {
        pixel_flow_x_integral /= meancount;
        pixel_flow_y_integral /= meancount;
    }

    double flow_quality = 255.0*meancount/updateVector.size();
    double delta_time = img_timestamp - img_timestamp_prev;

    double flow_x_ang = atan2(pixel_flow_x_integral, cameraParams.FocalLength[0]);
    double flow_y_ang = atan2(pixel_flow_y_integral, cameraParams.FocalLength[1]);

    out_features_previous = out_features_current;
    img_timestamp_prev = img_timestamp;

    sendOptFlowMessage(img_timestamp, delta_time, flow_x_ang, flow_y_ang, flow_quality);
}

void imageCallback(const cv::Mat& img)
{
    calcOptFlow(img, time(0));
}

int parseCommandline(int argc, char* argv[], std::string &res, std::string &calib_path)
{
    // set default values
    res = "VGA";
    calib_path = "cameraParams.yaml";

    int c;
    int ret = 0;

    while ((c = getopt(argc, argv, "c:r:")) != -1) {
        switch (c) {
        case 'c':
            {
                res =  std::string(optarg);
                break;
            }
        case 'r':
            {
                res = std::string(optarg);
                break;
            }
        case '?':
            ret = 1;
            break;
        default:
            ret = 1;
            break;
        }
    }

    return ret;
}

int main(int argc, char **argv)
{
    std::string res;
    std::string calibration_path;

    parseCommandline(argc, argv, res, calibration_path);

    CamConfig cfg;
    cfg.func = CAM_FUNC_OPTIC_FLOW;
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

    // try to setup udp socket for communcation
    if ((_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("create socket failed\n");
        return 0;
    }

    _srcaddr.sin_family = AF_INET;
    _srcaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    _srcaddr.sin_port = htons(UDP_PORT);

    _addrlen = sizeof(_srcaddr);

    updateVector.resize(100, 2);

    SnapCam cam(cfg);
    cam.setListener(imageCallback);

    while(1)
    {
        usleep(1e6);
    }
}
