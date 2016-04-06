/*
 * snap_cam_publisher.h
 *
 *  Created on: Mar 16, 2016
 *      Author: Christoph
 */

#ifndef SNAP_CAM_PUBLISHER_H
#define SNAP_CAM_PUBLISHER_H

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

#include "camera.h"
#include "camera_log.h"
#include "camera_parameters.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define DEFAULT_EXPOSURE_VALUE  250
#define MIN_EXPOSURE_VALUE 0
#define MAX_EXPOSURE_VALUE 65535
#define DEFAULT_GAIN_VALUE  50
#define MIN_GAIN_VALUE 0
#define MAX_GAIN_VALUE 255

#define DEFAULT_CAMERA_FPS 30
#define MS_PER_SEC 1000
#define NS_PER_MS 1000000
#define NS_PER_US 1000

const int SNAPSHOT_WIDTH_ALIGN = 64;
const int SNAPSHOT_HEIGHT_ALIGN = 64;
const int TAKEPICTURE_TIMEOUT_MS = 2000;

using namespace std;
using namespace camera;

struct CameraCaps
{
    vector<ImageSize> pSizes, vSizes, picSizes;
    vector<string> focusModes, wbModes, isoModes;
    Range brightness, sharpness, contrast;
    vector<Range> previewFpsRanges;
    vector<VideoFPS> videoFpsValues;
    vector<string> previewFormats;
    string rawSize;
};

enum OutputFormatType{
    YUV_FORMAT,
    RAW_FORMAT,
    JPEG_FORMAT
};

enum CamFunction {
    CAM_FUNC_HIRES = 0,
    CAM_FUNC_OPTIC_FLOW = 1,
    CAM_FUNC_RIGHT_SENSOR = 2,
    CAM_FUNC_STEREO = 3,
};

enum AppLoglevel {
    CAM_LOG_SILENT = 0,
    CAM_LOG_ERROR = 1,
    CAM_LOG_INFO = 2,
    CAM_LOG_DEBUG = 3,
    CAM_LOG_MAX,
};

/**
*  Helper class to store all parameter settings
*/
struct TestConfig
{
    bool dumpFrames;
    bool infoMode;
    bool testSnapshot;
    bool testVideo;
    int runTime;
    int exposureValue;
    int gainValue;
    CamFunction func;
    OutputFormatType outputFormat;
    OutputFormatType snapshotFormat;
    ImageSize pSize;
    ImageSize vSize;
    ImageSize picSize;
    int picSizeIdx;
    int fps;
    AppLoglevel logLevel;
};
/**
 * CLASS  CameraTest
 *
 * - inherits ICameraListers which provides core functionality
 * - User must define onPreviewFrame (virtual) function. It is
 *    the callback function for every preview frame.
 * - If user is using VideoStream then the user must define
 *    onVideoFrame (virtual) function. It is the callback
 *    function for every video frame.
 * - If any error occurs,  onError() callback function is
 *    called. User must define onError if error handling is
 *    required.
 */

class CameraTest : ICameraListener
{
public:

    CameraTest();
    ~CameraTest();
    int imagePublisher(ICameraFrame* frame);

    int initialize(int camId);

    /* listener methods */
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);

private:
    ICameraDevice* camera_;
    CameraParams params_;
    ImageSize pSize_, vSize_, picSize_;
    CameraCaps caps_;
    TestConfig config_;

    uint32_t vFrameCount_, pFrameCount_;
    float vFpsAvg_, pFpsAvg_;

    uint64_t vTimeStampPrev_, pTimeStampPrev_;

    pthread_cond_t cvPicDone;
    pthread_mutex_t mutexPicDone;
    bool isPicDone;

    int frameCounter;
    int camera_type;
    int camera_resolution;
    int camera_fps;
    int camera_used;
    std::string topic_name;

    ros::NodeHandle nh;
    image_transport::Publisher pub;

    int printCapabilities();
    int setParameters();
    int setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx);
};

#endif
