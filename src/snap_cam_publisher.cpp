/* Copyright (c) 2015, The Linux Foundataion. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*************************************************************************
*
*Application Notes:
*
*Camera selection:
*  Each camera is given a unique function id in the sensor driver.
*   HIRES = 0, OPTIC_FLOW = 1, LEFT SENSOR = 2, STEREO = 3
*  getNumberOfCameras gives information on number of camera connected on target.
*  getCameraInfo provides information on each camera loop.
*  camid is obtained by looping through available cameras and matching info.func
*  with the requested camera.
*
*Camera configuration:
*
* Optic flow:
*   Do not set the following parameters :
*    PictureSize
*    focus mode
*    white balance
*    ISO
*    preview format
*
*  The following parameters can be set only after starting preview :
*    Manual exposure
*    Manual gain
*
*  Notes:
*    Snapshot is not supported for optic flow.
*
*  How to enable RAW mode for optic flow sensor ?
*    RAW stream is only available for OV7251
*    RAW stream currently returns images in the preview callback.
*    When configuring RAW stream, video stream on the same sensor must not be enabled. Else you will not see preview callbacks.
*    When configuration RAW, these parameters must be set  in addition to other parameters for optic flow
*                params_.set("preview-format", "bayer-rggb");
*                params_.set("picture-format", "bayer-mipi-10gbrg");
*                params_.set("raw-size", "640x480");
*
*
*  Stereo:
*    Do not set the following parameters :
*     PictureSize
*     focus mode
*     white balance
*     ISO
*     preview format
*
*   The following parameters can be set only after starting preview :
*     Manual exposure
*     Manual gain
*     setVerticalFlip
*     setHorizontalMirror
*  left/right:
*    code is written with reference to schematic but for end users the left and right sensor appears swapped.
*    so for user experience in the current app left option is changed to right.
*    right sensor with reference to schematic always goes in stereo mode, so no left option for end users.
*
* How to perform vertical flip and horizontal mirror on individual images in stereo ?
*  In stereo since the image is merged,
*  it makes it harder to perform these operation on individual images which may be required based on  senor  orientation on target.
*  setVerticalFlip and setHorizontalMirror perform  perform these operation by changing the output configuration from the sensor.
*
*
* How to set FPS
*  Preview fps is set using the function : setPreviewFpsRange
*  Video fps is set using the function : setVideoFPS
*  setFPSindex scans through the supported fps values and returns index of requested fps in the array of supported fps.
*
* How to change the format to NV12
*  To change the format to NV12 use the "preview-format" key.
*  params.set(std::string("preview-format"), std::string("nv12"));
*
****************************************************************************/
/*
 * snap_cam_publisher.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Christoph
 */

#include "snap_cam_publisher.h"

using namespace std;
using namespace camera;

ImageSize FourKSize(4096,2160);
ImageSize UHDSize(3840,2160);
ImageSize FHDSize(1920,1080);
ImageSize HDSize(1280,720);
ImageSize VGASize(640,480);
ImageSize stereoVGASize(1280, 480);
ImageSize QVGASize(320,240);
ImageSize stereoQVGASize(640,240);

CameraTest::CameraTest()
{
    image_transport::ImageTransport it(nh);
    ros::NodeHandle _nh("~");

    //get parameters from launch file
    if(!_nh.getParam("camera_type", camera_type)) {
        ROS_INFO("Could not get camera type: default optical flow");
        camera_type = 0;
    }
    if(!_nh.getParam("camera_resolution", camera_resolution)) {
        ROS_INFO("Could not get camera resolution: default QVGA");
        camera_resolution = 0;
    }
    if(!_nh.getParam("camera_fps", camera_fps)) {
        ROS_INFO("Could not get camera fps: default 15");
        camera_fps = 0;
    }
    if(!_nh.getParam("topic_name", topic_name)) {
        std::string default_topicName = "snap_cam/image";
        ROS_INFO("Could not get topic_name: default topic name: %s", default_topicName.c_str());
        topic_name = default_topicName;
    }

    if (camera_type == 0 || camera_type == 1){
      initialize(camera_type);
      camera_used = camera_type;
    } else {
      camera_used = 0;
      initialize(0);
    }
    //printCapabilities();

    int pFpsIdx;
    if (camera_fps >= 0 && camera_fps <= 6)
      pFpsIdx = camera_fps;
    else
      pFpsIdx = 0;

    if (camera_type == 1) { //highres camera
      switch(camera_resolution) {
        case 1 :  { pSize_ = VGASize;
                    break;
                  }
        case 2 :  { pSize_ = HDSize;
                    break;
                  }
        case 3 :  { pSize_ = FHDSize;
                    break;
                  }
        case 4 :  { pSize_ = FourKSize;
                    break;
                  }
        default:  { pSize_ = QVGASize;
                    break;
                  }
      }
    } else {
      switch(camera_resolution) {
        case 1 :  { pSize_ = VGASize;
                    break;
                  }
        default:  { pSize_ = QVGASize;
                    break;
                  }
      }
    }


    frameCounter = 0;
    //duration
    int sleeptime = 300; //seconds

    //set parameters
    int focusModeIdx = 0; //fixed
    int wbModeIdx = 0; //whitebalance 0: auto 1: incandescent 2: fluorescent 3: warm-fluorescent 4: daylight 5: cloudy-daylight 6: twilight 7: shade 8: manual-cct
    int isoModeIdx = 0; //auto

    //params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);
    params_.setFocusMode(caps_.focusModes[focusModeIdx]);
    params_.setWhiteBalance(caps_.wbModes[wbModeIdx]);
    params_.setISO(caps_.isoModes[isoModeIdx]);
    params_.setVideoSize(pSize_);
    params_.setPreviewFormat(caps_.previewFormats[0]); //0:yuv420sp 1:yuv420p 2:nv12-venus 3:bayer-rggb
    params_.setPreviewSize(pSize_);
    params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);

    int rc = params_.commit();
    if (rc) {
        printf("commit failed\n");
        //goto delete_camera;
    } else {
        //printf("start recording\n");
        //camera_->startRecording();
        pub = it.advertise(topic_name, 1);
        printf("start preview\n");
        camera_->startPreview();
    }
}

int CameraTest::initialize(int camId)
{
    int rc;
    rc = ICameraDevice::createInstance(camId, &camera_);
    if (rc != 0) {
        printf("could not open camera %d\n", camId);
        return rc;
    }
    camera_->addListener(this);

    rc = params_.init(camera_);
    if (rc != 0) {
        printf("failed to init parameters\n");
        ICameraDevice::deleteInstance(&camera_);
        return rc;
    }
    //printf("params = %s\n", params_.toString().c_str());
    /* query capabilities */
    caps_.pSizes = params_.getSupportedPreviewSizes();
    caps_.vSizes = params_.getSupportedVideoSizes();
    caps_.picSizes = params_.getSupportedPictureSizes();
    caps_.focusModes = params_.getSupportedFocusModes();
    caps_.wbModes = params_.getSupportedWhiteBalance();
    caps_.isoModes = params_.getSupportedISO();
    caps_.brightness = params_.getSupportedBrightness();
    caps_.sharpness = params_.getSupportedSharpness();
    caps_.contrast = params_.getSupportedContrast();
    caps_.previewFpsRanges = params_.getSupportedPreviewFpsRanges();
    caps_.videoFpsValues = params_.getSupportedVideoFps();
    caps_.previewFormats = params_.getSupportedPreviewFormats();
    caps_.rawSize = params_.get("raw-size");
}

CameraTest::~CameraTest()
{
  printf("stop preview. frame counter = %d\n", frameCounter);
  camera_->stopPreview();

  /* release camera device */
  ICameraDevice::deleteInstance(&camera_);
}

static int dumpToFile(uint8_t* data, uint32_t size, char* name, uint64_t timestamp)
{
    FILE* fp;
    fp = fopen(name, "wb");
    if (!fp) {
        printf("fopen failed for %s\n", name);
        return -1;
    }
    fwrite(data, size, 1, fp);
    printf("saved filename %s\n", name);
    fclose(fp);
}

static inline uint32_t align_size(uint32_t size, uint32_t align)
{
    return ((size + align - 1) & ~(align-1));
}

void CameraTest::onError()
{
    printf("camera error!, aborting\n");
    exit(EXIT_FAILURE);
}

/**
 *
 * FUNCTION: onPreviewFrame
 *
 *  - This is called every frame I
 *  - In the test app, we save files only after every 30 frames
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void CameraTest::onPreviewFrame(ICameraFrame* frame)
{
    /*if (pFrameCount_ > 0 && pFrameCount_ % 30 == 0) {
        char name[50];

        if ( config_.outputFormat == RAW_FORMAT )
        {
            snprintf(name, 50, "P_%dx%d_%04d_%llu.raw",
                 pSize_.width, pSize_.height, pFrameCount_,frame->timeStamp);
        }else{
             snprintf(name, 50, "P_%dx%d_%04d_%llu.yuv",
                 pSize_.width, pSize_.height, pFrameCount_,frame->timeStamp);
        }

        if (config_.dumpFrames == true) {
            dumpToFile(frame->data, frame->size, name, frame->timeStamp);
        }
        //printf("Preview FPS = %.2f\n", pFpsAvg_);
    }

    uint64_t diff = frame->timeStamp - pTimeStampPrev_;
    pFpsAvg_ = ((pFpsAvg_ * pFrameCount_) + (1e9 / diff)) / (pFrameCount_ + 1);
    pFrameCount_++;
    pTimeStampPrev_  = frame->timeStamp;*/

    //-----------------------------------------------
    frameCounter++;
    imagePublisher(frame);
}

/**
 *
 * FUNCTION: onVideoFrame
 *
 *  - This is called every frame I
 *  - In the test app, we save files only after every 30 frames
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void CameraTest::onVideoFrame(ICameraFrame* frame)
{
    /*if (vFrameCount_ > 0 && vFrameCount_ % 30 == 0) {
        char name[50];
        snprintf(name, 50, "V_%dx%d_%04d_%llu.yuv",
                 vSize_.width, vSize_.height, vFrameCount_,frame->timeStamp);
        if (config_.dumpFrames == true) {
            dumpToFile(frame->data, frame->size, name, frame->timeStamp);
        }
        //printf("Video FPS = %.2f\n", vFpsAvg_);
        //printf("test frame data: %d\n", (int)*frame->data);
    }

    uint64_t diff = frame->timeStamp - vTimeStampPrev_;
    vFpsAvg_ = ((vFpsAvg_ * vFrameCount_) + (1e9 / diff)) / (vFrameCount_ + 1);
    vFrameCount_++;
    vTimeStampPrev_  = frame->timeStamp;*/
}

int CameraTest::printCapabilities()
{
    printf("Camera capabilities\n");

    printf("available preview sizes:\n");
    for (int i = 0; i < caps_.pSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.pSizes[i].width, caps_.pSizes[i].height);
    }
    printf("available video sizes:\n");
    for (int i = 0; i < caps_.vSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.vSizes[i].width, caps_.vSizes[i].height);
    }
    printf("available picture sizes:\n");
    for (int i = 0; i < caps_.picSizes.size(); i++) {
        printf("%d: %d x %d\n", i, caps_.picSizes[i].width, caps_.picSizes[i].height);
    }
    printf("available preview formats:\n");
    for (int i = 0; i < caps_.previewFormats.size(); i++) {
        printf("%d: %s\n", i, caps_.previewFormats[i].c_str());
    }
    printf("available focus modes:\n");
    for (int i = 0; i < caps_.focusModes.size(); i++) {
        printf("%d: %s\n", i, caps_.focusModes[i].c_str());
    }
    printf("available whitebalance modes:\n");
    for (int i = 0; i < caps_.wbModes.size(); i++) {
        printf("%d: %s\n", i, caps_.wbModes[i].c_str());
    }
    printf("available ISO modes:\n");
    for (int i = 0; i < caps_.isoModes.size(); i++) {
        printf("%d: %s\n", i, caps_.isoModes[i].c_str());
    }
    printf("available brightness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.brightness.min,
           caps_.brightness.max, caps_.brightness.step);
    printf("available sharpness values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.sharpness.min,
           caps_.sharpness.max, caps_.sharpness.step);
    printf("available contrast values:\n");
    printf("min=%d, max=%d, step=%d\n", caps_.contrast.min,
           caps_.contrast.max, caps_.contrast.step);

    printf("available preview fps ranges:\n");
    for (int i = 0; i < caps_.previewFpsRanges.size(); i++) {
        printf("%d: [%d, %d]\n", i, caps_.previewFpsRanges[i].min,
               caps_.previewFpsRanges[i].max);
    }
    printf("available video fps values:\n");
    for (int i = 0; i < caps_.videoFpsValues.size(); i++) {
        printf("%d: %d\n", i, caps_.videoFpsValues[i]);
    }
    return 0;
}

const char usageStr[] =
    "Camera API test application \n"
    "\n"
    "usage: camera-test [options]\n"
    "\n"
    "  -t <duration>   capture duration in seconds [10]\n"
    "  -d              dump frames\n"
    "  -i              info mode\n"
    "                    - print camera capabilities\n"
    "                    - streaming will not be started\n"
    "  -f <type>       camera type\n"
    "                    - hires\n"
    "                    - optic\n"
    "                    - right \n"
    "                    - stereo \n"
    "  -p <size>       Set resolution for preview frame\n"
    "                    - 4k             ( imx sensor only ) \n"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of optic flow and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "                    - stereoVGA      ( 1280x480 : Stereo only - Max resolution )\n"
    "                    - stereoQVGA     ( 640x240  : Stereo only )\n"
    "  -v <size>       Set resolution for video frame\n"
    "                    - 4k             ( imx sensor only ) \n"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of optic flow and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "                    - stereoVGA      ( 1280x480 : Stereo only - Max resolution )\n"
    "                    - stereoQVGA     ( 640x240  : Stereo only )\n"
    "                    - disable        ( do not start video stream )\n"
    "  -n              take a picture with  max resolution of camera ( disabled by default)\n"
    "                  $camera-test -f <type> -i to find max picture size\n"
    "  -s <size>       take pickture at set resolution ( disabled by default) \n"
    "                    - 4k             ( imx sensor only ) \n"
    "                    - 1080p          ( imx sensor only ) \n"
    "                    - 720p           ( imx sensor only ) \n"
    "                    - VGA            ( Max resolution of optic flow and right sensor )\n"
    "                    - QVGA           ( 320x240 ) \n"
    "                    - stereoVGA      ( 1280x480 : Stereo only - Max resolution )\n"
    "                    - stereoQVGA     ( 640x240  : Stereo only )\n"
    "  -e <value>      set exposure control (only for ov7251)\n"
    "                     min - 0\n"
    "                     max - 65535\n"
    "  -g <value>      set gain value (only for ov7251)\n"
    "                     min - 0\n"
    "                     max - 255\n"
    "  -r < value>     set fps value      (Enter supported fps for requested resolution) \n"
    "                    -  30 (default)\n"
    "                    -  60 \n"
    "                    -  90 \n"
    "  -o <value>      Output format\n"
    "                     0 :YUV format (default)\n"
    "                     1 : RAW format (default of optic)\n"
    "  -j <value>      Snapshot Format\n"
    "                     jpeg : JPEG format (default)\n"
    "                     raw  : Full-size MIPI RAW format\n"
    "  -V <level>      syslog level [0]\n"
    "                    0: silent\n"
    "                    1: error\n"
    "                    2: info\n"
    "                    3: debug\n"
    "  -h              print this message\n"
;

static inline void printUsageExit(int code)
{
    printf("%s", usageStr);
    exit(code);
}
/**
 * FUNCTION: setFPSindex
 *
 * scans through the supported fps values and returns index of
 * requested fps in the array of supported fps
 *
 * @param fps      : Required FPS  (Input)
 * @param pFpsIdx  : preview fps index (output)
 * @param vFpsIdx  : video fps index   (output)
 *
 *  */
int CameraTest::setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx)
{
    int defaultPrevFPSIndex = -1;
    int defaultVideoFPSIndex = -1;
    int i,rc = 0;
    for (i = 0; i < caps_.previewFpsRanges.size(); i++) {
        if (  (caps_.previewFpsRanges[i].max)/1000 == fps )
        {
            pFpsIdx = i;
            break;
        }
        if ( (caps_.previewFpsRanges[i].max)/1000 == DEFAULT_CAMERA_FPS )
        {
            defaultPrevFPSIndex = i;
        }
    }
    if ( i >= caps_.previewFpsRanges.size() )
    {
        if (defaultPrevFPSIndex != -1 )
        {
            pFpsIdx = defaultPrevFPSIndex;
        } else
        {
            pFpsIdx = -1;
            rc = -1;
        }
    }

    for (i = 0; i < caps_.videoFpsValues.size(); i++) {
        if ( fps == caps_.videoFpsValues[i])
        {
            vFpsIdx = i;
            break;
        }
        if ( DEFAULT_CAMERA_FPS == caps_.videoFpsValues[i])
        {
            defaultVideoFPSIndex = i;
        }
    }
    if ( i >= caps_.videoFpsValues.size())
    {
        if (defaultVideoFPSIndex != -1)
        {
            vFpsIdx = defaultVideoFPSIndex;
        }else
        {
            vFpsIdx = -1;
            rc = -1;
        }
    }
    return rc;
}
/**
 *  FUNCTION : setParameters
 *
 *  - When camera is opened, it is initialized with default set
 *    of parameters.
 *  - This function sets required parameters based on camera and
 *    usecase
 *  - params_setXXX and params_set  only updates parameter
 *    values in a local object.
 *  - params_.commit() function will update the hardware
 *    settings with the current state of the parameter object
 *  - Some functionality will not be application for all for
 *    sensor modules. for eg. optic flow sensor does not support
 *    autofocus/focus mode.
 *  - Reference setting for different sensors and format are
 *    provided in this function.
 *
 *  */
int CameraTest::setParameters()
{
    int focusModeIdx = 0;
    int wbModeIdx = 2;
    int isoModeIdx = 0;
    int pFpsIdx = 3;
    int vFpsIdx = 0;
    int prevFmtIdx = 3;
    int rc = 0;

    pSize_ = config_.pSize;
    vSize_ = config_.vSize;
    picSize_ = config_.picSize;

	switch ( config_.func ){
		case CAM_FUNC_OPTIC_FLOW:
			if (config_.outputFormat == RAW_FORMAT) {
				/* Do not turn on videostream for optic flow in RAW format */
				config_.testVideo = false;
				printf("Setting output = RAW_FORMAT for optic flow sensor \n");
				params_.set("preview-format", "bayer-rggb");
				params_.set("picture-format", "bayer-mipi-10gbrg");
				params_.set("raw-size", "640x480");
			}
			break;
		case CAM_FUNC_RIGHT_SENSOR:
			break;
		case CAM_FUNC_STEREO:
			break;
		case CAM_FUNC_HIRES:
		    if (config_.picSizeIdx != -1 ) {
		        picSize_ = caps_.picSizes[config_.picSizeIdx];
		        config_.picSize = picSize_;
		    } else {
		        picSize_ = config_.picSize;
		    }
                    if (config_.snapshotFormat == RAW_FORMAT) {
                        printf("raw picture format: %s\n",
                               "bayer-mipi-10bggr");
                        params_.set("picture-format",
                                     "bayer-mipi-10bggr");
                        printf("raw picture size: %s\n", caps_.rawSize.c_str());
                    } else {
                        printf("setting picture size: %dx%d\n",
                                picSize_.width, picSize_.height);
                        params_.setPictureSize(picSize_);
                    }

			printf("setting focus mode: %s\n",
				 caps_.focusModes[focusModeIdx].c_str());
			params_.setFocusMode(caps_.focusModes[focusModeIdx]);
			printf("setting WB mode: %s\n", caps_.wbModes[wbModeIdx].c_str());
			params_.setWhiteBalance(caps_.wbModes[wbModeIdx]);
			printf("setting ISO mode: %s\n", caps_.isoModes[isoModeIdx].c_str());
			params_.setISO(caps_.isoModes[isoModeIdx]);

			printf("setting preview format: %s\n",
				 caps_.previewFormats[prevFmtIdx].c_str());
			params_.setPreviewFormat(caps_.previewFormats[prevFmtIdx]);
			break;
		default:
			printf("invalid sensor function \n");
			break;
	}
    printf("setting preview size: %dx%d\n", pSize_.width, pSize_.height);
    params_.setPreviewSize(pSize_);
    printf("setting video size: %dx%d\n", vSize_.width, vSize_.height);
    params_.setVideoSize(vSize_);

    /* Find index and set FPS  */
    rc = setFPSindex(config_.fps, pFpsIdx, vFpsIdx);
    if ( rc == -1)
    {
        return rc;
    }
    printf("setting preview fps range: %d, %d ( idx = %d ) \n",
    caps_.previewFpsRanges[pFpsIdx].min,
    caps_.previewFpsRanges[pFpsIdx].max, pFpsIdx);
    params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);
    printf("setting video fps: %d ( idx = %d )\n", caps_.videoFpsValues[vFpsIdx], vFpsIdx );
    params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);


    return params_.commit();
}

/**
 *  FUNCTION: setDefaultConfig
 *
 *  set default config based on camera module
 *
 * */
static int setDefaultConfig(TestConfig &cfg) {

    cfg.outputFormat = YUV_FORMAT;
    cfg.dumpFrames = false;
    cfg.runTime = 10;
    cfg.infoMode = false;
    cfg.testVideo = true;
    cfg.testSnapshot = false;
    cfg.exposureValue = DEFAULT_EXPOSURE_VALUE;  /* Default exposure value */
    cfg.gainValue = DEFAULT_GAIN_VALUE;  /* Default gain value */
    cfg.fps = DEFAULT_CAMERA_FPS;
    cfg.picSizeIdx = -1;
    cfg.logLevel = CAM_LOG_SILENT;
    cfg.snapshotFormat = JPEG_FORMAT;

    switch (cfg.func) {
    case CAM_FUNC_OPTIC_FLOW:
        cfg.pSize   = VGASize;
        cfg.vSize   = VGASize;
        cfg.picSize   = VGASize;
        cfg.outputFormat = RAW_FORMAT;
        break;
    case CAM_FUNC_RIGHT_SENSOR:
        cfg.pSize   = VGASize;
        cfg.vSize   = VGASize;
        cfg.picSize   = VGASize;
        break;
    case CAM_FUNC_STEREO:
        cfg.pSize = stereoVGASize;
        cfg.vSize  = stereoVGASize;
        cfg.picSize  = stereoVGASize;
        break;
    case CAM_FUNC_HIRES:
        cfg.pSize = FHDSize;
        cfg.vSize = HDSize;
        cfg.picSize = FHDSize;
        break;
    default:
        printf("invalid sensor function \n");
        break;
    }

}

/**
 *  FUNCTION: parseCommandline
 *
 *  parses commandline options and populates the config
 *  data structure
 *
 *  */
static TestConfig parseCommandline(int argc, char* argv[])
{
    TestConfig cfg;
    cfg.func = CAM_FUNC_HIRES;

    int c;
    int outputFormat;
    int exposureValueInt = 0;
    int gainValueInt = 0;

    while ((c = getopt(argc, argv, "hdt:io:e:g:p:v:ns:f:r:V:j:")) != -1) {
        switch (c) {
        case 'f':
            {
                string str(optarg);
                if (str == "hires") {
                    cfg.func = CAM_FUNC_HIRES;
                } else if (str == "optic") {
                    cfg.func = CAM_FUNC_OPTIC_FLOW;
                } else if (str == "right") {
                    cfg.func = CAM_FUNC_RIGHT_SENSOR;
                } else if (str == "stereo") {
                    cfg.func = CAM_FUNC_STEREO;
                }
                break;
            }
        case '?':
            break;
        default:
            break;
        }
    }
    setDefaultConfig(cfg);

    optind = 1;
    while ((c = getopt(argc, argv, "hdt:io:e:g:p:v:ns:f:r:V:j:")) != -1) {
        switch (c) {
        case 't':
            cfg.runTime = atoi(optarg);
            break;
         case 'p':
            {
                string str(optarg);
                if (str == "4k") {
                    cfg.pSize = UHDSize;
                } else if (str == "1080p") {
                    cfg.pSize = FHDSize;
                } else if (str == "720p") {
                    cfg.pSize = HDSize;
                } else if (str == "VGA") {
                    cfg.pSize = VGASize;
                } else if (str == "QVGA") {
                    cfg.pSize = QVGASize;
                } else if (str == "stereoVGA") {
                    cfg.pSize = stereoVGASize;
                } else if (str == "stereoQVGA") {
                    cfg.pSize = stereoQVGASize;
                }
                break;
            }
        case 'v':
            {
                string str(optarg);
                if (str == "4k") {
                    cfg.vSize = UHDSize;
                    cfg.testVideo = true;
                } else if (str == "1080p") {
                    cfg.vSize = FHDSize;
                    cfg.testVideo = true;
                } else if (str == "720p") {
                    cfg.vSize = HDSize;
                    cfg.testVideo = true;
                } else if (str == "VGA") {
                    cfg.vSize = VGASize;
                    cfg.testVideo = true;
                } else if (str == "QVGA") {
                    cfg.vSize = QVGASize;
                    cfg.testVideo = true;
                } else if (str == "stereoVGA") {
                    cfg.vSize = stereoVGASize;
                    cfg.testVideo = true;
                } else if (str == "stereoQVGA"){
                    cfg.vSize = stereoQVGASize;
                    cfg.testVideo = true;
                } else if (str == "disable"){
                    cfg.testVideo = false;
                }
                break;
            }
        case 'n':
            cfg.testSnapshot = true;
            cfg.picSizeIdx = 0;
            break;
       case 's':
            {
                string str(optarg);
                if (str == "4k") {
                    cfg.picSize = UHDSize;
                } else if (str == "1080p") {
                    cfg.picSize = FHDSize;
                } else if (str == "720p") {
                    cfg.picSize = HDSize;
                } else if (str == "VGA") {
                    cfg.picSize = VGASize;
                } else if (str == "QVGA") {
                    cfg.picSize = QVGASize;
                } else if (str == "stereoVGA") {
                    cfg.picSize = stereoVGASize;
                } else if (str == "stereoQVGA") {
                    cfg.picSize = stereoQVGASize;
                }
                cfg.testSnapshot = true;
                cfg.picSizeIdx = -1;
                break;
            }
        case 'd':
            cfg.dumpFrames = true;
            break;
        case 'i':
            cfg.infoMode = true;
            break;
        case  'e':
            cfg.exposureValue =  atoi(optarg);
            if (cfg.exposureValue < MIN_EXPOSURE_VALUE || cfg.exposureValue > MAX_EXPOSURE_VALUE) {
                printf("Invalid exposure value. Using default\n");
                cfg.exposureValue = DEFAULT_EXPOSURE_VALUE;
            }
            break;
        case  'g':
            cfg.gainValue =  atoi(optarg);
            if (cfg.gainValue < MIN_GAIN_VALUE || cfg.gainValue > MAX_GAIN_VALUE) {
                printf("Invalid exposure value. Using default\n");
                cfg.gainValue = DEFAULT_GAIN_VALUE;
            }
            break;
        case 'r':
            cfg.fps = atoi(optarg);
            if (!(cfg.fps == 30 || cfg.fps == 60 || cfg.fps == 90 || cfg.fps == 120)) {
                cfg.fps = DEFAULT_CAMERA_FPS;
                printf("Invalid fps values. Using default = %d ", cfg.fps);
            }
            break;
        case 'o':
            outputFormat = atoi(optarg);
            switch (outputFormat) {
            case 0: /* IMX135 , IMX214 */
                cfg.outputFormat = YUV_FORMAT;
                break;
            case 1: /* IMX214 */
                cfg.outputFormat = RAW_FORMAT;
                cfg.testVideo = false;
                break;
            default:
                printf("Invalid format. Setting to default YUV_FORMAT");
                cfg.outputFormat = YUV_FORMAT;
                break;
            }
            break;
          case 'j': {
              string str(optarg);
              if (str == "jpeg") {
                  cfg.snapshotFormat = JPEG_FORMAT;
              } else if (str == "raw") {
                  cfg.snapshotFormat = RAW_FORMAT;
              } else {
                  printf("invalid snapshot format \"%s\", using default\n",
                         optarg);
              }
              break;
          }
        case 'V':
            cfg.logLevel = (AppLoglevel)atoi(optarg);
            break;
        case 'f':
            break;
        case 'h':
        case '?':
            printUsageExit(0);
        default:
            abort();
        }
    }

    if (cfg.snapshotFormat == RAW_FORMAT) {
        cfg.testVideo = false;
    }

    return cfg;
}

int CameraTest::imagePublisher(ICameraFrame* frame)
{
    int frame_height = pSize_.height;
    int frame_width = pSize_.width;

    cv::Mat matFrame;
    std::string image_encoding;

    if (camera_used) { //highres
        cv::Mat mYUV = cv::Mat(1.5*frame_height, frame_width, CV_8UC1, frame->data);
        cv::cvtColor( mYUV, matFrame, CV_YUV420sp2RGB );
        image_encoding = "rgb8";
    } else { //optical flow
        matFrame = cv::Mat(frame_height, frame_width, CV_8UC1, frame->data);
        image_encoding = "mono8";
    }

    // convert OpenCV image to ROS message
    //printf("time = %llu\n", frame->timeStamp);
    cv_bridge::CvImage cvImage;
    ros::Time image_timestamp(frame->timeStamp / 1000000000.0f); //from nano seconds to seconds
    cvImage.header.stamp = image_timestamp; //needs to be ros:Time
    cvImage.header.frame_id = "snap_cam_image";
    cvImage.encoding = image_encoding;
    cvImage.image = matFrame;

    sensor_msgs::Image msg;
    cvImage.toImageMsg(msg);

    if (nh.ok()) {
        pub.publish(msg);
        return 1;
    }
}
