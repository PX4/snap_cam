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

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>

#include "turbojpeg.h"

#include "camera.h"
#include <syslog.h>
#include "camera_parameters.h"

#define DEFAULT_EXPOSURE_VALUE  250
#define MIN_EXPOSURE_VALUE 0
#define MAX_EXPOSURE_VALUE 65535
#define DEFAULT_GAIN_VALUE  50
#define MIN_GAIN_VALUE 0
#define MAX_GAIN_VALUE 255

#define DEFAULT_CAMERA_FPS 90
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
    CameraTest(TestConfig config);
    ~CameraTest();
    int run();

    int initialize(int camId);

    /* listener methods */
    virtual void onError();
    virtual void onPreviewFrame(ICameraFrame* frame);
    virtual void onVideoFrame(ICameraFrame* frame);
    virtual void onPictureFrame(ICameraFrame* frame);

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

    int printCapabilities();
    int setParameters();
    int compressJpegAndSave(ICameraFrame *frame, char *name);
    int takePicture();
    int setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx);
};

CameraTest::CameraTest() :
    vFrameCount_(0),
    pFrameCount_(0),
    vFpsAvg_(0.0f),
    pFpsAvg_(0.0f),
    vTimeStampPrev_(0),
    pTimeStampPrev_(0),
    camera_(NULL)
{
    pthread_cond_init(&cvPicDone, NULL);
    pthread_mutex_init(&mutexPicDone, NULL);
}

CameraTest::CameraTest(TestConfig config) :
    vFrameCount_(0),
    pFrameCount_(0),
    vFpsAvg_(0.0f),
    pFpsAvg_(0.0f),
    vTimeStampPrev_(0),
    pTimeStampPrev_(0)
{
    config_ = config;
    pthread_cond_init(&cvPicDone, NULL);
    pthread_mutex_init(&mutexPicDone, NULL);
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

    static uint64_t last_ts = 0;

    float fps = 1e9/(timestamp - last_ts);
    last_ts = timestamp;

    printf("saved filename %s (%f fps)\n", name, fps);
    fclose(fp);
}

static inline uint32_t align_size(uint32_t size, uint32_t align)
{
    return ((size + align - 1) & ~(align-1));
}

int CameraTest::takePicture()
{
    int rc;
    pthread_mutex_lock(&mutexPicDone);
    isPicDone = false;
    printf("take picture\n");
    rc = camera_->takePicture();
    if (rc) {
        printf("takePicture failed\n");
        pthread_mutex_unlock(&mutexPicDone);
        return rc;
    }

    struct timespec waitTime;
    struct timeval now;

    gettimeofday(&now, NULL);
    waitTime.tv_sec = now.tv_sec + TAKEPICTURE_TIMEOUT_MS/MS_PER_SEC;
    waitTime.tv_nsec = now.tv_usec * NS_PER_US + (TAKEPICTURE_TIMEOUT_MS % MS_PER_SEC) * NS_PER_MS;
    /* wait for picture done */
    while (isPicDone == false) {
        rc = pthread_cond_timedwait(&cvPicDone, &mutexPicDone, &waitTime);
        if (rc == ETIMEDOUT) {
            printf("error: takePicture timed out\n");
            break;
        }
    }
    pthread_mutex_unlock(&mutexPicDone);
    return 0;
}

int CameraTest::compressJpegAndSave(ICameraFrame *frame, char* name)
{
    uint32_t jpegSize = 0;
    int jpegQuality = 90;
    uint8_t *dest = NULL;
    int rc = 0;

    tjhandle jpegCompressor = tjInitCompress();
    if (jpegCompressor == NULL) {
        printf("Could not open TurboJpeg compressor\n");
        return -1;
    }

    uint32_t w = picSize_.width;
    uint32_t h = picSize_.height;
    uint32_t stride = align_size(w, SNAPSHOT_WIDTH_ALIGN);
    uint32_t scanlines = align_size(h, SNAPSHOT_HEIGHT_ALIGN);

    uint32_t cbSize =  tjPlaneSizeYUV(1, w, 0, h, TJSAMP_420);
    uint32_t crSize =  tjPlaneSizeYUV(2, w, 0, h, TJSAMP_420);
    printf("st=%d, sc=%d\n", stride, scanlines);

    uint8_t *yPlane, *cbPlane, *crPlane;
    uint8_t* planes[3];
    int strides[3];

    yPlane = frame->data;
    cbPlane = (uint8_t*) malloc(cbSize);
    crPlane = (uint8_t*) malloc(crSize);

    uint8_t *pCbcrSrc;
    uint8_t *pCbDest = cbPlane;
    uint8_t *pCrDest = crPlane;

    uint32_t line=0;

    /* copy interleaved CbCr data to separate Cb and Cr planes */
    for (line=0; line < h/2; line++) {
        pCbcrSrc = frame->data + (stride * scanlines) + line*stride;
        uint32_t count = w/2;
        while (count != 0) {
            *pCrDest = *(pCbcrSrc++);
            *pCbDest = *(pCbcrSrc++);
            pCrDest++;
            pCbDest++;
            count--;
        }
    }
    planes[0] = yPlane;
    planes[1] = cbPlane;
    planes[2] = crPlane;
    strides[0] = stride;
    strides[1] = 0;
    strides[2] = 0;

    tjCompressFromYUVPlanes(jpegCompressor, planes, w, strides, h, TJSAMP_420,
                            &dest, (long unsigned int *)&jpegSize, jpegQuality,
                            TJFLAG_FASTDCT);

    /* save the file to disk */
    printf("saving JPEG image: %s\n", name);
    FILE *fp = fopen(name, "w");
    if (fp == NULL) {
        printf("fopen() failed\n");
        rc = -1;
        goto exit1;
    }
    fwrite(dest, jpegSize, 1, fp);
    fclose(fp);

exit1:
    tjFree(dest);
    tjDestroy(jpegCompressor);
    free(cbPlane);
    free(crPlane);
    return 0;
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
    if (pFrameCount_ > 0) {
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
            dumpToFile(frame->data, pSize_.width*pSize_.height, name, frame->timeStamp);
        }
        //printf("Preview FPS = %.2f\n", pFpsAvg_);
    }

    uint64_t diff = frame->timeStamp - pTimeStampPrev_;
    pFpsAvg_ = ((pFpsAvg_ * pFrameCount_) + (1e9 / diff)) / (pFrameCount_ + 1);
    pFrameCount_++;
    pTimeStampPrev_  = frame->timeStamp;
}

void CameraTest::onPictureFrame(ICameraFrame* frame)
{
    char yuvName[128], jpgName[128];
    char rawName[128];
    if (config_.snapshotFormat == RAW_FORMAT) {
        snprintf(rawName, 128, "snapshot_%s_mipi.raw", caps_.rawSize.c_str());
        dumpToFile(frame->data, frame->size, rawName, frame->timeStamp);
    } else {
        snprintf(jpgName, 128, "snapshot_%dx%d.jpg", picSize_.width, picSize_.height);
       dumpToFile(frame->data, frame->size, jpgName, frame->timeStamp);
    }
    /* notify the waiting thread about picture done */
    pthread_mutex_lock(&mutexPicDone);
    isPicDone = true;
    pthread_cond_signal(&cvPicDone);
    pthread_mutex_unlock(&mutexPicDone);
    printf("%s:%d\n", __func__, __LINE__);
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
    if (vFrameCount_ > 0) {
        char name[50];
        snprintf(name, 50, "V_%dx%d_%04d_%llu.yuv",
                 vSize_.width, vSize_.height, vFrameCount_,frame->timeStamp);
        if (config_.dumpFrames == true) {
            dumpToFile(frame->data, frame->size, name, frame->timeStamp);
        }
        //printf("Video FPS = %.2f\n", vFpsAvg_);
    }

    uint64_t diff = frame->timeStamp - vTimeStampPrev_;
    vFpsAvg_ = ((vFpsAvg_ * vFrameCount_) + (1e9 / diff)) / (vFrameCount_ + 1);
    vFrameCount_++;
    vTimeStampPrev_  = frame->timeStamp;
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
ImageSize UHDSize(3840,2160);
ImageSize FHDSize(1920,1080);
ImageSize HDSize(1280,720);
ImageSize VGASize(640,480);
ImageSize stereoVGASize(1280, 480);
ImageSize QVGASize(320,240);
ImageSize stereoQVGASize(640,240);

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
    int focusModeIdx = 3;
    int wbModeIdx = 2;
    int isoModeIdx = 0;
    int pFpsIdx = 3;
    int vFpsIdx = 3;
    int prevFmtIdx = 0;
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

int CameraTest::run()
{
    int rc = EXIT_SUCCESS;

    /* returns the number of camera-modules connected on the board */
    int n = getNumberOfCameras();

    if (n < 0) {
        printf("getNumberOfCameras() failed, rc=%d\n", n);
        return EXIT_FAILURE;
    }

    printf("num_cameras = %d\n", n);

    if (n < 1) {
        printf("No cameras found.\n");
        return EXIT_FAILURE;
    }

    /* The camID for sensor is not fixed. It depends on which drivers comes up first.
       Hence loop throuch all modules to find camID based on the functionality. */
    int camId=-1;

    /* find camera based on function */
    for (int i=0; i<n; i++) {
        CameraInfo info;
        getCameraInfo(i, info);
        printf(" i = %d , info.func = %d \n",i, info.func);
        if (info.func == config_.func) {
            camId = i;
        }
    }

    if (camId == -1 )
    {
        printf("Camera not found \n");
        exit(1);
    }

    printf("Testing camera id=%d\n", camId);

    initialize(camId);

    if (config_.infoMode) {
        printCapabilities();
        return rc;
    }

    rc = setParameters();
    if (rc) {
        printf("setParameters failed\n");
        printUsageExit(0);
        goto del_camera;
    }

    /* initialize perf counters */
    vFrameCount_ = 0;
    pFrameCount_ = 0;
    vFpsAvg_ = 0.0f;
    pFpsAvg_ = 0.0f;

    /* starts the preview stream. At every preview frame onPreviewFrame( ) callback is invoked */
    printf("start preview\n");
    camera_->startPreview();

    /* Set parameters which are required after starting preview */
    switch(config_.func)
    {
        case CAM_FUNC_OPTIC_FLOW:
            {
                 params_.setManualExposure(config_.exposureValue);
                 params_.setManualGain(config_.gainValue);
                 printf("Setting exposure value =  %d , gain value = %d \n", config_.exposureValue, config_.gainValue );
            }
            break;
        case CAM_FUNC_RIGHT_SENSOR:
            {
                 params_.setManualExposure(config_.exposureValue);
                 params_.setManualGain(config_.gainValue);
                 printf("Setting exposure value =  %d , gain value = %d \n", config_.exposureValue, config_.gainValue );
            }
            break;
        case CAM_FUNC_STEREO:
            {
                params_.setManualExposure(config_.exposureValue);
                params_.setManualGain(config_.gainValue);
                printf("Setting exposure value =  %d , gain value = %d \n", config_.exposureValue, config_.gainValue );
                params_.setVerticalFlip(true);
                params_.setHorizontalMirror(true);
                printf("Setting Vertical Flip and Horizontal Mirror bit in sensor \n");
            }
            break;

    }
    rc = params_.commit();
    if (rc) {
        printf("commit failed\n");
        exit(EXIT_FAILURE);
    }

    if (config_.testVideo  == true ) {
        /* starts video stream. At every video frame onVideoFrame( )  callback is invoked */
        printf("start recording\n");
        camera_->startRecording();
    }

    if (config_.testSnapshot == true) {
        printf("waiting for 2 seconds for exposure to settle...\n");
        /* sleep required to settle the exposure before taking snapshot.
           This app does not provide interactive feedback to user
           about the exposure */
        sleep(2);
        printf("taking picture\n");
        rc = takePicture();
        if (rc) {
            printf("takePicture failed\n");
            exit(EXIT_FAILURE);
        }
    }

    /* Put the main/run thread to sleep and process the frames in the callbacks */
    printf("waiting for %d seconds ...\n", config_.runTime);
    sleep(config_.runTime);

    /* After the sleep interval stop preview stream, stop video stream and end application */
    if (config_.testVideo  == true) {
        printf("stop recording\n");
        camera_->stopRecording();
    }
    printf("stop preview\n");
    camera_->stopPreview();

    printf("Average preview FPS = %.2f\n", pFpsAvg_);
    if( config_.testVideo  == true )
		printf("Average video FPS = %.2f\n", vFpsAvg_);

del_camera:
    /* release camera device */
    ICameraDevice::deleteInstance(&camera_);
    return rc;
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
    cfg.func = CAM_FUNC_OPTIC_FLOW;

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

int main(int argc, char* argv[])
{

    TestConfig config = parseCommandline(argc, argv);

    /* setup syslog level */
    if (config.logLevel == CAM_LOG_SILENT) {
        setlogmask(LOG_UPTO(LOG_EMERG));
    } else if (config.logLevel == CAM_LOG_DEBUG) {
        setlogmask(LOG_UPTO(LOG_DEBUG));
    } else if (config.logLevel == CAM_LOG_INFO) {
        setlogmask(LOG_UPTO(LOG_INFO));
    } else if (config.logLevel == CAM_LOG_ERROR) {
        setlogmask(LOG_UPTO(LOG_ERR));
    }
    openlog(NULL, LOG_NDELAY, LOG_DAEMON);

    CameraTest test(config);
    test.run();

    return EXIT_SUCCESS;
}
