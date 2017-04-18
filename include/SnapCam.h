/* Copyright (c) 2016, PX4 Development Team. All rights reserved.
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
 *     * Neither the name of PX4 nor the names of its
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
 */

#ifndef _SNAP_CAM_
#define _SNAP_CAM_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>

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

// workaround to only have to define these once
struct CameraSizes {
	static ImageSize FourKSize()        {static ImageSize is(4096, 2160); return is;};
	static ImageSize UHDSize()          {static ImageSize is(3840, 2160); return is;};
	static ImageSize FHDSize()          {static ImageSize is(1920, 1080); return is;};
	static ImageSize HDSize()           {static ImageSize is(1280, 720); return is;};
	static ImageSize VGASize()          {static ImageSize is(640, 480); return is;};
	static ImageSize stereoVGASize()    {static ImageSize is(1280, 480); return is;};
	static ImageSize QVGASize()         {static ImageSize is(320, 240); return is;};
	static ImageSize stereoQVGASize()   {static ImageSize is(640, 240); return is;};
};

struct CameraCaps {
	vector<ImageSize> pSizes, vSizes, picSizes;
	vector<string> focusModes, wbModes, isoModes;
	Range brightness, sharpness, contrast;
	vector<Range> previewFpsRanges;
	vector<VideoFPS> videoFpsValues;
	vector<string> previewFormats;
	string rawSize;
};

enum OutputFormatType {
	YUV_FORMAT,
	RAW_FORMAT,
	JPEG_FORMAT
};

enum CamFunction {
	CAM_FUNC_UNKNOWN = -1,
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
struct CamConfig {
	int32_t cameraId;
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

// Callback function.
typedef std::function<void(const cv::Mat &, uint64_t time_stamp)> CallbackFunction;

/**
 * CLASS  SnapCam
 *
 * inherits ICameraListers which provides core functionality
 */

class SnapCam : ICameraListener
{
public:

	SnapCam(CamConfig cfg);
	SnapCam(int argc, char *argv[]);
	SnapCam(std::string config_str);
	~SnapCam();

	void setListener(CallbackFunction fun);  // register a function callback
	template <class T>
	void setListener(CallbackFunction fun, T *obj);  //register a function callback that is a class member

	/* listener methods */
	virtual void onError();
	virtual void onPreviewFrame(ICameraFrame *frame);

private:
	int initialize(CamConfig cfg);

	ICameraDevice *camera_;
	CameraParams params_;
	ImageSize pSize_, vSize_, picSize_;
	CameraCaps caps_;
	CamConfig config_;

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

	int printCapabilities();
	int setParameters();
	int setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx);
	int findCamera(CamConfig cfg, int32_t &camera_id);

	CallbackFunction cb_;
};

#endif // _SNAP_CAM_
