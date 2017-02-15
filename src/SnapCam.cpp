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

/*
 * SnapCam.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Christoph, Nicolas
 */

#include "SnapCam.h"

#include <sstream>
#include <vector>

using namespace std;
using namespace camera;

static CamConfig parseCommandline(int argc, char *argv[]);
static uint64_t get_absolute_time();

SnapCam::SnapCam(CamConfig cfg)
	: cb_(nullptr)
{
	initialize(cfg);
}

SnapCam::SnapCam(int argc, char *argv[])
	: cb_(nullptr)
{
	initialize(parseCommandline(argc, argv));
}

SnapCam::SnapCam(std::string config_str)
	: cb_(nullptr)
{
	std::vector<char *> args;
	std::istringstream iss(config_str);

	std::string token;

	while (iss >> token) {
		char *arg = new char[token.size() + 1];
		copy(token.begin(), token.end(), arg);
		arg[token.size()] = '\0';
		args.push_back(arg);
	}

	args.push_back(0);

	initialize(parseCommandline(args.size(), &args[0]));
}

int SnapCam::findCamera(CamConfig cfg, int32_t &camera_id)
{
	int num_cams = camera::getNumberOfCameras();

	if (num_cams < 1) {
		printf("No cameras detected. Exiting.\n");
		return -1;
	}

	bool found = false;

	for (int i = 0; i < num_cams; ++i) {
		camera::CameraInfo info;
		getCameraInfo(i, info);
		if (info.func == static_cast<int>(cfg.func)) {
			camera_id = i;
			found = true;
		}
	}

	if (!found) {
		printf("Could not find camera of type %d. Exiting", cfg.func);
		return -1;
	}

	printf("Camera of type %d has ID = %d\n", cfg.func, camera_id);

	return 0;
}

int SnapCam::initialize(CamConfig cfg)
{
	int rc;
	int32_t cameraId;
	rc = SnapCam::findCamera(cfg, cameraId);

	if (rc != 0) {
		printf("Cannot find camera Id for type: %d", cfg.func);
		return rc;
	}
	cfg.cameraId = cameraId;

	rc = ICameraDevice::createInstance(cfg.cameraId, &camera_);

	if (rc != 0) {
		printf("Could not open camera %d\n", cfg.func);
		return rc;
	}

	camera_->addListener(this);

	rc = params_.init(camera_);

	if (rc != 0) {
		printf("failed to init parameters\n");
		ICameraDevice::deleteInstance(&camera_);
		return rc;
	}

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

	int pFpsIdx;

	if (cfg.fps >= 0 && cfg.fps <= 6) {
		pFpsIdx = cfg.fps;

	} else {
		pFpsIdx = 0;
	}

	pSize_ = cfg.pSize;


	frameCounter = 0;

	//set parameters
	int focusModeIdx = 0; //fixed
	int wbModeIdx =
		0; //whitebalance 0: auto 1: incandescent 2: fluorescent 3: warm-fluorescent 4: daylight 5: cloudy-daylight 6: twilight 7: shade 8: manual-cct
	int isoModeIdx = 0; //auto

	//params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);
	params_.setFocusMode(caps_.focusModes[focusModeIdx]);
	params_.setWhiteBalance(caps_.wbModes[wbModeIdx]);
	params_.setISO(caps_.isoModes[isoModeIdx]);
	params_.setVideoSize(pSize_);
	params_.setPreviewFormat(caps_.previewFormats[0]); //0:yuv420sp 1:yuv420p 2:nv12-venus 3:bayer-rggb
	params_.setPreviewSize(pSize_);
	params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);

	rc = params_.commit();

	if (rc) {
		printf("Commit failed\n");
		printCapabilities();

	} else {
		camera_->startPreview();
	}

	config_ = cfg;
}

SnapCam::~SnapCam()
{
	camera_->stopPreview();

	/* release camera device */
	ICameraDevice::deleteInstance(&camera_);
}

static inline uint32_t align_size(uint32_t size, uint32_t align)
{
	return ((size + align - 1) & ~(align - 1));
}

void SnapCam::onError()
{
	printf("camera error!, aborting\n");
	exit(EXIT_FAILURE);
}

/**
 *
 * FUNCTION: onPreviewFrame
 *
 *  - This is called every frame I
 *  - In parameter frame (ICameraFrame) also has the timestamps
 *    field which is public
 *
 * @param frame
 *
 */
void SnapCam::onPreviewFrame(ICameraFrame *frame)
{
	uint64_t time_stamp = get_absolute_time();

	if (!cb_) {
		return;        // as long as nobody is listening, we don't need to do anything
	}

	int frame_height = pSize_.height;
	int frame_width = pSize_.width;

	cv::Mat matFrame;

	if (config_.func) { //highres
		cv::Mat mYUV = cv::Mat(1.5 * frame_height, frame_width, CV_8UC1, frame->data);
		cv::cvtColor(mYUV, matFrame, CV_YUV420sp2RGB);

	} else { //optical flow
		matFrame = cv::Mat(frame_height, frame_width, CV_8UC1, frame->data);
	}

	cb_(matFrame, time_stamp);
    matFrame.deallocate();
}

int SnapCam::printCapabilities()
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
int SnapCam::setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx)
{
	int defaultPrevFPSIndex = -1;
	int defaultVideoFPSIndex = -1;
	int i, rc = 0;

	for (i = 0; i < caps_.previewFpsRanges.size(); i++) {
		if ((caps_.previewFpsRanges[i].max) / 1000 == fps) {
			pFpsIdx = i;
			break;
		}

		if ((caps_.previewFpsRanges[i].max) / 1000 == DEFAULT_CAMERA_FPS) {
			defaultPrevFPSIndex = i;
		}
	}

	if (i >= caps_.previewFpsRanges.size()) {
		if (defaultPrevFPSIndex != -1) {
			pFpsIdx = defaultPrevFPSIndex;

		} else {
			pFpsIdx = -1;
			rc = -1;
		}
	}

	for (i = 0; i < caps_.videoFpsValues.size(); i++) {
		if (fps == caps_.videoFpsValues[i]) {
			vFpsIdx = i;
			break;
		}

		if (DEFAULT_CAMERA_FPS == caps_.videoFpsValues[i]) {
			defaultVideoFPSIndex = i;
		}
	}

	if (i >= caps_.videoFpsValues.size()) {
		if (defaultVideoFPSIndex != -1) {
			vFpsIdx = defaultVideoFPSIndex;

		} else {
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
int SnapCam::setParameters()
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

	switch (config_.func) {
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
		if (config_.picSizeIdx != -1) {
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

	if (rc == -1) {
		return rc;
	}

	printf("setting preview fps range: %d, %d ( idx = %d ) \n",
	       caps_.previewFpsRanges[pFpsIdx].min,
	       caps_.previewFpsRanges[pFpsIdx].max, pFpsIdx);
	params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);
	printf("setting video fps: %d ( idx = %d )\n", caps_.videoFpsValues[vFpsIdx], vFpsIdx);
	params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);


	return params_.commit();
}

/**
 *  FUNCTION: setListener
 *
 *  Set a listener for image callbacks
 *  The callback is provided with a cv::Mat image
 *
 * */
void SnapCam::setListener(CallbackFunction fun)
{
	cb_ = fun;
}

/**
 *  FUNCTION: setListener
 *
 *  Set a class member listener for image callbacks
 *  The callback is provided with a cv::Mat image
 *
 * */
template <class T>
void SnapCam::setListener(CallbackFunction fun, T *obj)
{
	cb_ = std::bind(fun, obj);
}

/**
 *  FUNCTION: setDefaultConfig
 *
 *  set default config based on camera module
 *
 * */
static int setDefaultConfig(CamConfig &cfg)
{

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
		cfg.pSize   = CameraSizes::VGASize();
		cfg.vSize   = CameraSizes::VGASize();
		cfg.picSize   = CameraSizes::VGASize();
		cfg.outputFormat = RAW_FORMAT;
		break;

	case CAM_FUNC_RIGHT_SENSOR:
		cfg.pSize   = CameraSizes::VGASize();
		cfg.vSize   = CameraSizes::VGASize();
		cfg.picSize   = CameraSizes::VGASize();
		break;

	case CAM_FUNC_STEREO:
		cfg.pSize = CameraSizes::stereoVGASize();
		cfg.vSize  = CameraSizes::stereoVGASize();
		cfg.picSize  = CameraSizes::stereoVGASize();
		break;

	case CAM_FUNC_HIRES:
		cfg.pSize = CameraSizes::FHDSize();
		cfg.vSize = CameraSizes::HDSize();
		cfg.picSize = CameraSizes::FHDSize();
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
static CamConfig parseCommandline(int argc, char *argv[])
{
	CamConfig cfg;
	cfg.func = CAM_FUNC_HIRES;

	int c;
	int outputFormat;
	int exposureValueInt = 0;
	int gainValueInt = 0;

	while ((c = getopt(argc, argv, "hdt:io:e:g:p:v:ns:f:r:V:j:")) != -1) {
		switch (c) {
		case 'f': {
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

		case 'p': {
				string str(optarg);

				if (str == "4k") {
					cfg.pSize = CameraSizes::UHDSize();

				} else if (str == "1080p") {
					cfg.pSize = CameraSizes::FHDSize();

				} else if (str == "720p") {
					cfg.pSize = CameraSizes::HDSize();

				} else if (str == "VGA") {
					cfg.pSize = CameraSizes::VGASize();

				} else if (str == "QVGA") {
					cfg.pSize = CameraSizes::QVGASize();

				} else if (str == "stereoVGA") {
					cfg.pSize = CameraSizes::stereoVGASize();

				} else if (str == "stereoQVGA") {
					cfg.pSize = CameraSizes::stereoQVGASize();
				}

				break;
			}

		case 'v': {
				string str(optarg);

				if (str == "4k") {
					cfg.vSize = CameraSizes::UHDSize();
					cfg.testVideo = true;

				} else if (str == "1080p") {
					cfg.vSize = CameraSizes::FHDSize();
					cfg.testVideo = true;

				} else if (str == "720p") {
					cfg.vSize = CameraSizes::HDSize();
					cfg.testVideo = true;

				} else if (str == "VGA") {
					cfg.vSize = CameraSizes::VGASize();
					cfg.testVideo = true;

				} else if (str == "QVGA") {
					cfg.vSize = CameraSizes::QVGASize();
					cfg.testVideo = true;

				} else if (str == "stereoVGA") {
					cfg.vSize = CameraSizes::stereoVGASize();
					cfg.testVideo = true;

				} else if (str == "stereoQVGA") {
					cfg.vSize = CameraSizes::stereoQVGASize();
					cfg.testVideo = true;

				} else if (str == "disable") {
					cfg.testVideo = false;
				}

				break;
			}

		case 'n':
			cfg.testSnapshot = true;
			cfg.picSizeIdx = 0;
			break;

		case 's': {
				string str(optarg);

				if (str == "4k") {
					cfg.picSize = CameraSizes::UHDSize();

				} else if (str == "1080p") {
					cfg.picSize = CameraSizes::FHDSize();

				} else if (str == "720p") {
					cfg.picSize = CameraSizes::HDSize();

				} else if (str == "VGA") {
					cfg.picSize = CameraSizes::VGASize();

				} else if (str == "QVGA") {
					cfg.picSize = CameraSizes::QVGASize();

				} else if (str == "stereoVGA") {
					cfg.picSize = CameraSizes::stereoVGASize();

				} else if (str == "stereoQVGA") {
					cfg.picSize = CameraSizes::stereoQVGASize();
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
			printf("Invalid arguments\n");

		default:
			abort();
		}
	}

	if (cfg.snapshotFormat == RAW_FORMAT) {
		cfg.testVideo = false;
	}

	return cfg;
}

static uint64_t get_absolute_time()
{
	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_MONOTONIC, &time);
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;
	return micros;
}
