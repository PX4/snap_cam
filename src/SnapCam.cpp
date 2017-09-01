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
	: cb_(nullptr),
	auto_exposure_(false),
	set_crop_(false)
{
	initialize(cfg);
}

SnapCam::SnapCam(int argc, char *argv[])
	: cb_(nullptr),
	auto_exposure_(false),
	set_crop_(false),
	msv_error_int_(0.0f)
{
	initialize(parseCommandline(argc, argv));
}

SnapCam::SnapCam(std::string config_str)
	: cb_(nullptr),
	auto_exposure_(false),
	set_crop_(false)
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
	int vFpsIdx;

	pSize_ = cfg.pSize;


	frameCounter = 0;

	//set parameters
	int focusModeIdx = 0; //fixed
	int wbModeIdx =
		0; //whitebalance 0: auto 1: incandescent 2: fluorescent 3: warm-fluorescent 4: daylight 5: cloudy-daylight 6: twilight 7: shade 8: manual-cct
	int isoModeIdx = 0; //auto

	rc = setFPSindex(cfg.fps, pFpsIdx, vFpsIdx);

	if ( rc == -1)
	{
		printf("FPS indexing failed pFpsIdx: %d  vFpsIdx: %d\n", pFpsIdx, vFpsIdx);
		return rc;
	}

	params_.setVideoFPS(caps_.videoFpsValues[vFpsIdx]);
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
		return rc;
	}

	camera_->startPreview();

	params_.setManualExposure(cfg.exposureValue);
	params_.setManualGain(cfg.gainValue);
	printf("Setting exposure value =  %d , gain value = %d \n", cfg.exposureValue, cfg.gainValue );

	rc = params_.commit();

	if (rc) {
		printf("Commit failed\n");
		printCapabilities();

	} else {
		camera_->startRecording();
	}

	config_ = cfg;
}

SnapCam::~SnapCam()
{
	camera_->stopPreview();
	camera_->stopRecording();

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
void SnapCam::onVideoFrame(ICameraFrame *frame)
{
	uint64_t time_stamp = get_absolute_time();

	if (!cb_) {
		return;        // as long as nobody is listening, we don't need to do anything
	}

	int frame_height = pSize_.height;
	int frame_width = pSize_.width;

	cv::Mat matFrame;

	if (config_.func == 0) { //highres
		cv::Mat mYUV = cv::Mat(1.5 * frame_height, frame_width, CV_8UC1, frame->data);
		cv::cvtColor(mYUV, matFrame, CV_YUV420sp2RGB);
		mYUV.release();

	} else { //optical flow
		matFrame = cv::Mat(frame_height, frame_width, CV_8UC1, frame->data);
	}

	if (set_crop_) {
		static cv::Rect crop(frame_width/2-crop_width_/2, frame_height/2-crop_height_/2, crop_width_, crop_height_);
		cv::Mat croppedImage = matFrame(crop);
		// Copy the data into new matrix -> croppedImage.data can not be used in calcFlow()...
		croppedImage.copyTo(matFrame);
		croppedImage.release();
	}

	if (auto_exposure_) {
		updateExposureAndGain(matFrame);
	}

	cb_(matFrame, time_stamp);
	matFrame.release();
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
		if (fps == 30 * caps_.videoFpsValues[i]) {
			vFpsIdx = i;
			break;
		}

		if (DEFAULT_CAMERA_FPS == 30 * caps_.videoFpsValues[i]) {
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

void SnapCam::updateExposureAndGain(cv::Mat &frame)
{
	//limit update rate to 5Hz
	static int counter = 1;
	const int devider = std::round(config_.fps*0.2);
	if (counter%devider != 0) {
		counter++;
		return;
	}
	counter = 1;

	//init histogram variables
	cv::Mat hist;
	int channels[] = {0};
	int histSize[] = {10}; //10 bins
	float range[] = { 0, 255 };
	const float* ranges[] = { range };

	// only use 128x128 window to calculate exposure
	cv::Mat mask(frame.rows,frame.cols,CV_8U,cv::Scalar(0));
	if (frame.cols > HISTOGRAM_MASK_SIZE && frame.rows > HISTOGRAM_MASK_SIZE) {
		mask(cv::Rect(frame.cols/2-HISTOGRAM_MASK_SIZE/2, frame.rows/2-HISTOGRAM_MASK_SIZE/2,
			HISTOGRAM_MASK_SIZE, HISTOGRAM_MASK_SIZE)) = 255;
	} else {
		mask = 255;
	}

	//calculate the histogram with 10 bins
	calcHist( &frame, 1, channels, mask,
	     hist, 1, histSize, ranges,
	     true, // the histogram is uniform
	     false );

	//calculate Mean Sample Value (MSV)
	float msv = 0.0f;
	for (int i = 0; i < histSize[0]; i++) {
		msv += (i+1)*hist.at<float>(i)/16384.0f; //128x128 -> 16384
	}

	//get first exposure and gain value
	static float exposure_old = config_.exposureValue; // config_.exposureValue does not change -> use exposure_old
	static float gain_old = config_.gainValue; // config_.gainValue does not change -> use gain_old
	float exposure = exposure_old;
	float gain = gain_old;

	//calculate MSV error, derivative and integral
	float msv_error = MSV_TARGET - msv;
	msv_error_old_ = msv_error;
	msv_error_int_ += msv_error;
	float msv_error_d = msv_error - msv_error_old_;

	//calculate new exposure value based on MSV
	exposure += EXPOSURE_P*msv_error + EXPOSURE_I*msv_error_int_ + EXPOSURE_D*msv_error_d;

	//adjust the gain if exposure is saturated
	if (gain > MIN_GAIN_VALUE || (exposure > MAX_EXPOSURE_VALUE-1.0f && exposure_old > MAX_EXPOSURE_VALUE-1.0f)) {

		//calculate new gain value based on MSV
		gain += GAIN_P*msv_error + GAIN_I*msv_error_int_ + GAIN_D*msv_error_d;

		if (gain < MIN_GAIN_VALUE)
			gain = MIN_GAIN_VALUE;
		if (gain > MAX_GAIN_VALUE)
			gain = MAX_GAIN_VALUE;

		//set new gain value if bigger than threshold
		if (fabs(gain - gain_old) > GAIN_CHANGE_THRESHOLD || (gain > MAX_GAIN_VALUE-1.0f && gain_old < MAX_GAIN_VALUE) ||
		   (gain < MIN_GAIN_VALUE+1.0f && gain_old > MIN_GAIN_VALUE)) {
			params_.setManualGain(std::round(gain));
			params_.commit();
			gain_old = gain;
		}

	} else { //adjust exposure

		if (exposure < MIN_EXPOSURE_VALUE)
			exposure = MIN_EXPOSURE_VALUE;
		if (exposure > MAX_EXPOSURE_VALUE)
			exposure = MAX_EXPOSURE_VALUE;

		//set new exposure value if bigger than threshold or exposure old is not yet bigger than MAX_EXPOSURE_VALUE
		if (fabs(exposure - exposure_old) > EXPOSURE_CHANGE_THRESHOLD ||
		   (exposure > MAX_EXPOSURE_VALUE-1.0f && exposure_old < MAX_EXPOSURE_VALUE)) {
			params_.setManualExposure(std::round(exposure));
			params_.commit();
			exposure_old = exposure;
		}

	}

	msv_error_old_ = msv_error;

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
