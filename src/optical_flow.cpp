/*
* optical_flow.cpp
*
*  Created on: Mar 16, 2016
*      Author: Christoph, Nicolas
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

#ifdef CATKIN_BUILD
#include <ros/package.h>
#endif

#include "flow_opencv.hpp"
#include "calib_yaml_interface.h"

#include <mavlink/v1.0/mavlink_types.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <fcntl.h>
#include <arpa/inet.h>

#include "ringbuffer.h"

#include "logging.h"
#include "SnapCam.h"

#define UDP_LOCAL_PORT_DEFAULT  14558
#define UDP_REMOTE_PORT_DEFAULT 14557
#define UDP_REMOTE_SEND_PORT_DEFAULT 14556
#define BUFFER_LENGTH 2041 // minimum buffer size that can be used with qnx (I don't know why)
#define OPTICAL_FLOW_OUTPUT_RATE 15
#define IMAGE_CROP_WIDTH 128
#define IMAGE_CROP_HEIGHT 128
struct sockaddr_in _srcaddr;

// command line parameters
struct Params {
	std::string res;
	std::string calibration_path;
	uint32_t num_features;
	uint32_t fps;
	std::string target_ip;
	uint16_t udp_local_port;
	uint16_t udp_remote_port;
	uint8_t gain;
	uint16_t exposure;
	bool auto_exposure;
} cl_params;

struct GyroTimestamped {
	float xgyro;
	float ygyro;
	float zgyro;
	uint64_t time_usec;
};

ringbuffer::RingBuffer rb_imu(1000, sizeof(GyroTimestamped));
ringbuffer::RingBuffer rb_flow(10000, sizeof(mavlink_optical_flow_rad_t));

// declare helper functions
uint64_t get_absolute_time();
void loadCustomCameraCalibration(const std::string calib_path);
void send_mavlink_message(const uint8_t msgid, const void *msg, uint8_t component_ID);
void sendOptFlowMessage(uint64_t timestamp, uint32_t dt, double flow_x, double flow_y, double quality);
void calcOptFlow(const cv::Mat &Image, uint64_t img_timestamp);
void imageCallback(const cv::Mat &img, uint64_t time_stamp);
int parseCommandline(int argc, char *argv[], Params &cl_params);
void handle_message(mavlink_message_t *msg);
void handle_message_highres_imu(mavlink_message_t *msg);
bool read_mavlink_messages(int &sock, struct sockaddr_in &theirAddr);
int calc_imu_time_offset();

//decalare variables
static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

mavlink_highres_imu_t last_highres_imu;

CameraParameters cameraParams = {};

int64_t imu_time_offset_usecs = 0;
OpticalFlowOpenCV *_optical_flow;

int image_width;
int image_height;

int sock;
struct sockaddr_in myAddr;
struct sockaddr_in theirAddr;
struct sockaddr_in
	theirAddrSend; // for some reason we can't send to the port we're receiving from, need to sent to default port

uint8_t buf[BUFFER_LENGTH];
int main(int argc, char **argv)
{
	// set default values
	cl_params.res = "VGA";
	cl_params.calibration_path = "";
	cl_params.num_features = 10;
	cl_params.fps = 30;
	cl_params.target_ip = "127.0.0.1";
	cl_params.udp_local_port = UDP_LOCAL_PORT_DEFAULT;
	cl_params.udp_remote_port = UDP_REMOTE_PORT_DEFAULT;
	cl_params.exposure = 100;
	cl_params.gain = 0;
	cl_params.auto_exposure = false;

	parseCommandline(argc, argv, cl_params);

	//check which calibration file to take if calibration_path != "" assume it's the correct one
	if (cl_params.calibration_path == "" && cl_params.res == "VGA") { //default
#ifdef CATKIN_BUILD
		cl_params.calibration_path = ros::package::getPath("snap_cam") + "/calib/VGA/cameraParameters.yaml";
#else
		cl_params.calibration_path = "../calib/VGA/cameraParameters.yaml";
		INFO("Using VGA/cameraParameters.yaml for calibration");
#endif
	}

	if (cl_params.calibration_path == "" && cl_params.res == "QVGA") { //default for QVGA
#ifdef CATKIN_BUILD
		cl_params.calibration_path = ros::package::getPath("snap_cam") + "/calib/QVGA/cameraParameters.yaml";
#else
		cl_params.calibration_path = "../calib/QVGA/cameraParameters.yaml";
		INFO("Using QVGA/cameraParameters.yaml for calibration");
#endif
	}

	loadCustomCameraCalibration(cl_params.calibration_path);

	CamConfig cfg;
	cfg.func = CAM_FUNC_OPTIC_FLOW;

	if (cl_params.res == "VGA") {
		cfg.pSize = CameraSizes::VGASize();

	} else if (cl_params.res == "QVGA") {
		cfg.pSize = CameraSizes::QVGASize();

	} else {
		ERROR("Invalid resolution specification %s. Defaulting to VGA\n", cl_params.res.c_str());
		cfg.pSize = CameraSizes::stereoQVGASize();
	}

	cfg.exposureValue = cl_params.exposure;
	cfg.gainValue = cl_params.gain;

	image_width = cfg.pSize.width;
	image_height = cfg.pSize.height;

	switch (cl_params.fps) {
	case 30:
		cfg.fps = 30;
		break;

	case 60:
		cfg.fps = 60;
		break;

	case 90:
		cfg.fps = 90;
		break;

	default:
		cfg.fps = 30;
		ERROR("Invalid frame-rate option %d. Defaulting to 30 FPS\n", cl_params.fps);
	}

	// try to setup udp socket for communcation
	if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		ERROR("create socket failed\n");
		return -1;
	}

	// set up udp communcation
	// my address
	memset(&myAddr, 0, sizeof(myAddr));
	myAddr.sin_family = AF_INET;
	myAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myAddr.sin_port = htons(cl_params.udp_local_port);
	INFO("local port: %d\n", cl_params.udp_local_port);

	// remote address
	memset(&theirAddr, 0, sizeof(theirAddr));
	theirAddr.sin_family = AF_INET;
	theirAddr.sin_addr.s_addr = inet_addr(cl_params.target_ip.c_str());
	theirAddr.sin_port = htons(cl_params.udp_remote_port);
	INFO("remote address: %s:%d\n", cl_params.target_ip.c_str(), cl_params.udp_remote_port);

	theirAddrSend.sin_family = AF_INET;
	theirAddrSend.sin_addr.s_addr = htonl(INADDR_ANY);
	theirAddrSend.sin_port = htons(UDP_REMOTE_SEND_PORT_DEFAULT);

	/* Bind the socket to port - necessary to receive packets */
	if (-1 == bind(sock, (struct sockaddr *)&myAddr, sizeof(struct sockaddr))) {
		perror("error bind failed");
		close(sock);
		exit(EXIT_FAILURE);
	}

	/* Attempt to make it non blocking */
	if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(sock);
		exit(EXIT_FAILURE);
	}

	//initialize optical flow
	_optical_flow = new OpticalFlowOpenCV(cameraParams.CameraMatrix[0][0], cameraParams.CameraMatrix[1][1],
			OPTICAL_FLOW_OUTPUT_RATE, IMAGE_CROP_WIDTH, IMAGE_CROP_HEIGHT);
	_optical_flow->setCameraMatrix(cameraParams.CameraMatrix[0][0], cameraParams.CameraMatrix[1][1],
			cameraParams.CameraMatrix[0][2] - (cfg.pSize.width/2 - IMAGE_CROP_WIDTH/2),
			cameraParams.CameraMatrix[1][2] - (cfg.pSize.height/2 - IMAGE_CROP_HEIGHT/2)); //account for cropping
	_optical_flow->setCameraDistortion(cameraParams.RadialDistortion[0], cameraParams.RadialDistortion[1],
			cameraParams.RadialDistortion[2]);

	// calculate the offset of the AppsProc time from the aDSP time, for
	// use later in converting the IMU timestamp from the aDSP to
	// AppsProc time
	if (calc_imu_time_offset() != 0)
	{
	  fprintf(stderr, "error calculating the aDSP/AppsProc time offset.");
	  exit(EXIT_FAILURE);
	}

	// open the camera and set the callback to get the images
	SnapCam cam(cfg);
	cam.setListener(imageCallback);
	if (cl_params.auto_exposure) {
		INFO("Using auto exposure");
		cam.setAutoExposure(cl_params.auto_exposure);
	}

	while (1) {
		// read mavlink messages quickly to keep up with the queue
		read_mavlink_messages(sock, theirAddr);
		usleep(1e3); // loop with 1 kHz
	}

}  // main

uint64_t get_absolute_time()
{
	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_MONOTONIC, &time);
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;
	return micros;
}

void loadCustomCameraCalibration(const std::string calib_path)
{
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

	ssize_t len = sendto(sock, buf, packet_len, 0, (struct sockaddr *)&theirAddrSend, sizeof(struct sockaddr_in));

	if (len <= 0) {
		ERROR("Failed sending mavlink message\n");
	}
}

void sendOptFlowMessage()
{
	while (!rb_flow.empty()) {
		// check if we already have all the imu messages for the oldest flow
		mavlink_optical_flow_rad_t oldest_flow;

		if (!rb_flow.peak(&oldest_flow)) {
			ERROR("Failed to peak flow");
			return;
		}

		uint64_t integration_start_time = oldest_flow.time_usec - oldest_flow.integration_time_us;

		GyroTimestamped latest_gyro;
		if (!rb_imu.peak_head(&latest_gyro)) {
			ERROR("IMU buffer is empty");
			return;
		}

		if (integration_start_time > latest_gyro.time_usec) {
			WARN("The integration start time is %lld us ahead of the lastest IMU", integration_start_time - latest_gyro.time_usec);
			WARN("Oldest flow time %lld , latest gyro time %lld, integration_start_time %lld", oldest_flow.time_usec,
			     latest_gyro.time_usec, integration_start_time);
		}

		mavlink_optical_flow_rad_t sensor_msg;
		rb_flow.get(&sensor_msg);

		GyroTimestamped g;

		// first skip any gyro measurements that are too old (should only happen the first time)
		do {
			if (!rb_imu.get(&g)) {
				ERROR("IMU buffer is empty while catching up");
				return;
			}
		} while (g.time_usec < integration_start_time);

		// now integrate the measurements until we are at the current image
		double xgyro_int = 0;
		double ygyro_int = 0;
		double zgyro_int = 0;
		double dt = 0;
		uint64_t last_imu_time = g.time_usec;

		do {
			if (!rb_imu.get(&g)) {
				break;
			}

			dt = double(g.time_usec - last_imu_time) / 1e6;
			last_imu_time = g.time_usec;
			if (g.time_usec == 0 || last_imu_time == 0 || dt > 0.01) {
				WARN("IMU time stamp equals 0");
			} else {
				xgyro_int += g.xgyro * dt;
				ygyro_int += g.ygyro * dt;
				zgyro_int += g.zgyro * dt;
			}
		} while (g.time_usec < sensor_msg.time_usec);

		sensor_msg.integrated_xgyro = - ygyro_int; //swap directions to match PX4Flow and SENS_FLOW_ROT 6 (270 degrees)
		sensor_msg.integrated_ygyro = xgyro_int; //swap directions to match PX4Flow and SENS_FLOW_ROT 6 (270 degrees)
		sensor_msg.integrated_zgyro = zgyro_int;

		//send optical flow mavlink message to px4
		send_mavlink_message(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &sensor_msg, 200);
	}
}

void calcOptFlow(const cv::Mat &Image, uint64_t img_timestamp)
{

	float flow_x_ang = 0.0f;
  float flow_y_ang = 0.0f;
	static int dt_us;

	int flow_quality = _optical_flow->calcFlow(Image.data, img_timestamp, dt_us, flow_x_ang, flow_y_ang);

	if (flow_quality >= 0) {

		mavlink_optical_flow_rad_t sensor_msg;

		sensor_msg.time_usec = img_timestamp;
		sensor_msg.sensor_id = 0; //?
		sensor_msg.integration_time_us = dt_us;
		sensor_msg.integrated_x = - flow_y_ang; //swap directions to match PX4Flow and SENS_FLOW_ROT 6 (270 degrees)
		sensor_msg.integrated_y = flow_x_ang; //swap directions to match PX4Flow and SENS_FLOW_ROT 6 (270 degrees)
		sensor_msg.integrated_xgyro = 0.0;  // will be filled later
		sensor_msg.integrated_ygyro = 0.0;  //  will be filled later
		sensor_msg.integrated_zgyro = 0.0;  //  will be filled later
		sensor_msg.temperature = 0.0;
		sensor_msg.quality = flow_quality;
		sensor_msg.time_delta_distance_us = 0.0; //?
		sensor_msg.distance = -1.0; // mark as invalid

		if (rb_flow.force(&sensor_msg)) {
			WARN("Flow buffer is overflowing %d", rb_flow.space());
		}

		sendOptFlowMessage();
	}
}

void imageCallback(const cv::Mat &img, uint64_t time_stamp)
{
	static cv::Rect crop(image_width/2-IMAGE_CROP_WIDTH/2, image_height/2-IMAGE_CROP_HEIGHT/2,
			IMAGE_CROP_WIDTH, IMAGE_CROP_HEIGHT);
	cv::Mat croppedImage = img(crop);
	cv::Mat cropped;
	// Copy the data into new matrix -> croppedImage.data can not be used in calcFlow()...
	croppedImage.copyTo(cropped);

	calcOptFlow(cropped, time_stamp);
	croppedImage.release();
	cropped.release();
}

int parseCommandline(int argc, char *argv[], Params &cl_params)
{
	int c;
	int ret = 0;

	while ((c = getopt(argc, argv, "c:r:n:f:e:g:a")) != -1) {
		switch (c) {
		case 'c': {
				cl_params.calibration_path =  std::string(optarg);
				break;
			}

		case 'r': {
				cl_params.res = std::string(optarg);
				break;
			}

		case 'n': {
				cl_params.num_features = atoi(optarg);
				cl_params.num_features = cl_params.num_features < 0 ? 0 : cl_params.num_features;
				break;
			}

		case 'f': {
				cl_params.fps = atoi(optarg);
				break;
			}

		case 'e': {
				if (atoi(optarg) >= 0 && atoi(optarg) < 500)
					cl_params.exposure = atoi(optarg);
				break;
			}

		case 'g': {
			if (atoi(optarg) >= 0 && atoi(optarg) < 256)
				cl_params.gain = atoi(optarg);
				break;
			}

		case 'a': {
			cl_params.auto_exposure = true;
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

void handle_message(mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
		handle_message_highres_imu(msg);
	}
}

void handle_message_highres_imu(mavlink_message_t *msg)
{
	mavlink_highres_imu_t highres_imu;
	mavlink_msg_highres_imu_decode(msg, &highres_imu);
	memcpy(&last_highres_imu, &highres_imu, sizeof(mavlink_highres_imu_t));
	GyroTimestamped gyro;
	gyro.xgyro      = highres_imu.xgyro;
	gyro.ygyro      = highres_imu.ygyro;
	gyro.zgyro      = highres_imu.zgyro;
	gyro.time_usec  = highres_imu.time_usec;

	if (rb_imu.force(&gyro)) {
		ERROR("IMU buffer is overflowing!");
	}
}

bool read_mavlink_messages(int &sock, struct sockaddr_in &theirAddr)
{
	bool ret = false;
	ssize_t recsize;
	socklen_t fromlen;

	memset(buf, 0, BUFFER_LENGTH);
	recsize = recvfrom(sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&theirAddr, &fromlen);

	if (recsize > 0) {
		// Something received
		mavlink_message_t msg;
		mavlink_status_t status;
		unsigned int temp = 0;

		for (int i = 0; i < recsize; ++i) {
			temp = buf[i];

			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
				// Packet received
				handle_message(&msg);
				ret = true;
			}
		}
	}

	memset(buf, 0, BUFFER_LENGTH);
	return ret;
}

int calc_imu_time_offset(void)
{
	int64_t dsptimeNsec;
	int64_t appstimeInNsec;
	uint64_t timeNSecMonotonic;
	struct timespec t;
	static const char qdspTimerTickPath[] = "/sys/kernel/boot_adsp/qdsp_qtimer";
	char qdspTicksStr[20] = "";
	static const double clockFreqUsec = 1 / 19.2;

	FILE * qdspClockfp = fopen(qdspTimerTickPath, "r");
	if (qdspClockfp != NULL) {
		fread(qdspTicksStr, 16, 1, qdspClockfp);
		uint64_t qdspTicks = strtoull(qdspTicksStr, 0, 16);
		fclose(qdspClockfp);
		dsptimeNsec = (int64_t) (qdspTicks * clockFreqUsec * 1e3);
	} else {
		printf("error: unable to read the Q6 DSP time clock.\n");
		return -1;
	}

	clock_gettime(CLOCK_MONOTONIC, &t);
	timeNSecMonotonic = (uint64_t) (t.tv_sec) * 1000000000ULL + t.tv_nsec;
	appstimeInNsec = (int64_t) timeNSecMonotonic;

	imu_time_offset_usecs = (appstimeInNsec - dsptimeNsec) / 1000;
	printf("IMU time offset is: %lld\n", imu_time_offset_usecs);

	return 0;
}
