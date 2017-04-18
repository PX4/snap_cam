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

#include "trackFeatures.h"
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
std::vector<cv::Point2f> features_current;
std::vector<int> updateVector;
cv::Mat_<cv::Point2f> out_features_previous;

uint64_t img_timestamp_prev = 0;
mavlink_highres_imu_t last_highres_imu;

CameraParameters cameraParams = {};

int64_t imu_time_offset_usecs = 0;

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
	cl_params.fps = 15;
	cl_params.target_ip = "127.0.0.1";
	cl_params.udp_local_port = UDP_LOCAL_PORT_DEFAULT;
	cl_params.udp_remote_port = UDP_REMOTE_PORT_DEFAULT;

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

	switch (cl_params.fps) {
	case 15:
		cfg.fps = 0;
		break;

	case 24:
		cfg.fps = 1;
		break;

	case 30:
		cfg.fps = 2;
		break;

	case 60:
		cfg.fps = 3;
		break;

	case 90:
		cfg.fps = 4;
		break;

	case 120:
		cfg.fps = 5;
		break;

	default:
		cfg.fps = 0;
		ERROR("Invalid frame-rate option %d. Defaulting to 15 FPS\n", cl_params.fps);
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

	updateVector.resize(cl_params.num_features, 2);

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
				ERROR("IMU buffer is empty! Last IMU time %lld, new image time %lld, dt %lld", g.time_usec, sensor_msg.time_usec,
				      sensor_msg.time_usec - g.time_usec);
				return;
			}

			dt = double(g.time_usec - last_imu_time) / 1e6;
			last_imu_time = g.time_usec;
			xgyro_int += g.xgyro * dt;
			ygyro_int += g.ygyro * dt;
			zgyro_int += g.zgyro * dt;
		} while (g.time_usec < sensor_msg.time_usec);

		sensor_msg.integrated_xgyro = xgyro_int;
		sensor_msg.integrated_ygyro = ygyro_int;
		sensor_msg.integrated_zgyro = zgyro_int;

		//send optical flow mavlink message to px4
		send_mavlink_message(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, &sensor_msg, 200);
	}
}

void calcOptFlow(const cv::Mat &Image, uint64_t img_timestamp)
{
	if (!img_timestamp_prev) {
		img_timestamp_prev = img_timestamp;
		return;
	}

	std::vector<cv::Point2f> useless;
	int meancount = 0;

	double pixel_flow_x_mean = 0.0;
	double pixel_flow_y_mean = 0.0;
	double pixel_flow_x_integral = 0.0;
	double pixel_flow_y_integral = 0.0;
	double pixel_flow_x_stddev = 0.0;
	double pixel_flow_y_stddev = 0.0;

	trackFeatures(Image, Image, features_current, useless, updateVector, 0);

	cv::Mat_<float> cam_matrix(3, 3);
	cam_matrix <<   cameraParams.CameraMatrix[0][0], cameraParams.CameraMatrix[0][1], cameraParams.CameraMatrix[0][2],
		   cameraParams.CameraMatrix[1][0], cameraParams.CameraMatrix[1][1], cameraParams.CameraMatrix[1][2],
		   cameraParams.CameraMatrix[2][0], cameraParams.CameraMatrix[2][1], cameraParams.CameraMatrix[2][2];
	cv::Mat_<float> distortion(1, 4);
	distortion <<   cameraParams.RadialDistortion[0], cameraParams.RadialDistortion[1], 0, 0,
		   cameraParams.RadialDistortion[2];

	int npoints = updateVector.size();
	cv::Mat_<cv::Point2f> out_features_current(1, npoints);

	cv::undistortPoints(features_current, out_features_current, cam_matrix, distortion);

	// cv::undistortPoints returns normalized coordinates... -> convert
	for (int i = 0; i < npoints; i++) {
		out_features_current(i).x = out_features_current(i).x * cameraParams.CameraMatrix[0][0] +
					    cameraParams.CameraMatrix[0][2];
		out_features_current(i).y = out_features_current(i).y * cameraParams.CameraMatrix[1][1] +
					    cameraParams.CameraMatrix[1][2];
	}

	if (!out_features_current.empty() && !out_features_previous.empty()) {
		// compute the mean flow
		for (int i = 0; i < updateVector.size(); i++) {
			if (updateVector[i] == 1) {
				pixel_flow_x_mean += out_features_current(i).x - out_features_previous(i).x;
				pixel_flow_y_mean += out_features_current(i).y - out_features_previous(i).y;
				meancount++;
			}
		}

		if (meancount) {
			pixel_flow_x_mean /= meancount;
			pixel_flow_y_mean /= meancount;

			// compute the flow variance
			for (int i = 0; i < updateVector.size(); i++) {
				if (updateVector[i] == 1) {
					pixel_flow_x_stddev += (out_features_current(i).x - out_features_previous(i).x - pixel_flow_x_mean) *
							       (out_features_current(i).x - out_features_previous(i).x - pixel_flow_x_mean);
					pixel_flow_y_stddev += (out_features_current(i).y - out_features_previous(i).y - pixel_flow_y_mean) *
							       (out_features_current(i).y - out_features_previous(i).y - pixel_flow_y_mean);
				}
			}

			pixel_flow_x_stddev /= meancount;
			pixel_flow_y_stddev /= meancount;

			// convert to std deviation
			pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev);
			pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev);

			// re-compute the mean flow with only the 95% consenting features
			meancount = 0;

			for (int i = 0; i < updateVector.size(); i++) {
				if (updateVector[i] == 1) {
					double this_flow_x = out_features_current(i).x - out_features_previous(i).x;
					double this_flow_y = out_features_current(i).y - out_features_previous(i).y;

					if (abs(this_flow_x - pixel_flow_x_mean) < 2 * pixel_flow_x_stddev
					    && abs(this_flow_y - pixel_flow_y_mean) < 2 * pixel_flow_y_stddev) {
						pixel_flow_x_integral += out_features_current(i).x - out_features_previous(i).x;
						pixel_flow_y_integral += out_features_current(i).y - out_features_previous(i).y;
						meancount++;

					} else {
						updateVector[i] = 0;
					}
				}

			}

			if (meancount) {
				pixel_flow_x_integral /= meancount;
				pixel_flow_y_integral /= meancount;

				double flow_quality = 255.0 * meancount / updateVector.size();
				uint32_t delta_time = img_timestamp - img_timestamp_prev;

				double flow_x_ang = atan2(pixel_flow_x_integral, cameraParams.FocalLength[0]);
				double flow_y_ang = atan2(pixel_flow_y_integral, cameraParams.FocalLength[1]);

				mavlink_optical_flow_rad_t sensor_msg;

				sensor_msg.time_usec = img_timestamp;
				sensor_msg.sensor_id = 0; //?
				sensor_msg.integration_time_us = delta_time;
				sensor_msg.integrated_x = flow_x_ang;
				sensor_msg.integrated_y = flow_y_ang;
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

		} else {
			WARN("No valid measurements");
		}
	}

	for (int i = 0; i < updateVector.size(); i++) {
		if (updateVector[i] == 2) {
			updateVector[i] = 1;
		}

		if (updateVector[i] == 0) {
			updateVector[i] = 2;
		}
	}

	out_features_previous = out_features_current;
	img_timestamp_prev = img_timestamp;
}

void imageCallback(const cv::Mat &img, uint64_t time_stamp)
{
	calcOptFlow(img, time_stamp);
}

int parseCommandline(int argc, char *argv[], Params &cl_params)
{
	int c;
	int ret = 0;

	while ((c = getopt(argc, argv, "c:r:n:f:")) != -1) {
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
