#ifndef CALIB_YAML_INTERFACE_H_
#define CALIB_YAML_INTERFACE_H_

#include <yaml-cpp/yaml.h>
#include <string>
#include "logging.h"

// cameraParameters
// =========================================================
struct CameraParameters { // parameters of one camera
	double CameraMatrix[3][3];
	double FocalLength[2];
	double PrincipalPoint[2];
	double RadialDistortion[3];
	enum {PLUMB_BOB = 0, ATAN = 1};
	int DistortionModel;
};


inline CameraParameters parseYaml(const YAML::Node &node)
{
	CameraParameters v;

	std::string plumb_bob = "plumb_bob";
	std::string atan = "atan";

	YAML::Node CameraMatrix = node["CameraMatrix"];

	for (std::size_t i = 0; i < CameraMatrix.size(); i++) {
		for (std::size_t j = 0; j < CameraMatrix[i].size(); j++) {
			v.CameraMatrix[i][j] = CameraMatrix[i][j].as<double>();
		}
	}


	YAML::Node DistortionModel = node["DistortionModel"];

	if (!atan.compare(DistortionModel.as<std::string>())) {
		v.DistortionModel = v.ATAN;

	} else {
		v.DistortionModel = v.PLUMB_BOB;
	}

	YAML::Node RadialDistortion = node["RadialDistortion"];

	for (std::size_t i = 0; i < RadialDistortion.size(); i++) {
		v.RadialDistortion[i] = RadialDistortion[i].as<double>();
	}

	for (std::size_t i = RadialDistortion.size(); i < 3; i++) {
		v.RadialDistortion[i] = 0.0;
	}

	YAML::Node FocalLength = node["FocalLength"];

	for (std::size_t i = 0; i < FocalLength.size(); i++) {
		v.FocalLength[i] = FocalLength[i].as<double>();
	}

	YAML::Node PrincipalPoint = node["PrincipalPoint"];

	for (std::size_t i = 0; i < PrincipalPoint.size(); i++) {
		v.PrincipalPoint[i] = PrincipalPoint[i].as<double>();
	}

	return v;
}

#endif
