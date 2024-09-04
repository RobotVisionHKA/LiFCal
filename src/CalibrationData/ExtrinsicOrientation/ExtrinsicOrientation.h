/**
 * @brief Declaration of the class ExtrinsicOrientation.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ExtrinsicOrientation.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <Eigen/Dense>

#include "colmap/geometry/rigid3.h"


class ExtrinsicOrientation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ExtrinsicOrientation(int imageID, colmap::Rigid3d transformation, int cameraID, std::string imageName);
	~ExtrinsicOrientation(void);

	/// Calculate transform from world to camera coordinates.
	void calculateWorldToCamera();

	/// Image ID.
	int imageID;
	/// Rotation quaternions.
	Eigen::Quaterniond rotQuat;
	/// Translation vector.
	Eigen::Vector3d tranVec;
	/// Transformation matrix world to camera.
	Eigen::Matrix4d worldToCameraMatrix;

private:
	int cameraID;
	std::string imageName;
};
