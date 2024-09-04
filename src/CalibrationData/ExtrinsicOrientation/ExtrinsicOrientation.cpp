/**
 * @brief Definition of the class ExtrinsicOrientation.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ExtrinsicOrientation.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <iostream>

#include "CameraModel.h"
#include "ExtrinsicOrientation.h"


ExtrinsicOrientation::ExtrinsicOrientation(int imageID, colmap::Rigid3d transformation, int cameraID, std::string imageName)
{
	this->imageID = imageID;
	this->rotQuat = transformation.rotation;
	this->tranVec = transformation.translation;
	this->cameraID = cameraID;
	this->imageName = imageName;

	Eigen::Matrix3x4d worldToCameraMatrix3x4 = transformation.ToMatrix();

	this->worldToCameraMatrix.block<3,4>(0,0) = worldToCameraMatrix3x4;
	this->worldToCameraMatrix.block<1,4>(3,0).array() = 0, 0, 0, 1;
}

ExtrinsicOrientation::~ExtrinsicOrientation(void)
{
}
