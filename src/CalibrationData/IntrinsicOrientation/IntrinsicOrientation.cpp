/**
 * @brief Definition of the class IntrinsicOrientation.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file IntrinsicOrientation.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>

#include "CalibrationData/IntrinsicOrientation/IntrinsicOrientation.h"

#include <colmap/sensor/models.h>


IntrinsicOrientation::IntrinsicOrientation(void)
{
	this->fx = 0;
	this->fy = 0;
	this->cx = 0;
	this->cy = 0;
	this->k[0] = 0; this->k[1] = 0;
	this->p[0] = 0; this->p[1] = 0;
}

IntrinsicOrientation::~IntrinsicOrientation(void)
{
}

/**
 * @brief Returns intrinsic parameters.
 */
void IntrinsicOrientation::getIntrinsicParam(double &f, Eigen::Vector2i &imageSize, Eigen::Vector2d &c, Eigen::Vector2d &k, Eigen::Vector2d &p)
{
	f = (this->fx + this->fy) / 2; // Take the mean of the focal legths in both directions
	imageSize = Eigen::Vector2i(this->width, this->height);
	c = Eigen::Vector2d(this->cx, this->cy);
	k = Eigen::Vector2d(this->k[0], this->k[1]);
	p = Eigen::Vector2d(this->p[0], this->p[1]);
}

/**
 * @brief Load instrinsic orientation data from the map.
 */
bool IntrinsicOrientation::LoadIntrinsicOrientation(std::unordered_map<colmap::camera_t, class colmap::Camera> map_cameras)
{
	// All images are taken with the same camera
	colmap::camera_t index = 1;
	if(map_cameras[index].NumParams() != colmap::CameraModelNumParams(map_cameras[index].ModelId())) return false;
	else
	{
		this->width = map_cameras[index].Width();
		this->height = map_cameras[index].Height();
		this->fx = map_cameras[index].Params(0);
		this->fy = map_cameras[index].Params(1);
		this->cx = map_cameras[index].Params(2);
		this->cy = map_cameras[index].Params(3);
		this->k[0] = map_cameras[index].Params(4);
		this->k[1] = map_cameras[index].Params(5);
		this->p[0] = map_cameras[index].Params(6);
		this->p[1] = map_cameras[index].Params(7);
	}

	return true;
}
