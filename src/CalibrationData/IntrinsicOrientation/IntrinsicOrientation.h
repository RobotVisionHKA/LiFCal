/**
 * @brief Declaration of the class IntrinsicOrientation.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file IntrinsicOrientation.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>

#include <colmap/scene/reconstruction.h>


class IntrinsicOrientation
{
public:
	IntrinsicOrientation(void);
	~IntrinsicOrientation(void);
	
	/// Load instrinsic orientation data from the map.
	bool LoadIntrinsicOrientation(std::unordered_map<colmap::camera_t, class colmap::Camera> map_cameras);

	void getIntrinsicParam(double &f, Eigen::Vector2i &imageSize, Eigen::Vector2d &c, Eigen::Vector2d &k, Eigen::Vector2d &p);

	void printParameter(){
		printf("INTRINSIC ORIENTATION:\n\tfx: %2.10f\n\tfy: %2.10f\n\tcx: %2.10f\n\tcy: %2.10f\n\tk1: %2.10f\n\tk2: %2.10f\n\tp1: %2.10f\n\tp2: %2.10f\n\twidth: %d\n\theight: %d\n",
			fx,fy,cx,cy,k[0],k[1],p[0],p[1], width, height);}

private:
	double fx, fy;
	double cx, cy;
	double k[2];	// Coefficients for radial distortion
	double p[2];	// Coefficients for tangential distortion
	int width, height;
};
