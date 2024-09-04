/**
 * @brief Declaration of the classes ObjectPoints and ObjectPoint.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ObjectPoints.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <Eigen/Dense>

#include "colmap/scene/point3d.h"


class ObjectPoint;

class ObjectPoints
{
public:
	ObjectPoints(void);
	~ObjectPoints(void);

	/// Load object point data from the map
	bool LoadObjectPoints(std::unordered_map<colmap::point3D_t, class colmap::Point3D> map_point3D);

	/// Vector of all object points
	std::vector<ObjectPoint*> objPoints;
	/// Object points by point ID
	std::map<int,ObjectPoint*> objPointsByPointID;
};

class ObjectPoint
{
public:
	ObjectPoint(int pointID, Eigen::Vector3d coordinate, double error, int nRays)
	{
		this->pointID = pointID;
		this->coordinate = coordinate;
		this->error  = error;
		this->nRays = nRays;
	}

	~ObjectPoint(void);

	/// object point ID
	int pointID;
	/// 3d world coordinates
	Eigen::Vector3d coordinate;

private:
	/// Error for the point
	double error;
	/// Number of rays
	int nRays;
};
