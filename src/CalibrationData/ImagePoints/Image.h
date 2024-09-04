/**
 * @brief Declaration of the classes Image and ImageCoordinate.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file Image.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>


class ImageCoordinate;

class Image
{
public:
	Image(int id){this->id = id;}
	~Image(void);

	/// Image id
	int id;
	/// Vector of image coordinates
	std::vector<ImageCoordinate*> imageCoordinates;
	/// Vector of image coordinates without outliers
	std::vector<ImageCoordinate*> imageCoordinatesInliers;
	/// Image coordinates by point ID
	std::map<int,ImageCoordinate*> imageCoordinatesByPointID;
};


class ImageCoordinate
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ImageCoordinate(int pointID, int imageID, Eigen::Vector2d coordinate, bool outlier)
	{
		this->pointID = pointID;
		this->imageID = imageID;
		this->coordinate = coordinate;
		this->outlier = outlier;
	}

	~ImageCoordinate(void){}

	/// Returns true if the that image point was detected as outlier
	inline bool isOutlier(){return outlier;}
	/// Object point ID
	int pointID;
	/// Image ID
	int imageID;
	/// Measured image coordinates
	Eigen::Vector2d coordinate;

private:
	/// Outlier or not
	bool outlier;
};
