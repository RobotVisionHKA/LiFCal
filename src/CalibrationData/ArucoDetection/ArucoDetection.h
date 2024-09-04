/**
 * @brief Declaration of the class ArucoDetection.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ArucoDetection.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <vector>
#include <map>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "Eigen/Dense"


class Image;

class ArucoDetection
{
public:
	ArucoDetection(int indexDictionary, std::string folderPathTotalFocus);
	~ArucoDetection(void);

	/// Detect the aruco markers in the total focus images.
	void detectArucoMarkers(std::vector<int> imagesIDOrder);

	/// Get the number of aruco markers detected in each image.
	void GetNumberOfInliersForEachImage(void);

	/// Get the coordinates of the center of the aruco markers from the coordinates of the angles.
	Eigen::Vector2d getCenterMarker(std::vector<cv::Point2f> cornersMarker);

	/// Vector of all images
	std::vector<Image*> imagesAruco;
	/// Images by image ID.
	std::map<int,Image*> imagesByIDAruco;
	/// List of the IDs of the aruco markers
	std::vector<int> markerIds;
	/// Number of aruco markers inliers for each image
	std::vector<int> NumberInliersAruco;

private:
	/// Type of dictionnary
	//	dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,
    //  DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, 
    //  DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,
    //  DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,
    //  DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20
    int indexDictionary;
    std::string folderPathTotalFocus;
};

