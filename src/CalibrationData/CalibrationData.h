/**
 * @brief Declaration of the class CalibrationData.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CalibrationData.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>


#define MAX_NUMBER_OF_CAMERA_PARAMETERS 17

class IntrinsicOrientation;
class ExtrinsicOrientations;
class ObjectPoints;
class Images;
class Constraints;
class Constraint;
class ArucoDetection;

struct frame
{
	int id;
	Eigen::Matrix4d worldToCam;
	Eigen::Vector3d rotationAngles;
	Eigen::Vector3d transVector;

	std::vector<Eigen::Vector2d>  imageCoordinates;
	std::map<int,Eigen::Vector2d*> imageCoordinatesByID;
	std::map<int,Eigen::Vector3d*> objectCoordinatesByID;

	std::vector<Eigen::Vector2d>  rawImageCoordinates;
	std::vector<Eigen::Vector2d>  microLensCenter;
	std::vector<Eigen::Vector3d*> objectCoordinatesByRawID;
};

struct constraint
{
	int id_1;
	int id_2;
	double distance;
};

enum type_of_calibration
{
	CALIBRATION_ARUCO,
	RECALIBRATION
};

class CalibrationData
{
public:
	CalibrationData(void);
	~CalibrationData(void);

	/// Read Data from from the first calibration
	bool readDataFromFirstCalibration(std::string folderPath);
	/// Read Data from from the constraints file
	bool readConstaints(std::string dirConstraintsFile);
	/// Read fixed parameters from the file
	bool readFixedParameters(std::string dirFixedParametersFile);
	/// Reduce the number of points to represent the 3D scene
	void ReduceNumberPoints(int nbPoints);
	/// Detect aruco markers in the image and add the points to the data
	void addArucoMarkersToData(int dictionaryMarkerNumber, std::string folderPathTotalFocus);
	/// Scale the point cloud and the extrinsic orientation using the constraints
	void scaleData();
	/// Remap point IDs and get transformations.
	bool getCalibDataCV(std::vector<Eigen::Vector3d> &worldCoordinates, std::vector<frame> &frames);
	/// Returns a list of the constraints coming from the file
	bool getConstraintsList(std::vector<Constraint> &constraintList);
	/// Returns data in Computer Vision conventional format
	bool getIntrinsicParamCV(double &c, Eigen::Vector2i &imageSize, Eigen::Vector2d &h, Eigen::Vector2d &a, Eigen::Vector2d &b);
	/// Get the corresponding COLMAP ID from the ID of a 3D point
	int getCorrespondingCOLMAPID(int id);
	/// Get fixed parameters f and B for recalibration
	bool getFixedParameters(double &f, double &B);
	/// Type of the calibration
	enum type_of_calibration calib_type;

private:

	/// Pointer to intrinsic orientation data
	IntrinsicOrientation* ptIntOr;
	/// Pointer to extrinsic orientation data
	ExtrinsicOrientations* ptExtOr;
	/// Pointer to object point data
	ObjectPoints* ptObjPts;
	/// Pointer to image coordinate data
	Images* ptImPts;
	/// Pointer to constraints.
	Constraints* ptConstraints;
	/// Pointer to image coordinate data for aruco markers
	ArucoDetection* ptArucoPts;
	// mapping ID from first estimate to new ID
	std::map<int,int> pointIdMap;
	/// Map new ID to Colmap ID
	std::map<int,int> pointIdMapFromNewToColmap;
	/// Fixed values for f and B in the case of a recalibration
	double f_fixed;
	double B_fixed;
};
