/**
 * @brief Declaration of the class CameraCalibration.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CameraCalibration.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <vector>
#include "Eigen/Dense"


//#define DEPTH_IM_RES_DIV 2.0

class CalibrationData;
class CalibrationSettings;
class MicroLensGrid;
class EpiPolarLine;
class Constraint;
class ColmapReconstructor;
struct frame;

class CameraCalibration
{
public:
	CameraCalibration(std::string type_of_calibration, std::string settingsFile, std::string constraintsFile);
	~CameraCalibration();

private:
	void initialization();
	bool doCalibration_aruco_calib();
	bool doCalibration_recalib();
	bool readDepthData();
	bool initPlenopticParameters();
	bool initPlenopticParametersRecalibration();
	void defineEpiPolarLines();
	bool projectPointsToRawImage();
	bool performBundleAdjustment();
	void showResults();
	bool storeResults();

	bool storeCameraModel();
	bool storeExtrinsicOrientations();
	bool storeExtrinsicOrientationsTxt();
	bool storeRawImagePointsCsv();
	bool storeProtocol();

	void calcReprojectionError(double inlierThreshold = 1.0);

	// Path to the yaml file with the calibration settings
	std::string dirSettingsFile;
	// Path to the file with the constraints
	std::string dirConstraintsFile;

	/// Path to total focus images
	std::string dirTotalFocusImages;
	/// Path to fisrt calibration results
	std::string dirFirstCalibrationData;
	/// Path to virtual depth data.
	std::string dirDepthData;
	/// Path to micro lens calibration file
	std::string dirMlCalibFile;
	/// Path to result directory
	std::string dirResults;

	int config;
	
	/// Number of 3D points used for the light field bundle adjustment
	int n3DPoints;
	/// Number of the used OpenCV marker dictionary
	int markerDict;

	int nRadialDistParam;
	bool tangentialDistParam;
	/// Camera pose are optimized
	bool refinePoses;
	/// 3D object points are optimized
	bool refine3Dpoints;
	/// Robust cost function has to be used
	bool useRobustCostFunction;
	/// adjust micro lens centers (since micro lens centers are different from estimated micro image centers)
	bool mlCenterAdjustment;

	/// Calibration settings read in file
	CalibrationSettings* calibSettings;

	/// Stores all calibration data read from the html file
	CalibrationData* calibData;

	/// First estimate for the reconstruction with colmap
	ColmapReconstructor* firstEstimateColmap;

	/// Micro lens grid (contains all micro lens centers)
	MicroLensGrid* mlGrid;
	std::vector<std::vector<EpiPolarLine*>*> epiLineWeb;

	std::vector<Eigen::Vector3d> p3d_w;
	std::vector<frame> frames;
	/// Virtual depth values including outliers
	std::vector<std::vector<double>> virtualDepthValues;
	std::vector<Constraint> constraintList;
	std::vector<Eigen::Vector4i> orientationConstraint;

	/// Initial parameters
	double pixelSize;	// Size of a pixel in mm
	double pixelSize_totFoc;	// Equivalent size of a pixel in mm in the totally focused image
	int rawWidth;	// Width of the raw image in pixels
	int rawHeight;	// Height of the raw image in pixels
	Eigen::Vector2i imageSize;
	Eigen::Vector2d c_init; // Principal point (x,y) in pixel
	Eigen::Vector2d radialDist_init; // Radial distortion parameters
	Eigen::Vector2d tangentialDist_init; // Tangential distortion parameters
	std::vector<Eigen::VectorXd> poses_init;
	double fPH_init, fL_init, bL0_init, B_init;

	/// Scaling factor between virtual depth and raw image
	int depth_to_raw_im_scale;

	double* init_camera, *camera, *init_views, *views; 

	std::vector<std::vector<Eigen::Vector3d>> projectionErrors;
	double std_x, std_y, std_iv;
	double mae_x, mae_y, mae_iv;
	int numPts, numInliers;

	// Estimated parameters
	double fL, bL0, B;
	Eigen::Vector2d c;
	Eigen::VectorXd radialDist;
	Eigen::Vector2d tangentialDist;
	Eigen::VectorXd depthDist;
};
