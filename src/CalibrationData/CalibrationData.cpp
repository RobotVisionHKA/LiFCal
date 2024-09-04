/**
 * @brief Definition of the class CalibrationData.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CalibrationData.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>
#include <iostream>
#include <string>
#include <limits.h>
#include <numeric>
#include <random>

#include "CalibrationData/IntrinsicOrientation/IntrinsicOrientation.h"
#include "CalibrationData/ExtrinsicOrientation/ExtrinsicOrientation.h"
#include "CalibrationData/ExtrinsicOrientation/ExtrinsicOrientations.h"
#include "CalibrationData/ObjectPoints/ObjectPoints.h"
#include "CalibrationData/ImagePoints/Image.h"
#include "CalibrationData/ImagePoints/Images.h"
#include "CalibrationData/Constraints/Constraints.h"
#include "CalibrationData/ArucoDetection/ArucoDetection.h"
#include "CalibrationData/CalibrationData.h"
#include "CameraModel.h"

#include "colmap/scene/reconstruction.h"
#include "colmap/util/misc.h"


CalibrationData::CalibrationData(void)
{
	ptIntOr = 0;
	ptExtOr = 0;
	ptObjPts = 0;
	ptImPts = 0;
	ptConstraints = 0;
}


CalibrationData::~CalibrationData(void)
{
	if(ptIntOr!=0) delete ptIntOr;
	if(ptExtOr!=0) delete ptExtOr;
	if(ptObjPts!=0) delete ptObjPts;
	if(ptImPts!=0) delete ptImPts;
	if(ptConstraints!=0) delete ptConstraints;
}


/**
 * @brief Read Data from from the first calibration.
 */
bool CalibrationData::readDataFromFirstCalibration(std::string folderPath)
{
	printf("\nReading initial calibration data.\n");

	colmap::Reconstruction first_reconstruction_data;

	if ((colmap::ExistsFile(colmap::JoinPaths(folderPath, "cameras.bin")) &&
		colmap::ExistsFile(colmap::JoinPaths(folderPath, "images.bin")) &&
		colmap::ExistsFile(colmap::JoinPaths(folderPath, "points3D.bin"))) ||
		(colmap::ExistsFile(colmap::JoinPaths(folderPath, "cameras.txt")) &&
		colmap::ExistsFile(colmap::JoinPaths(folderPath, "images.txt")) &&
		colmap::ExistsFile(colmap::JoinPaths(folderPath, "points3D.txt"))))
	{
		first_reconstruction_data.Read(folderPath);
	}
	else
	{
		printf("some data files from the first calibration are missing.\n");
		return false;
	}

	// Read intrinsic parameters
	std::unordered_map<colmap::camera_t, class colmap::Camera> map_cameras = first_reconstruction_data.Cameras();
	if(map_cameras.empty())
	{
		printf("No intrinsic orientation data.\n");
		return false;
	}
	else
	{
		printf("Reading intrinsic parameters ...\n");
		ptIntOr = new IntrinsicOrientation();
		if(!ptIntOr->LoadIntrinsicOrientation(map_cameras)) { printf("Error when reading intrinsic orientation.\n"); delete ptIntOr; ptIntOr = 0; }
		if(ptIntOr!=0)ptIntOr->printParameter();
	}
	
	// Read extrinsic parameters and image coordinates
	std::unordered_map<colmap::image_t, class colmap::Image> map_images = first_reconstruction_data.Images();
	if(map_images.empty())
	{
		printf("No extrinsic parameters were found.\n");
		return false;
	}
	else
	{
		printf("Reading extrinsic parameters ...\n");
		ptExtOr = new ExtrinsicOrientations();
		if(!ptExtOr->LoadExtrinsicOrientations(map_images)) { printf("Error when reading extrinsic orientation.\n"); delete ptExtOr; ptExtOr = 0; }

		printf("Reading image coordinates ...\n");
		ptImPts = new Images();
		if(!ptImPts->LoadImageCoordinates(map_images)) { printf("Error when reading image coordinates.\n"); delete ptImPts; ptImPts = 0; }
	}


	// Read object coordinates
	std::unordered_map<colmap::point3D_t, class colmap::Point3D> map_point3D = first_reconstruction_data.Points3D();
	if(map_point3D.empty())
	{
		printf("No object points were found.\n");
		return false;
	}
	else
	{
		printf("Reading object points ...\n");
		ptObjPts = new ObjectPoints();
		if(!ptObjPts->LoadObjectPoints(map_point3D)) { printf("Error when reading object points.\n"); delete ptObjPts; ptObjPts = 0; }
	}

	printf("Finished reading parameters.\n");
	return true;
}

/**
 * @brief Read Data from the first calibration.
 */
bool CalibrationData::readConstaints(std::string dirConstraintsFile)
{
	printf("Reading constaints/distances ...\n");
	if(!boost::filesystem::exists(dirConstraintsFile))
	{
		printf("No constaints/distances data. File does not exist.\n");
		return false;
	}
	else
	{
		ptConstraints = new Constraints(dirConstraintsFile);
		if(!ptConstraints->ReadConstraints()) { printf("Error when parsing constaints/distances.\n"); delete ptConstraints; ptConstraints = 0; }
		else
		{
			for(std::vector<Constraint>::iterator it = ptConstraints->ConstraintList.begin(); it!=ptConstraints->ConstraintList.end(); it++)
				printf("\t%d\t%d\t%8.2f\t%8.2f\n",it->pointID_1,it->pointID_2,it->distance,it->sigma);
		}
	}

	printf("Finished reading constaints.\n");
	return true;
}

/**
 * @brief Read fixed values for the parameters f and B.
 */
bool CalibrationData::readFixedParameters(std::string dirFixedParametersFile)
{
	printf("Reading fixed parameters for f and B ...\n");
	if(!boost::filesystem::exists(dirFixedParametersFile))
	{
		printf("No fixed parameters data. File does not exist.\n");
		return false;
	}
	else
	{
		std::string line;
		// Open file in read mode
		std::ifstream fixedParametersFile(dirFixedParametersFile);

		if (!fixedParametersFile.is_open()) return false;
		else
		{
			// Read each line
			while (getline(fixedParametersFile, line))
			{
				if (line.empty() || line[0] == '#')
				{
					continue;
				}

				std::istringstream iss(line);
				iss >> this->f_fixed >> this->B_fixed;
			}
			fixedParametersFile.close();
		}
	}

	printf("Finished reading fixed parameters f and B.\n");
	printf("f = %f\n", this->f_fixed);
	printf("B = %f\n", this->B_fixed);
	return true;
}

/**
 * @brief Reduce the number of points to represent the 3D scene.
 */
void CalibrationData::ReduceNumberPoints(int nbPoints)
{
	std::vector<int> idsPointsKept;
	std::vector<int> idsPointsRemoved;
	auto* objPointsVect = &this->ptObjPts->objPoints;

    std::vector<int> allPositions(objPointsVect->size()) ;
    std::iota (std::begin(allPositions), std::end(allPositions), 0);
    
    auto rng = std::default_random_engine {};
    std::shuffle(std::begin(allPositions), std::end(allPositions), rng);

	std::vector<int> posToKeep(allPositions.begin(), allPositions.begin() + nbPoints);

	for(int i = 0; i < objPointsVect->size(); i++)
	{
		if(std::find(posToKeep.begin(), posToKeep.end(), i) != posToKeep.end())
		{
			idsPointsKept.push_back((*objPointsVect)[i]->pointID);
		}
		else
		{
			switch(calib_type)
			{
				case CALIBRATION_ARUCO:
					// Do not remove iDs of the Aruco markers
					if(std::find(this->ptConstraints->ConstraintsPointsIds.begin(), this->ptConstraints->ConstraintsPointsIds.end(), (*objPointsVect)[i]->pointID) == this->ptConstraints->ConstraintsPointsIds.end())
					{
						idsPointsRemoved.push_back((*objPointsVect)[i]->pointID);
					}
					break;
				case RECALIBRATION:
					idsPointsRemoved.push_back((*objPointsVect)[i]->pointID);
					break;
				default:
					break;
			}
		}
	}

	auto endObjPointsVect = std::remove_if(objPointsVect->begin(),
					objPointsVect->end(),
					[idsPointsRemoved](auto point) {
						return std::find(idsPointsRemoved.begin(), idsPointsRemoved.end(), point->pointID) != idsPointsRemoved.end();
					});
	objPointsVect->erase(endObjPointsVect, objPointsVect->end());

	// For the map
	for(unsigned int iIdMarker = 0; iIdMarker < idsPointsRemoved.size(); iIdMarker++)
	{
		this->ptObjPts->objPointsByPointID.erase(idsPointsRemoved[iIdMarker]);
	}

	// Remove ids of supressed points

	// For the vector
	for(unsigned int i = 0; i < this->ptImPts->images.size(); i++)
	{
		auto* imageCoordinatesIndex = &this->ptImPts->images[i]->imageCoordinates;
		auto endImCoord = std::remove_if(imageCoordinatesIndex->begin(),
						imageCoordinatesIndex->end(),
						[idsPointsRemoved](auto point) {
							return std::find(idsPointsRemoved.begin(), idsPointsRemoved.end(), point->pointID) != idsPointsRemoved.end();
						});
		imageCoordinatesIndex->erase(endImCoord, imageCoordinatesIndex->end());

		auto* imageCoordinatesInliersIndex = &this->ptImPts->images[i]->imageCoordinatesInliers;
		auto endImCoordIn = std::remove_if(imageCoordinatesInliersIndex->begin(),
						imageCoordinatesInliersIndex->end(),
						[idsPointsRemoved](auto point) {
							return std::find(idsPointsRemoved.begin(), idsPointsRemoved.end(), point->pointID) != idsPointsRemoved.end();
						});
		imageCoordinatesInliersIndex->erase(endImCoordIn, imageCoordinatesInliersIndex->end());

		auto*  imCoordByID = &this->ptImPts->images[i]->imageCoordinatesByPointID;
		for(unsigned int iIdMarker = 0; iIdMarker < idsPointsRemoved.size(); iIdMarker++)
		{
			if (imCoordByID->find(idsPointsRemoved[iIdMarker]) != imCoordByID->end())
				imCoordByID->erase(idsPointsRemoved[iIdMarker]);
		}
	}

	// For the map
	for(std::map<int,Image*>::iterator itImage = this->ptImPts->imagesByID.begin(); itImage!=this->ptImPts->imagesByID.end(); itImage++)
	{
			auto* imageCoordinatesByIdIndex = &itImage->second->imageCoordinates;
			auto endImCoordId = std::remove_if(imageCoordinatesByIdIndex->begin(),
							imageCoordinatesByIdIndex->end(),
							[idsPointsRemoved](auto point) {
								return std::find(idsPointsRemoved.begin(), idsPointsRemoved.end(), point->pointID) != idsPointsRemoved.end();
							});
			imageCoordinatesByIdIndex->erase(endImCoordId, imageCoordinatesByIdIndex->end());

			auto* imageCoordinatesInliersByIdIndex = &itImage->second->imageCoordinatesInliers;
			auto endImCoordInId = std::remove_if(imageCoordinatesInliersByIdIndex->begin(),
							imageCoordinatesInliersByIdIndex->end(),
							[idsPointsRemoved](auto point) {
								return std::find(idsPointsRemoved.begin(), idsPointsRemoved.end(), point->pointID) != idsPointsRemoved.end();
							});
			imageCoordinatesInliersByIdIndex->erase(endImCoordInId, imageCoordinatesInliersByIdIndex->end());

			auto*  imCoordByIDMap = &itImage->second->imageCoordinatesByPointID;
			for(unsigned int iIdMarker = 0; iIdMarker < idsPointsRemoved.size(); iIdMarker++)
			{
				if (imCoordByIDMap->find(idsPointsRemoved[iIdMarker]) != imCoordByIDMap->end())
					imCoordByIDMap->erase(idsPointsRemoved[iIdMarker]);
			}
	}
	
	printf("Reduced number of 3D points.\n");
}

/**
 * @brief Detect aruco markers in the image and add the points to the data.
 */
void CalibrationData::addArucoMarkersToData(int dictionaryMarkerNumber, std::string folderPathTotalFocus)
{
	ptArucoPts = new ArucoDetection(dictionaryMarkerNumber, folderPathTotalFocus);
	ptArucoPts->detectArucoMarkers(this->ptImPts->imagesIDOrder);

	for(unsigned int i = 0; i < this->ptImPts->images.size(); i++)
	{
		int imageIDMap = this->ptImPts->imagesIDOrder[i];

		// Remove ids of colmap that are the same as the aruco markers

		// For the vector
		auto* imageCoordinatesIndex = &this->ptImPts->images[i]->imageCoordinates;
		auto endImCoord = std::remove_if(imageCoordinatesIndex->begin(),
						imageCoordinatesIndex->end(),
						[this](auto point) {
							return std::find(ptArucoPts->markerIds.begin(), ptArucoPts->markerIds.end(), point->pointID) != ptArucoPts->markerIds.end();
						});
		imageCoordinatesIndex->erase(endImCoord, imageCoordinatesIndex->end());

		auto* imageCoordinatesInliersIndex = &this->ptImPts->images[i]->imageCoordinatesInliers;
		auto endImCoordIn = std::remove_if(imageCoordinatesInliersIndex->begin(),
						imageCoordinatesInliersIndex->end(),
						[this](auto point) {
							return std::find(ptArucoPts->markerIds.begin(), ptArucoPts->markerIds.end(), point->pointID) != ptArucoPts->markerIds.end();
						});
		imageCoordinatesInliersIndex->erase(endImCoordIn, imageCoordinatesInliersIndex->end());

		auto*  imCoordByID = &this->ptImPts->images[i]->imageCoordinatesByPointID;
		for(unsigned int iIdMarker = 0; iIdMarker < ptArucoPts->markerIds.size(); iIdMarker++)
		{
			if (imCoordByID->find(ptArucoPts->markerIds[iIdMarker]) != imCoordByID->end())
				imCoordByID->erase(ptArucoPts->markerIds[iIdMarker]);
		}

		// For the map
		auto* imageCoordinatesByIdIndex = &this->ptImPts->imagesByID[imageIDMap]->imageCoordinates;
		auto endImCoordId = std::remove_if(imageCoordinatesByIdIndex->begin(),
						imageCoordinatesByIdIndex->end(),
						[this](auto point) {
							return std::find(ptArucoPts->markerIds.begin(), ptArucoPts->markerIds.end(), point->pointID) != ptArucoPts->markerIds.end();
						});
		imageCoordinatesByIdIndex->erase(endImCoordId, imageCoordinatesByIdIndex->end());

		auto* imageCoordinatesInliersByIdIndex = &this->ptImPts->imagesByID[imageIDMap]->imageCoordinatesInliers;
		auto endImCoordInId = std::remove_if(imageCoordinatesInliersByIdIndex->begin(),
						imageCoordinatesInliersByIdIndex->end(),
						[this](auto point) {
							return std::find(ptArucoPts->markerIds.begin(), ptArucoPts->markerIds.end(), point->pointID) != ptArucoPts->markerIds.end();
						});
		imageCoordinatesInliersByIdIndex->erase(endImCoordInId, imageCoordinatesInliersByIdIndex->end());

		auto*  imCoordByIDMap = &this->ptImPts->imagesByID[imageIDMap]->imageCoordinatesByPointID;
		for(unsigned int iIdMarker = 0; iIdMarker < ptArucoPts->markerIds.size(); iIdMarker++)
		{
			if (imCoordByIDMap->find(ptArucoPts->markerIds[iIdMarker]) != imCoordByIDMap->end())
				imCoordByIDMap->erase(ptArucoPts->markerIds[iIdMarker]);
		}

		// Concatenate the points of the aruco markers with those from colmap
		imageCoordinatesIndex->insert(imageCoordinatesIndex->begin(),
			this->ptArucoPts->imagesAruco[i]->imageCoordinates.begin(),
			this->ptArucoPts->imagesAruco[i]->imageCoordinates.end());

		imageCoordinatesInliersIndex->insert(imageCoordinatesInliersIndex->begin(),
			this->ptArucoPts->imagesAruco[i]->imageCoordinatesInliers.begin(),
			this->ptArucoPts->imagesAruco[i]->imageCoordinatesInliers.end());

		this->ptImPts->images[i]->imageCoordinatesByPointID.insert(
			this->ptArucoPts->imagesAruco[i]->imageCoordinatesByPointID.begin(),
			this->ptArucoPts->imagesAruco[i]->imageCoordinatesByPointID.end());
	}

	// Remove the 3D points with the same ids as those of the aruco markers

	// For the vector
	auto* objPointsVect = &this->ptObjPts->objPoints;
	auto endObjPointsVect = std::remove_if(objPointsVect->begin(),
					objPointsVect->end(),
					[this](auto point) {
						return std::find(ptArucoPts->markerIds.begin(), ptArucoPts->markerIds.end(), point->pointID) != ptArucoPts->markerIds.end();
					});
	objPointsVect->erase(endObjPointsVect, objPointsVect->end());

	// For the map
	for(unsigned int iIdMarker = 0; iIdMarker < ptArucoPts->markerIds.size(); iIdMarker++)
	{
		this->ptObjPts->objPointsByPointID.erase(ptArucoPts->markerIds[iIdMarker]);
	}

	// Add 3D points for the aruco markers by taking the 3D coordinates of the nearest projected 2D point in the image

	// For each aruco marker
	for(std::vector<int>::iterator iterArucoId = ptArucoPts->markerIds.begin(); iterArucoId < ptArucoPts->markerIds.end(); iterArucoId++)
	{
		int ArucoPointID = *iterArucoId;
		int nRays = 0;

		// Go through all images
		for(unsigned int imagePos = 0; imagePos < this->ptImPts->images.size(); imagePos++)
		{
			// If the aruco marker was detected in the image
			if(this->ptImPts->images[imagePos]->imageCoordinatesByPointID.count(ArucoPointID))
			{
				nRays++;

				// If the aruco marker is not yet in the vector of the 3D points (use only the first image where the aruco is in to give him the 3D coordinates)
				if(this->ptObjPts->objPointsByPointID.count(ArucoPointID) == 0)
				{
					int nearestPointId;
					double distancePoints2 = DBL_MAX;
					Eigen::Vector2d Coordinate2DAruco = this->ptImPts->images[imagePos]->imageCoordinatesByPointID[ArucoPointID]->coordinate;
					Eigen::Vector2d Coordinate2DColmap;

					// For each point generated by colmap
					for(std::vector<ImageCoordinate*>::iterator iterColmapPoint = this->ptImPts->images[imagePos]->imageCoordinatesInliers.begin()+ptArucoPts->NumberInliersAruco[imagePos];
						iterColmapPoint < this->ptImPts->images[imagePos]->imageCoordinatesInliers.end();
						iterColmapPoint++)
					{
						Coordinate2DColmap = (*iterColmapPoint)->coordinate;

						// If a new point is closer to the aruco marker, then save its id
						if((((Coordinate2DColmap(0) - Coordinate2DAruco(0)) * (Coordinate2DColmap(0) - Coordinate2DAruco(0))) +
							((Coordinate2DColmap(1) - Coordinate2DAruco(1)) * (Coordinate2DColmap(1) - Coordinate2DAruco(1))) < distancePoints2) && (ArucoPointID != (*iterColmapPoint)->pointID))
							{
								distancePoints2 = ((Coordinate2DColmap(0) - Coordinate2DAruco(0)) * (Coordinate2DColmap(0) - Coordinate2DAruco(0))) +
									((Coordinate2DColmap(1) - Coordinate2DAruco(1)) * (Coordinate2DColmap(1) - Coordinate2DAruco(1)));
								nearestPointId = (*iterColmapPoint)->pointID;
							}
					}

					// Take the 3D coordinates of the neareast 2D point in the image for the initiaization of the position of the aruco marker in space
					Eigen::Vector3d coordinateArucoPoint = this->ptObjPts->objPointsByPointID[nearestPointId]->coordinate;
					ObjectPoint* objPt = new ObjectPoint(ArucoPointID, coordinateArucoPoint, -1.0, nRays);
					this->ptObjPts->objPointsByPointID[ArucoPointID] = objPt;
					this->ptObjPts->objPoints.push_back(objPt);
				}
			}
		}
	}
	printf("Added markers to data.");
}

/*
 * @brief Scale the point cloud and the extrinsic orientation using the constraints.
 */
void CalibrationData::scaleData()
{
	int refPointID_1 = this->ptConstraints->ConstraintList[0].pointID_1;
	int refPointID_2 = this->ptConstraints->ConstraintList[0].pointID_2;
	double realDistance = this->ptConstraints->ConstraintList[0].distance;

	Eigen::Vector3d coordPcPoint1 = this->ptObjPts->objPointsByPointID[refPointID_1]->coordinate;
	Eigen::Vector3d coordPcPoint2 = this->ptObjPts->objPointsByPointID[refPointID_2]->coordinate;
	double pcDistance = sqrt((coordPcPoint2(0) - coordPcPoint1(0)) * (coordPcPoint2(0) - coordPcPoint1(0)) +
								(coordPcPoint2(1) - coordPcPoint1(1)) * (coordPcPoint2(1) - coordPcPoint1(1)) +
								(coordPcPoint2(2) - coordPcPoint1(2)) * (coordPcPoint2(2) - coordPcPoint1(2)));
	
	double scaleFactor = realDistance / pcDistance;

	// Scale extrinsic orientations
	for(std::vector<ExtrinsicOrientation*>::iterator iterExtOrVec = this->ptExtOr->extOrientations.begin(); iterExtOrVec < this->ptExtOr->extOrientations.end(); iterExtOrVec++)
	{
		(*iterExtOrVec)->tranVec *= scaleFactor;
		(*iterExtOrVec)->worldToCameraMatrix.block<3,1>(0,3) *= scaleFactor;
	}

	// Scale object points
	for(std::vector<ObjectPoint*>::iterator iterObjPtsVec = this->ptObjPts->objPoints.begin(); iterObjPtsVec < this->ptObjPts->objPoints.end(); iterObjPtsVec++)
	{
		(*iterObjPtsVec)->coordinate *= scaleFactor;
	}
	printf("Data scaled.");
}

/*
 * @brief Remap point IDs and get transformations.
 */
bool CalibrationData::getCalibDataCV(std::vector<Eigen::Vector3d> &worldCoordinates, std::vector<frame> &frames)
{
	// Check if any data was read
	if(ptObjPts == 0 || ptImPts == 0 || ptExtOr == 0) return false;

	// Store all object points in the worldCoordinate vector
	worldCoordinates.resize(this->ptObjPts->objPoints.size());

	for(unsigned int i=0; i<worldCoordinates.size();i++)
	{
		this->pointIdMap[this->ptObjPts->objPoints[i]->pointID] = i;
		this->pointIdMapFromNewToColmap[i] = this->ptObjPts->objPoints[i]->pointID;
		memcpy(worldCoordinates[i].data(),this->ptObjPts->objPoints[i]->coordinate.data(),3*sizeof(double));
	}

	// Store image points with corresponding extrinsic orientation
	frames.resize(this->ptImPts->images.size());
	for(unsigned int i=0; i<frames.size(); i++)
	{
		unsigned int nImageCoordinates = this->ptImPts->images[i]->imageCoordinatesInliers.size();
		frames[i].imageCoordinates.resize(nImageCoordinates);
		int imageID = this->ptImPts->images[i]->id;
		frames[i].id = imageID;
		for(unsigned int ii=0; ii<nImageCoordinates; ii++)
		{
			int oldId = this->ptImPts->images[i]->imageCoordinatesInliers[ii]->pointID;
			int newId = this->pointIdMap[oldId];

			// Converting points into pixel coordinates
			Eigen::Vector2d coordinate = this->ptImPts->images[i]->imageCoordinatesInliers[ii]->coordinate;

			// Store pixel coordinates in vector
			frames[i].imageCoordinates[ii] = coordinate;
			frames[i].imageCoordinatesByID[newId] = &frames[i].imageCoordinates[ii];
			frames[i].objectCoordinatesByID[ii] = &worldCoordinates[newId];

			// Convert transform form world to camera coordinates
			frames[i].worldToCam = this->ptExtOr->extOrientationsByImageID[imageID]->worldToCameraMatrix;
			frames[i].transVector = this->ptExtOr->extOrientationsByImageID[imageID]->tranVec;
			frames[i].rotationAngles = this->ptExtOr->extOrientationsByImageID[imageID]->rotQuat.toRotationMatrix().eulerAngles(0, 1, 2);
		}
	}

	printf("Remaped point IDs and got transformation.");

	return true;
}

/*
 * @brief Returns a list of the constraints coming from the file.
 */
bool CalibrationData::getConstraintsList(std::vector<Constraint> &constraintList)
{
	constraintList.clear();
	constraintList.reserve(ptConstraints->ConstraintList.size());
	for(std::vector<Constraint>::iterator it = ptConstraints->ConstraintList.begin(); it!=ptConstraints->ConstraintList.end(); it++)
	{
		constraintList.push_back(*it);
		constraintList.back().pointID_1 = this->pointIdMap[it->pointID_1];
		constraintList.back().pointID_2 = this->pointIdMap[it->pointID_2];
	}

	printf("Got constraint data.");
	return true;
}

/**
 * @brief Returns data in Computer Vision conventional format.
 */
bool CalibrationData::getIntrinsicParamCV(double &f, Eigen::Vector2i &imageSize, Eigen::Vector2d &c, Eigen::Vector2d &k, Eigen::Vector2d &p)
{
	if(this->ptIntOr != 0)
	{
		this->ptIntOr->getIntrinsicParam(f, imageSize, c, k, p);
		printf("Got intrinsic parameters.");
		return true;
	}

	else return false;
}

int CalibrationData::getCorrespondingCOLMAPID(int id)
{
	return this->pointIdMapFromNewToColmap[id];
}

/**
 * @brief Returns fixed parameters f and B for recalibration.
 */
bool CalibrationData::getFixedParameters(double &f, double &B)
{
	f = this->f_fixed;
	B = this->B_fixed;

	return false;
}
