/**
 * @brief Definition of the class CameraCalibration.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CameraCalibration.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>
#include <string>
#include <fstream>
#include "ceres/ceres.h"
#include "Eigen/Dense"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/thread.hpp"
#include "boost/format.hpp"
#include "opencv2/opencv.hpp"

#include "thirdparty/pugixml-1.7/src/pugixml.hpp"

#include "Utility/myUtility/myUtility.h"
#include "Utility/FileManagement.h"
#include "Utility/Settings.h"
#include "CalibrationData/CalibrationData.h"
#include "CalibrationData/Constraints/Constraints.h"
#include "BundleAdjustment/BundleAdjustment.h"
#include "MicroLensGrid/MicroLensGrid.h"
#include "MicroLensGrid/MicroLens.h"
#include "MicroLensGrid/EpiPolarLine.h"
#include "CameraModel.h"
#include "CameraCalibration.h"

#include "ColmapReconstructor/ColmapReconstructor.h"


/**
 * @brief Constructor.
 *
 * In this constructor all needed parameters are read from the yaml file.
 */
CameraCalibration::CameraCalibration(std::string type_of_calibration, std::string settingsFile, std::string constraintsFile)
{
	dirSettingsFile = settingsFile;
	dirConstraintsFile = constraintsFile;

	initialization();

	if(!calibSettings->isDataValid())
	{
		printf("Input data in the yaml file are not valid.\nAborting.\n");
		return;
	}

	// Get data from yaml file
	pixelSize = calibSettings->getPixelSize();
	rawWidth = calibSettings->getRawImageWidth();
	rawHeight = calibSettings->getRawImageHeight();

	dirTotalFocusImages = calibSettings->getDirTotalFocusImages();
	dirMlCalibFile = calibSettings->getDirMlCalibFile();
	dirDepthData = calibSettings->getDirDepthData();
	
	n3DPoints = calibSettings->getNumberPoints();
	markerDict = calibSettings->getMarkerDictionary();

	nRadialDistParam = calibSettings->getNRadialDistParam();
	tangentialDistParam = calibSettings->getTangentialDistParam();
	mlCenterAdjustment = calibSettings->getMlCenterAdjustment();
	refinePoses = calibSettings->getRefinePoses();
	refine3Dpoints = calibSettings->getRefine3Dpoints();
	useRobustCostFunction = calibSettings->getUseRobustCostFunction();

	calibSettings->displaySettings();

	// Perform calibration
	if(type_of_calibration == "calib_marker")
	{
		calibData->calib_type = CALIBRATION_ARUCO;
		if(!doCalibration_aruco_calib())
		{
			printf("\nCalibration failed.\n");
			return;
		}
	}
	else if(type_of_calibration == "recalib")
	{
		calibData->calib_type = RECALIBRATION;
		if(!doCalibration_recalib())
		{
			printf("\nCalibration failed.\n");
			return;
		}
	}
	else
	{
		printf("\nThe given mode is not valid (%s).\n", type_of_calibration.c_str());
		myUtility::displayHelp();
		return;
	}
	


	// Print results
	showResults();

	// Store results
	if(myUtility::askYesNo("Do you want to store the results?"))
	{
		printf("Please enter a path where the results shall be stored:\n");
		std::cin >> dirResults;
		// Check if path exists or can be created
		while(!fileManagement::Exists(dirResults))
		{
			if(fileManagement::CreateDir(dirResults))
				break;
			printf("Was not able to create directory. Please enter again:\n");
			std::cin >> dirResults;
		}
		printf("\n");

		// Creating a unique date and time dependened extention
		std::string sUniqueExtention = myUtility::getTimeStampString();
		this->dirResults += "/Calibration_Results_" + sUniqueExtention;
		// Create folder
		if(!fileManagement::CreateDir(this->dirResults))
		{
			printf("Was not able to store results.\n");
			return;
		}

		if(!this->storeResults())
		{
			printf("Faild to store results.\n");
			return;
		}
	}
}

/**
 * @brief Destructor.
 *
 * Delete created pointers.
 */
CameraCalibration::~CameraCalibration()
{
	delete calibData;
	delete calibSettings;
	delete firstEstimateColmap;

	if(mlGrid != NULL) delete mlGrid;
	if(init_camera != NULL) delete init_camera;
	if(camera != NULL) delete camera;
	if(views != NULL) delete views;
	if(init_views != NULL) delete init_views;

	for(int i=0;i<this->epiLineWeb.size(); i++)
	{
		for(int ii=0; ii<this->epiLineWeb[i]->size(); ii++)
			delete this->epiLineWeb[i]->at(ii);
		delete this->epiLineWeb[i];
	}
}

/**
 * @brief Do initialization of some parameters.
 */
void CameraCalibration::initialization()
{
	printf(	"*******************************************************************************\n"
			"***   LiFCal: Online Light Field Camera Calibration via Bundle Adjustment   ***\n"
			"*******************************************************************************\n"
			"   Authors: Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller\n"
			"   Universities: Technical University of Munich\n"
			"                 Karlsruhe University of Applied Sciences\n"
			"   Version: 1.0.0\n"
			"   Date: 2024\n\n"
			);

	// Default values for the parameters
	n3DPoints = 500;
	markerDict = 10;
	nRadialDistParam = 2;
	tangentialDistParam = true;
	mlCenterAdjustment = true;
	refinePoses = true;
	refine3Dpoints = true;
	useRobustCostFunction = true;

	calibData = new CalibrationData();
	firstEstimateColmap = new ColmapReconstructor();

    // Check settings file
    cv::FileStorage fsSettings(dirSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << dirSettingsFile << std::endl;
       exit(-1);
    }

	calibSettings = new CalibrationSettings();

	calibSettings->setDefaultValues(n3DPoints,
									markerDict,
									nRadialDistParam,
									tangentialDistParam,
									mlCenterAdjustment,
									refinePoses,
									refine3Dpoints,
									useRobustCostFunction);
	
	calibSettings->parseFile(dirSettingsFile);

	mlGrid = NULL;
	init_camera= NULL;
	camera = NULL;
	init_views = NULL;
	views = NULL;
}

/**
 * @brief Perform camera calibration based bundle adjustment in the micro images for a calibration with Aruco markers.
 */
bool CameraCalibration::doCalibration_aruco_calib()
{
	// Perform fisrt estimate for the calibration with colmap
	if(!firstEstimateColmap->reconstructionColmap(dirTotalFocusImages, dirFirstCalibrationData))
	{
		printf("First calibration with colmap failed.");
		return false;
	}

	// Read data from first calibration
	if(!calibData->readDataFromFirstCalibration(dirFirstCalibrationData))
		return false;
	
	// Read constraints from the file
	if(!calibData->readConstaints(dirConstraintsFile))
		return false;

	calibData->ReduceNumberPoints(n3DPoints);

	calibData->addArucoMarkersToData(markerDict, dirTotalFocusImages);

	calibData->scaleData();

	calibData->getCalibDataCV(p3d_w, frames);
	
	calibData->getConstraintsList(constraintList);

	calibData->getIntrinsicParamCV(fPH_init, imageSize, c_init, radialDist_init, tangentialDist_init);

	// Calculate scaling facotr between virtual image and raw image
	depth_to_raw_im_scale = rawWidth/imageSize[0];
	if(depth_to_raw_im_scale < rawHeight/imageSize[1])
		depth_to_raw_im_scale = rawHeight/imageSize[1];

	pixelSize_totFoc = depth_to_raw_im_scale * pixelSize;

	// Read depth data
	if(!readDepthData())
		return false;

	// Read micro lens grid
	mlGrid = new MicroLensGrid();
	if(!mlGrid->readInGrid(this->dirMlCalibFile, rawWidth, rawHeight))
		return false;
	mlGrid->defineMlMaps();
	defineEpiPolarLines();

	// Project points to micro lenses
	if(!projectPointsToRawImage())
		return false;

	// Estimate initial parameters for bL0 and B
	if(!initPlenopticParameters())
		return false;

	// Do non linear optimization
	if(!performBundleAdjustment())
		return false;

	return true;
}

/**
 * @brief Perform camera calibration based bundle adjustment in the micro images for a recalibration with fixed values for f an B.
 */
bool CameraCalibration::doCalibration_recalib()
{
	// Perform fisrt estimate for the calibration with colmap
	if(!firstEstimateColmap->reconstructionColmap(dirTotalFocusImages, dirFirstCalibrationData))
	{
		printf("First calibration with colmap failed.");
		return false;
	}

	// Read data from first calibration
	if(!calibData->readDataFromFirstCalibration(dirFirstCalibrationData))
		return false;

	// Read constraints from the file
	if(!calibData->readFixedParameters(dirConstraintsFile))
		return false;

	calibData->ReduceNumberPoints(n3DPoints);

	calibData->getCalibDataCV(p3d_w, frames);

	calibData->getIntrinsicParamCV(fPH_init, imageSize, c_init, radialDist_init, tangentialDist_init);

	// Calculate scaling facotr between virtual image and raw image
	depth_to_raw_im_scale = rawWidth/imageSize[0];
	if(depth_to_raw_im_scale < rawHeight/imageSize[1])
		depth_to_raw_im_scale = rawHeight/imageSize[1];

	pixelSize_totFoc = depth_to_raw_im_scale * pixelSize;

	// Read depth data
	if(!readDepthData())
		return false;

	// Read micro lens grid
	mlGrid = new MicroLensGrid();
	if(!mlGrid->readInGrid(this->dirMlCalibFile, rawWidth, rawHeight))
		return false;
	mlGrid->defineMlMaps();
	defineEpiPolarLines();

	// Project points to micro lenses
	if(!projectPointsToRawImage())
		return false;

	// Estimate initial parameters for bL0 and B
	if(!initPlenopticParametersRecalibration())
		return false;

	// Do non linear optimization
	if(!performBundleAdjustment())
		return false;

	return true;
}


/**
 * @brief Read virtual depth from files.
 */
bool CameraCalibration::readDepthData()
{
	printf("\nReading depth data.\n");

	char fileName[256];

	// Get file list
	std::vector<std::string> vFileList;
	fileManagement::GetFileList(dirDepthData,".png", vFileList);

	cv::Mat depthImageTmp;

	virtualDepthValues.resize(frames.size());
	std::vector<std::vector<double>>::iterator itVdepthFrame;
	std::vector<frame>::iterator itFrame;
	for(itFrame = frames.begin(), itVdepthFrame = virtualDepthValues.begin(); itFrame != frames.end(); itFrame++, itVdepthFrame++)
	{
		int frameID = itFrame->id;

		depthImageTmp = imread(dirDepthData + "/" + vFileList[frameID-1], cv::IMREAD_UNCHANGED);

		if(depthImageTmp.cols*depthImageTmp.rows == 0)
		{
			printf("Could not open depth image (%s). Exit!\n", fileName);
			return false;
		}
		if(depthImageTmp.rows != this->imageSize[1] || depthImageTmp.cols != this->imageSize[0])
		{
			printf("ERROR: Wrong depth image size. Exit!\n");
			return false;
		}

		itVdepthFrame->resize(itFrame->imageCoordinates.size());
		std::vector<Eigen::Vector2d>::iterator itPt;
		std::vector<double>::iterator itVdepth;
		for(itPt = itFrame->imageCoordinates.begin(), itVdepth = itVdepthFrame->begin(); itPt != itFrame->imageCoordinates.end(); itPt++, itVdepth++)
		{
			Eigen::Vector2i coordinate((*itPt)[0]+0.5,(*itPt)[1]+0.5);
			int value = depthImageTmp.at<unsigned short>(coordinate[1],coordinate[0]);

			double iVdepth = 0;
			if(value>0)
			{
				iVdepth = (double)value/65535.0;
				iVdepth = 1.0-iVdepth;

				if(iVdepth<=0.5 && iVdepth>0.0)
				{
					// Is a valid virtual depth value (store and continue)
					(*itVdepth) = 1.0/iVdepth;
					continue;
				}
			}

			printf("Interpolate virtual depth value virtual depth value.\n");			
			for(int dist=1;dist<50;dist++)
			{
				int numValues = 0;
				double iVdepthSum = 0;
				for(int x = coordinate[0]-dist;x<=coordinate[0]+dist;x++)
				{
					if(x<0) x=0;
					if(x>=depthImageTmp.cols) break;
					for(int y = coordinate[1]-dist;y<=coordinate[1]+dist;y++)
					{
						if(y<0) y=0;
						if(y>=depthImageTmp.rows) break;

						int value = depthImageTmp.at<unsigned short>(y,x);
						double iVdepth = 0;
						if(value>0)
						{
							iVdepth = (double)value/65535.0;
							iVdepth = 1.0-iVdepth;

							if(iVdepth<=0.5 && iVdepth>0.0)
							{
								// Is a valid virtual depth value (store and continue)
								numValues++;
								iVdepthSum += iVdepth;
							}
						}


					}
				}
				if(numValues>=10)
				{
					(*itVdepth) = (double)numValues/iVdepthSum;
					break;
				}

				if(dist == 49)
				{
					printf("Interpolation failed.\n");
					(*itVdepth) = -1;
				}
			}
		}
	}
	return true;
}

/**
 * @brief Estimate initial camera parameters.
 */
bool CameraCalibration::initPlenopticParameters()
{
	printf("\nEstimate initial parameters for B and bL0 and set fL.\n");

	fL_init = fPH_init*pixelSize_totFoc;

	int numInliers = 0;
	for(auto v: frames)
		numInliers += v.imageCoordinates.size();

	// Linear problem: bL=v.B+bL0 -> b=ax 
	Eigen::MatrixXd a = Eigen::MatrixXd::Constant(numInliers, 2, 1); 
	Eigen::VectorXd b (numInliers);
	Eigen::Vector2d x;

	int offset = 0;	
	for(unsigned int i=0 ; i<frames.size() ; ++i)
	{
		for(unsigned int p=0 ; p<frames[i].imageCoordinates.size() ; ++p)
		{	
			// Initiate a = [v 1]
			a(p + offset,0) = this->virtualDepthValues[i][p];
			// Initiate b = [bL]
			Eigen::Vector4d p_c_tmp = frames[i].worldToCam * frames[i].objectCoordinatesByID[p]->homogeneous();
			b[p + offset] = (fL_init * p_c_tmp.z() ) / ( p_c_tmp.z() - fL_init);

			if((this->virtualDepthValues[i][p] < 2) || (b[p + offset] < 0))
			{
				a(p + offset,0) = 0;
				a(p + offset,1) = 0;
				b[p + offset] = 0;
			}
		}
		offset += frames[i].imageCoordinates.size();
	}

	x = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b); 
	B_init = x[0]; 	// Distance between the main lens and the MLA
	bL0_init = x[1];	// Distance between the MLA and the sensor
	printf("\t-> fL_init = %1.5f, B_init = %1.5f, bL0_init = %1.5f\n", fL_init, B_init, bL0_init);

	return true;
}

/**
 * @brief Initialize plenoptic camera parameters.
 */
bool CameraCalibration::initPlenopticParametersRecalibration()
{
	printf("\nInitialize the parameters for B and bL0 and set fL.\n");

	calibData->getFixedParameters(fL_init, B_init);
	bL0_init = fL_init - 2 * B_init;
	printf("\t-> fL_init = %1.5f, B_init = %1.5f, bL0_init = %1.5f\n", fL_init, B_init, bL0_init);

	return true;
}

/**
 * @brief Define web of epipolar lines / baselines.
 * 
 * Epipolar lines are defined similar to a spider web around the pixel of interest.
 * Baselines are sorted ascending by their length.
 * All used epi-polar lines have an angle phi with -pi/2 < phi <= pi/2.
 */
void CameraCalibration::defineEpiPolarLines()
{
	// Maximum base linse distance (in pixels)
	float maxDist = this->mlGrid->lensDiameter*10;
	
	// Epi-polar lines to next neighboring micro lenses
	EpiPolarLine epl_0 = EpiPolarLine(1,0,this->mlGrid->lensDiameter,2.0f);
	EpiPolarLine epl_1 = EpiPolarLine(0.5,sqrt(0.75),this->mlGrid->lensDiameter,2.0f);
	EpiPolarLine _epl_1 = EpiPolarLine(-0.5,-sqrt(0.75),this->mlGrid->lensDiameter,2.0f);
	EpiPolarLine epl_2 = EpiPolarLine(0.5,-sqrt(0.75),this->mlGrid->lensDiameter,2.0f);
	EpiPolarLine _epl_2 = EpiPolarLine(-0.5,sqrt(0.75),this->mlGrid->lensDiameter,2.0f);

	if(mlGrid->isRotationOnGrid())
	{
		double cosAlpha = cos(mlGrid->rotation);
		double sinAlpha = sin(mlGrid->rotation);

		double epx, epy;

		epx = epl_0.epiLine[0], epy = epl_0.epiLine[1];
		epl_0.epiLine[0] = epx*cosAlpha+epy*sinAlpha;
		epl_0.epiLine[1] = -epx*sinAlpha+epy*cosAlpha;

		epx = epl_1.epiLine[0], epy = epl_1.epiLine[1];
		epl_1.epiLine[0] = epx*cosAlpha+epy*sinAlpha;
		epl_1.epiLine[1] = -epx*sinAlpha+epy*cosAlpha;

		epx = _epl_1.epiLine[0], epy = _epl_1.epiLine[1];
		_epl_1.epiLine[0] = epx*cosAlpha+epy*sinAlpha;
		_epl_1.epiLine[1] = -epx*sinAlpha+epy*cosAlpha;

		epx = epl_2.epiLine[0], epy = epl_2.epiLine[1];
		epl_2.epiLine[0] = epx*cosAlpha+epy*sinAlpha;
		epl_2.epiLine[1] = -epx*sinAlpha+epy*cosAlpha;

		epx = _epl_2.epiLine[0], epy = _epl_2.epiLine[1];
		_epl_2.epiLine[0] = epx*cosAlpha+epy*sinAlpha;
		_epl_2.epiLine[1] = -epx*sinAlpha+epy*cosAlpha;
	}


	int i=0;
	std::vector<EpiPolarLine*> epi_Lines;
	epi_Lines.push_back(new EpiPolarLine(epl_1));
	epi_Lines.push_back(new EpiPolarLine(epl_2));
	EpiPolarLine* pEplNew_1, *pEplNew_2;
	while(epi_Lines.back()->baseLineDist<maxDist)
	{
		if(i%2==0)
		{
			pEplNew_1 = epi_Lines[i*2]->add(_epl_2);
			pEplNew_2 = epi_Lines[i*2+1]->add(_epl_1);
		}
		else
		{
			pEplNew_1 = epi_Lines[i*2]->add(epl_1);
			pEplNew_2 = epi_Lines[i*2+1]->add(epl_2);
		}
		epi_Lines.push_back(pEplNew_1);
		epi_Lines.push_back(pEplNew_2);
		
		i++;
	}
	epi_Lines.push_back(new EpiPolarLine(epl_0));
	int initLenght = epi_Lines.size();
	for(int i=0;i<initLenght;i++)
	{
		EpiPolarLine* pLastEpiLine = epi_Lines[i];
		while(pLastEpiLine->baseLineDist<maxDist)
		{
			epi_Lines.push_back(pLastEpiLine->add(epl_0));
			pLastEpiLine = epi_Lines.back();
		}
	}
	
	// Sort epi-polar Lines
	// Initialize sorted web by the first epi polar line in the list
	epiLineWeb.push_back(new std::vector<EpiPolarLine*>());
	epiLineWeb[0]->push_back(epi_Lines[0]);

	bool isSmaller = false, isEqual = false;
	for(int i=1;i<epi_Lines.size();i++)
	{
		// Epi-polar lines pointing against the y-unit vector and which are out of range are ingnored
		if((epi_Lines[i]->epiLine[1]==-1.0f)||(epi_Lines[i]->baseLineDist>maxDist))
		{
			delete epi_Lines[i];
			epi_Lines[i] = NULL;
			continue;
		}
		isSmaller = false;
		isEqual = false;
		int ii;
		for(ii=0;(ii<epiLineWeb.size())&&(!(isSmaller||isEqual)) ;ii++)
		{
			if(((float)epiLineWeb.at(ii)->at(0)->baseLineDist) == ((float)epi_Lines[i]->baseLineDist)) isEqual = true;
			else if(epiLineWeb.at(ii)->at(0)->baseLineDist > epi_Lines[i]->baseLineDist) isSmaller=true;
		}

		if(isSmaller)
		{
			epiLineWeb.insert(epiLineWeb.begin()+ii-1,new std::vector<EpiPolarLine*>());
			epiLineWeb[ii-1]->push_back(epi_Lines[i]);
		}
		else if(isEqual)
			epiLineWeb[ii-1]->push_back(epi_Lines[i]);
		else
		{
			epiLineWeb.push_back(new std::vector<EpiPolarLine*>());
			epiLineWeb.back()->push_back(epi_Lines[i]);
		}
	}
	epi_Lines.clear();
}


/**
 * @brief Projecting points from virtual image to micro images.
 */
bool CameraCalibration::projectPointsToRawImage()
{
	// Values are set based on the inverse virutal depth
	bool lensTypeValid[3];

	for(unsigned int iFrame=0; iFrame<frames.size(); iFrame++)
	{
		frames[iFrame].rawImageCoordinates.clear();
		frames[iFrame].microLensCenter.clear();
		frames[iFrame].objectCoordinatesByRawID.clear();

		for(unsigned int p=0 ; p<frames[iFrame].imageCoordinates.size() ; p++)
		{
			// Get virtual depth value
			float vdepth = this->virtualDepthValues[iFrame][p];
			if(!(vdepth>2.0 && vdepth<20.0)) continue;

			float x = frames[iFrame].imageCoordinates[p][0];
			float y = frames[iFrame].imageCoordinates[p][1];

			// Define search range for corresponding micro lenses
			float radius = mlGrid->lensDiameter*0.5f*vdepth + 2.0f;
			float radius_2 = radius*radius;

			// Upsample coordinates
			float xUps_flt = ((float)depth_to_raw_im_scale)*(x+0.5f)-0.5f;
			float yUps_flt = ((float)depth_to_raw_im_scale)*(y+0.5f)-0.5f;
			// The integer values are just needed to select the micro lenses
			int xUps_int = xUps_flt+0.5f;
			if(xUps_int>=rawWidth) xUps_int = rawWidth-1;
			int yUps_int = yUps_flt+0.5f;
			if(yUps_int>=rawHeight) yUps_int = rawHeight-1;
			int idxUps = xUps_int+rawWidth*yUps_int;

			// Get closest mico lens
			MicroLens *ml = mlGrid->mapNextMl[idxUps];
			if(ml == NULL)
			{
				printf("Error.\n");
				continue;
			}

			// Get microlens center of next micro lens
			float centerX_next = ml->centerX;
			float centerY_next = ml->centerY;

			// Calculate distance to the next micro lens
			float distCenterX_next = centerX_next-xUps_flt;
			float distCenterY_next = centerY_next-yUps_flt;
			float distCenter_2_next = distCenterX_next*distCenterX_next+distCenterY_next*distCenterY_next;
			if(distCenter_2_next>radius_2) continue;

			// Collect all micro lenses within the radius
			std::vector<MicroLens*> microLenses;

			microLenses.push_back(ml);
			
			for(std::vector<std::vector<EpiPolarLine*>*>::iterator itDist = this->epiLineWeb.begin(); itDist !=this->epiLineWeb.end(); itDist++)
			{
				// Check if base line distance is within the radius
				if((*itDist)->at(0)->baseLineDist > radius) break;

				for(std::vector<EpiPolarLine*>::iterator itBase = (*itDist)->begin(); itBase!=(*itDist)->end(); itBase++)
				{
					for(int iEpl=0; iEpl<2; iEpl++)
					{
						float baseLineDist = (*itBase)->baseLineDist;
						float epx = (*itBase)->epiLine[0];
						float epy = (*itBase)->epiLine[1];
						if(iEpl==0){ 
							epx = (*itBase)->epiLine[0];
							epy = (*itBase)->epiLine[1];
						}
						else {
							epx = -(*itBase)->epiLine[0];
							epy = -(*itBase)->epiLine[1];
						}

						float centerX = centerX_next + baseLineDist*epx;
						float centerY = centerY_next + baseLineDist*epy;

						float distCenterX = centerX-xUps_flt;
						float distCenterY = centerY-yUps_flt;

						float distCenter_2 = distCenterX*distCenterX + distCenterY*distCenterY;
						if(distCenter_2>radius_2) continue;

						int centerX_int = centerX+0.5;
						int centerY_int = centerY+0.5;
						if(centerX_int < 0) centerX_int=0;
						if(centerX_int>=rawWidth) centerX_int = rawWidth-1;
						if(centerY_int < 0) centerY_int=0;
						if(centerY_int>=rawHeight) centerY_int = rawHeight-1;

						int idxMl = centerX_int+centerY_int*rawWidth;
						ml = mlGrid->mapMlPointer[idxMl];

						if(ml == NULL) continue;

						microLenses.push_back(ml);
					}
				}
			}

			for(int iMl = 0; iMl < microLenses.size(); iMl++)
			{
				float centerX = microLenses[iMl]->centerX;
				float centerY = microLenses[iMl]->centerY;
				float xR = (xUps_flt-centerX)/vdepth+centerX;
				float yR = (yUps_flt-centerY)/vdepth+centerY;

				if(!(xR>=0 && xR<=rawWidth-1 && yR>=0 && yR<=rawHeight-1))
					continue;

				float distToCenterX = xR-centerX;
				float distToCenterY = yR-centerY;
				float distToCenter_2 = distToCenterX*distToCenterX+distToCenterY*distToCenterY;

				// Ingnore pixels which are close to the boarder of a micro image
				if(distToCenter_2 >= mlGrid->lensValidityRadius_2) continue;

				frames[iFrame].rawImageCoordinates.push_back(Eigen::Vector2d(xR,yR));
				frames[iFrame].microLensCenter.push_back(Eigen::Vector2d(centerX,centerY));
				frames[iFrame].objectCoordinatesByRawID.push_back(frames[iFrame].objectCoordinatesByID[p]); 
			}
		}
	}

	return true;
}

/**
 * @brief Define optimization problem and run bundle adjustment based on new camera interpretation.
 */
bool CameraCalibration::performBundleAdjustment()
{
	printf("\nPerform Bundle Adjustment.\n");

	unsigned int config = 0;

	int nParametersTotal = 0;
	int nObservations = 0;

	int nCameraParameters = 5; // fL, B, bL0 and principal point
	
	// Radial distortion parameters
	if(nRadialDistParam>2)nRadialDistParam = 2;
	nCameraParameters += nRadialDistParam;

	config |= (nRadialDistParam & 0x000003);

	// Tangential distortion parameters
	if(tangentialDistParam)
	{
		nCameraParameters += 2;
		config |= (0x000004);
	}

	nParametersTotal = nCameraParameters;
	// Refine frame poses
	int nViewParameters = 6*frames.size();
	if(refinePoses)
	{
		nParametersTotal += nViewParameters;
		config |= (0x000100);
	}

	if(useRobustCostFunction)
		config |= (0x000200);

	if(refine3Dpoints)
		config |= (0x000400);

	if(mlCenterAdjustment)
		config |= (0x000800);

	for(unsigned int iFrame=0; iFrame < frames.size(); iFrame++)
	{
		nObservations += frames[iFrame].rawImageCoordinates.size();
	}

	init_camera = new double[MAX_NUMBER_OF_CAMERA_PARAMETERS];
	camera = new double[MAX_NUMBER_OF_CAMERA_PARAMETERS];
	for(int i=0;i<MAX_NUMBER_OF_CAMERA_PARAMETERS;i++)
	{
		camera[i] = 0;
		init_camera[i] = 0;
	}

	views = new double[nViewParameters];
	init_views = new double[nViewParameters];

	camera[0] = fL_init;			// thin lens focal length
	camera[1] = bL0_init;			// bL0 in millimeter
	camera[2] = B_init;				// B in millimeter
	camera[3] = c_init[0];			// cx in pixel
	camera[4] = c_init[1];			// cy in pixel

	int idx = 5;
	// Radial distortion parameters
	double ps_square = pixelSize_totFoc*pixelSize_totFoc;
	for(int i=0; i<nRadialDistParam; i++)
	{
		camera[idx+i] = 0;
	}
	idx += nRadialDistParam;

	// Tangential distortion parameters
	if(tangentialDistParam)
	{
		camera[idx] = 0;
		camera[idx+1] = 0;
		idx += 2;
	}

	memcpy(init_camera,camera, nCameraParameters*sizeof(double));
	memcpy(init_views,views, nViewParameters*sizeof(double));

	ceres::Problem minimizationProblem;
	for(unsigned int iImage = 0; iImage < frames.size(); iImage++)
	{
		frame* currentFrame;
		currentFrame = &frames[iImage];

		for(int i=0;i<3;i++)
		{
			views[6*iImage+i] = currentFrame->rotationAngles[i];
			views[6*iImage+3+i] = currentFrame->transVector[i];
		}

		unsigned int nImPts = currentFrame->rawImageCoordinates.size();
		for(unsigned int i=0; i<nImPts; i++)
		{
			Eigen::Vector2d imagePt = currentFrame->rawImageCoordinates[i];
			Eigen::Vector2d mlCenter = currentFrame->microLensCenter[i];
			Eigen::Vector3d objectPt = *currentFrame->objectCoordinatesByRawID[i];
			Eigen::Vector3d *pObjectPt = currentFrame->objectCoordinatesByRawID[i];

			ceres::CostFunction* cost_function;
			if(refinePoses)
			{
				if(this->refine3Dpoints)
					cost_function = OurCostFunctionBundle::Create(config, (double)imagePt[0], (double)imagePt[1], pixelSize_totFoc, pixelSize_totFoc, (double)depth_to_raw_im_scale, mlCenter);
				else
				{
					cost_function = OurCostFunctionBundle::Create(config, (double)imagePt[0], (double)imagePt[1], pixelSize_totFoc, pixelSize_totFoc, (double)depth_to_raw_im_scale, mlCenter, pObjectPt);
					pObjectPt = NULL;
				}
				
				if(pObjectPt != NULL)
				{
					if(useRobustCostFunction)
						minimizationProblem.AddResidualBlock( cost_function, new ceres::CauchyLoss(0.5), camera, views+(6*iImage), (double*)pObjectPt->data() );
					else
						minimizationProblem.AddResidualBlock( cost_function, NULL /* squared loss */, camera, views+(6*iImage), (double*)pObjectPt->data() );
				}
				else
				{
					if(useRobustCostFunction)
						minimizationProblem.AddResidualBlock( cost_function, new ceres::CauchyLoss(0.5), camera, views+(6*iImage) );
					else
						minimizationProblem.AddResidualBlock( cost_function, NULL /* squared loss */, camera, views+(6*iImage) );
				}
			}
			else
			{
				cost_function = OurCostFunctionBundle::Create(config, (double)imagePt[0], (double)imagePt[1], pixelSize_totFoc, pixelSize_totFoc, (double)depth_to_raw_im_scale, mlCenter, pObjectPt, views + 6*iImage);

				if(useRobustCostFunction)
					minimizationProblem.AddResidualBlock( cost_function, new ceres::CauchyLoss(0.5), camera);
				else
					minimizationProblem.AddResidualBlock( cost_function, NULL /* squared loss */, camera);
			}
		}
	}

	if(this->refine3Dpoints && calibData->calib_type != RECALIBRATION)
	{
		// Add constraints
		for(std::vector<Constraint>::iterator it = this->constraintList.begin(); it!=this->constraintList.end(); it++)
		{
			ceres::CostFunction* cost_function;
			cost_function = OurConstraintFunctionBundle::Create(it->distance,it->sigma);
			minimizationProblem.AddResidualBlock( cost_function, NULL /* squared loss */, this->p3d_w[it->pointID_1].data(), this->p3d_w[it->pointID_2].data() );
		}
	}

	if(calibData->calib_type == RECALIBRATION)
	{
		// Manifold used to restrict camera parameters f and B for modal solvers
		ceres::SubsetManifold* constant_parameters_manifold = nullptr;
		if (true)
		{
			std::vector<int> constant_params;

			constant_params.push_back(0);	// f parameter
			constant_params.push_back(2);	// B parameter

			constant_parameters_manifold = new ceres::SubsetManifold(MAX_NUMBER_OF_CAMERA_PARAMETERS, constant_params);
		}
		minimizationProblem.SetManifold(camera, constant_parameters_manifold);

		// Upper and lower bound for bl_0
		minimizationProblem.SetParameterLowerBound(camera, 1, 0.7*bL0_init);
		minimizationProblem.SetParameterUpperBound(camera, 1, 1.3*bL0_init);
		
		// Upper and lower bound for c_x
		minimizationProblem.SetParameterLowerBound(camera, 3, 0.7*c_init[0]);
		minimizationProblem.SetParameterUpperBound(camera, 3, 1.3*c_init[0]);
		
		// Upper and lower bound for c_y
		minimizationProblem.SetParameterLowerBound(camera, 4, 0.7*c_init[1]);
		minimizationProblem.SetParameterUpperBound(camera, 4, 1.3*c_init[1]);
	}

	ceres::Solver::Options options_bundle;
	options_bundle.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
	options_bundle.minimizer_progress_to_stdout = true;
	options_bundle.function_tolerance = 1.0e-6;
	options_bundle.parameter_tolerance = 1.0e-8;
	options_bundle.max_num_iterations = 200;
	options_bundle.num_threads = std::thread::hardware_concurrency();
	ceres::Solver::Summary summary_bundle;

	// Solve problem
	ceres::Solve(options_bundle, &minimizationProblem, &summary_bundle);

	// Store estimated parameters
	this->fL = camera[0];
	this->bL0 = camera[1];
	this->B = camera[2];
	this->c[0] = camera[3];
	this->c[1] = camera[4];

	idx = 5;
	this->radialDist = Eigen::VectorXd(this->nRadialDistParam);
	for(int i=0; i<this->nRadialDistParam; i++)
		this->radialDist[i] = camera[idx++];

	if(this->tangentialDistParam)
	{
		this->tangentialDist[0] = camera[idx++];
		this->tangentialDist[1] = camera[idx++];
	}
	else
	{
		this->tangentialDist[0] = 0;
		this->tangentialDist[1] = 0;
	}

	printf("Finished Bundle Adjustment.\n");
	return true;
}

void CameraCalibration::showResults()
{
	// Display estimated parameters
	printf("Estimated Parameters:\n");
	printf("\tfL  : %18.15f -> %18.15f\n",this->fL_init, this->fL);
	printf("\tbL0 : %18.15f -> %18.15f\n",this->bL0_init, this->bL0);
	printf("\tB   : %18.15f -> %18.15f\n",this->B_init, this->B);
	printf("\tcx  : %18.15f -> %18.15f\n",this->c_init[0], this->c[0]);
	printf("\tcy  : %18.15f -> %18.15f\n",this->c_init[1], this->c[1]);

	for(int i=0; i<this->nRadialDistParam; i++)
		printf("\ta%d  : %18.15f -> %18.15f\n",i,0.0,this->radialDist[i]);

	if(this->tangentialDistParam)
	{
		printf("\tb0  : %18.15f -> %18.15f\n",this->tangentialDist_init[0], this->tangentialDist[0]);
		printf("\tb1  : %18.15f -> %18.15f\n",this->tangentialDist_init[1], this->tangentialDist[1]);
	}

	// Depth distortion model
	int nDepthDistParam = 0;

	printf("\n");


	calcReprojectionError();
	printf("Reprojection Error:\n"
		"\tx_std =  %8.5f\n\ty_std =  %8.5f\n"
		"\tx_mae =  %8.5f\n\ty_mae =  %8.5f\n"
		"\toutl. = %d/%d\n\n",std_x,std_y,mae_x,mae_y, this->numPts-this->numInliers, this->numPts);
}

void CameraCalibration::calcReprojectionError(double inlierThreshold)
{
	double spx_raw = this->pixelSize_totFoc/((float)depth_to_raw_im_scale);
	double spy_raw = this->pixelSize_totFoc/((float)depth_to_raw_im_scale);

	double c_raw[2];
	c_raw[0] = (this->c[0]+0.5)*((float)depth_to_raw_im_scale)-0.5;
	c_raw[1] = (this->c[1]+0.5)*((float)depth_to_raw_im_scale)-0.5;

	double* pRadialDist = (double*)this->radialDist.data();
	if(this->nRadialDistParam<=0) pRadialDist = NULL;

	double* pTangentialDist = (double*)this->tangentialDist.data();
	if(!this->tangentialDistParam) pTangentialDist = NULL;

	double sum_sq_error_x = 0;
	double sum_sq_error_y = 0;
	int n_error = 0;

	int n_inliers = 0;

	this->mae_x = 0;
	this->mae_y = 0;

	this->projectionErrors.resize(frames.size());

	for(unsigned int iImage = 0; iImage < frames.size(); iImage++)
	{
		frame* currentFrame;
		currentFrame = &frames[iImage];

		unsigned int nImPts = currentFrame->rawImageCoordinates.size();
		this->projectionErrors[iImage].resize(nImPts);

		for(unsigned int i=0; i<nImPts; i++)
		{
			Eigen::Vector2d imagePt = currentFrame->rawImageCoordinates[i];
			Eigen::Vector2d mlCenter = currentFrame->microLensCenter[i];
			Eigen::Vector3d p3d_w = *currentFrame->objectCoordinatesByRawID[i];

			Eigen::Vector3d rotationAngles, transVector;
			for(int k=0;k<3;k++)
			{
				rotationAngles[k] = views[6*iImage+k];
				transVector[k] = views[6*iImage+3+k];
			}

			Eigen::Matrix4d RT = RigidBody::getTransformationMatrix(rotationAngles,transVector);
			Eigen::Vector4d p3d_hc = RT*p3d_w.homogeneous();
			Eigen::Vector3d p3d_c = p3d_hc.hnormalized();

			double x_i, y_i;
			CameraModel::projectPoint<double>(x_i,y_i,p3d_c,spx_raw,spy_raw,fL,bL0,B,c_raw,mlCenter.data(),pRadialDist,this->nRadialDistParam,pTangentialDist,this->mlCenterAdjustment);

			double error_x = x_i - imagePt[0];
			double error_y = y_i - imagePt[1];

			if(abs(error_x)>mae_x) mae_x = abs(error_x);
			if(abs(error_y)>mae_y) mae_y = abs(error_y);

			this->projectionErrors[iImage][i] = Eigen::Vector3d(error_x,error_y,0.0);

			if(error_x*error_x+error_y*error_y<=inlierThreshold*inlierThreshold)
				n_inliers++;

			sum_sq_error_x += error_x*error_x;
			sum_sq_error_y += error_y*error_y;
			n_error++;
		}
	}

	this->std_x = sqrt(sum_sq_error_x/n_error);
	this->std_y = sqrt(sum_sq_error_y/n_error);
	this->std_iv = -1;
	this->mae_iv = -1;
	this->numPts = n_error;
	this->numInliers = n_inliers;
}

bool CameraCalibration::storeResults()
{
	double spx = this->pixelSize_totFoc;
	double spy = this->pixelSize_totFoc;

	double* p_c = this->c.data();

	double* pRadialDist = (double*)this->radialDist.data();
	if(this->nRadialDistParam<=0) pRadialDist = NULL;

	double* pTangentialDist = (double*)this->tangentialDist.data();
	if(!this->tangentialDistParam) pTangentialDist = NULL;

	printf("Storing data and Results\n");
	// Camera model (xml)
	if(!storeCameraModel()) return false;

	// Camera poses (xml)
	if(!storeExtrinsicOrientations()) return false;

	// Camera poses (txt)
	if(!storeExtrinsicOrientationsTxt()) return false;

	// Image points (xml)
	if(!storeRawImagePointsCsv()) return false;

	// point cloud world coordinates (ply)
	printf("Storing point cloud in world coordinates ...\n");
	std::ofstream f_obj_pts(this->dirResults + "/objectCoordinates.ply");
	f_obj_pts << std::string("ply\n");
	f_obj_pts << std::string("format ascii 1.0\n");
	f_obj_pts << std::string("element vertex ") << this->p3d_w.size() << std::string("\n");
	f_obj_pts << std::string("property float x\n");
	f_obj_pts << std::string("property float y\n");
	f_obj_pts << std::string("property float z\n");
	f_obj_pts << std::string("property uchar intensity\n");
	f_obj_pts << std::string("end_header\n");
	for(int i=0;i<p3d_w.size();i++) 
		f_obj_pts << p3d_w[i][0] << " " << p3d_w[i][1] << " " << p3d_w[i][2] << " " << 0 << "\n";
	f_obj_pts.close();

	// Point cloud world coordinates with COLMAP IDs (txt)
	printf("Storing point cloud in world coordinates ...\n");
	std::ofstream f_obj_pts_COLMAP_IDs(this->dirResults + "/objectCoordinatesWithCOLMAPIDs.txt");
	f_obj_pts_COLMAP_IDs << std::string("# COLMAP_ID X Y Z\n");
	for(int i=0;i<p3d_w.size();i++) 
		f_obj_pts_COLMAP_IDs << calibData->getCorrespondingCOLMAPID(i) << " " << p3d_w[i][0] << " " << p3d_w[i][1] << " " << p3d_w[i][2] << "\n";
	f_obj_pts_COLMAP_IDs.close();

	// Camera orientations (ply) (just for visualization in cloud compare)
	printf("Storing camera orientations as ply ...\n");
	std::ofstream f_cam_pose(this->dirResults + "/cameraOrientations.ply");
	f_cam_pose << std::string("ply\n");
	f_cam_pose << std::string("format ascii 1.0\n");
	f_cam_pose << std::string("element vertex ") << this->frames.size()*5 << std::string("\n");
	f_cam_pose << std::string("property float x\n");
	f_cam_pose << std::string("property float y\n");
	f_cam_pose << std::string("property float z\n");
	f_cam_pose << std::string("property uchar red\n");
	f_cam_pose << std::string("property uchar green\n");
	f_cam_pose << std::string("property uchar blue\n");
	f_cam_pose << std::string("element face ") << this->frames.size()*4 << std::string("\n");
	f_cam_pose << std::string("property list uchar int vertex_index\n");
	f_cam_pose << std::string("end_header\n");

	float cx = this->c[0];
	float cy = this->c[1];
	float f = this->fL/this->pixelSize_totFoc;
	float fL = this->fL*3;
	float w = this->imageSize[0];
	float h = this->imageSize[1];
	Eigen::Vector4d v0(0,0,0,1);
	Eigen::Vector4d v1((0-cx)/f*fL,(0-cy)/f*fL,fL,1);
	Eigen::Vector4d v2((0-cx)/f*fL,(h-1-cy)/f*fL,fL,1);
	Eigen::Vector4d v3((w-1-cx)/f*fL,(h-1-cy)/f*fL,fL,1);
	Eigen::Vector4d v4((w-1-cx)/f*fL,(0-cy)/f*fL,fL,1);
	for(int i=0;i<this->frames.size();i++)
	{
		// Get extrinsic orientation
		double* angle = views + 6*i;
		Eigen::Vector3d rotVec(angle[0],angle[1],angle[2]);
		double* trans = views + 6*i + 3;
		Eigen::Vector3d transVec(trans[0],trans[1],trans[2]);
		Eigen::Matrix4d m = RigidBody::getTransformationMatrix<double>(rotVec, transVec);
		m = m.inverse().eval();

		Eigen::Vector4d v0_tmp = m*v0;
		Eigen::Vector4d v1_tmp = m*v1;
		Eigen::Vector4d v2_tmp = m*v2;
		Eigen::Vector4d v3_tmp = m*v3;
		Eigen::Vector4d v4_tmp = m*v4;

		f_cam_pose << v0_tmp[0] << " " << v0_tmp[1] << " " << v0_tmp[2] << " " << 0 << " " << 0 << " " << 255 << "\n";
		f_cam_pose << v1_tmp[0] << " " << v1_tmp[1] << " " << v1_tmp[2] << " " << 0 << " " << 0 << " " << 255 << "\n";
		f_cam_pose << v2_tmp[0] << " " << v2_tmp[1] << " " << v2_tmp[2] << " " << 0 << " " << 0 << " " << 255 << "\n";
		f_cam_pose << v3_tmp[0] << " " << v3_tmp[1] << " " << v3_tmp[2] << " " << 0 << " " << 0 << " " << 255 << "\n";
		f_cam_pose << v4_tmp[0] << " " << v4_tmp[1] << " " << v4_tmp[2] << " " << 0 << " " << 0 << " " << 255 << "\n";
	}
	for(int i=0;i<this->frames.size();i++)
	{
		int vertex0 = i*5;
		int vertex1 = i*5+1;
		int vertex2 = i*5+2;
		int vertex3 = i*5+3;
		int vertex4 = i*5+4;
		int numVertperFace = 3;
		f_cam_pose << numVertperFace << " " << vertex0 << " " << vertex1 << " " << vertex2 << "\n";
		f_cam_pose << numVertperFace << " " << vertex0 << " " << vertex2 << " " << vertex3 << "\n";
		f_cam_pose << numVertperFace << " " << vertex0 << " " << vertex3 << " " << vertex4 << "\n";
		f_cam_pose << numVertperFace << " " << vertex0 << " " << vertex4 << " " << vertex1 << "\n";
	}
	f_cam_pose.close();

	// Point cloud camera coordinates (ply)
	printf("Storing point cloud in camera coordinates (only visible points) ...\n");

	// Create folders
	std::string dirRefCameraCoordinates = this->dirResults + "/refCameraCoordinates";
	if(!fileManagement::CreateDir(dirRefCameraCoordinates)) return false;
	std::string dirProjectedCameraCoordinates = this->dirResults + "/projectedCameraCoordinates";
	if(!fileManagement::CreateDir(dirProjectedCameraCoordinates)) return false;

	char sBuffer[10];
	for(int iImage=0;iImage<this->frames.size();iImage++)
	{
		frame* currentFrame = &frames[iImage];
		int frameID = currentFrame->id;
		sprintf(sBuffer,"%04d",frameID);

		// Get extrinsic orientation
		double* angle = views + 6*iImage;
		Eigen::Vector3d rotVec(angle[0],angle[1],angle[2]);
		double* trans = views + 6*iImage + 3;
		Eigen::Vector3d transVec(trans[0],trans[1],trans[2]);
		Eigen::Matrix4d RT_new = RigidBody::getTransformationMatrix<double>(rotVec, transVec);

		int nImPts = currentFrame->imageCoordinates.size();

		// Store point cloud in camera coordinates
		std::ofstream f_cam_new(dirRefCameraCoordinates + "/cameraCoordinates_"+std::string(sBuffer)+".ply");
		f_cam_new << std::string("ply\n");
		f_cam_new << std::string("format ascii 1.0\n");
		f_cam_new << std::string("element vertex ") << nImPts << std::string("\n");
		f_cam_new << std::string("property float x\n");
		f_cam_new << std::string("property float y\n");
		f_cam_new << std::string("property float z\n");
		f_cam_new << std::string("property uchar intensity\n");
		f_cam_new << std::string("end_header\n");
		for(int iPoint=0;iPoint<nImPts;iPoint++)
		{
			// Get object point
			Eigen::Vector3d objPt = *currentFrame->objectCoordinatesByID[iPoint];

			// Transform object point to new camera coordinates
			Eigen::Vector4d newCamPt = RT_new*objPt.homogeneous();
			f_cam_new << newCamPt[0] << " " << newCamPt[1] << " " << newCamPt[2] << " " << 0 << "\n";
		}
		f_cam_new.close();

		// Store back projected camera coordinates
		std::ofstream f_cam_proj(dirProjectedCameraCoordinates + "/cameraCoordinates_"+std::string(sBuffer)+".ply");
		f_cam_proj << std::string("ply\n");
		f_cam_proj << std::string("format ascii 1.0\n");
		f_cam_proj << std::string("element vertex ") << nImPts << std::string("\n");
		f_cam_proj << std::string("property float x\n");
		f_cam_proj << std::string("property float y\n");
		f_cam_proj << std::string("property float z\n");
		f_cam_proj << std::string("property uchar intensity\n");
		f_cam_proj << std::string("end_header\n");
		for(int iPoint=0;iPoint<nImPts;iPoint++)
		{
			// Get measured virtual image point
			Eigen::Vector2d imagePt = currentFrame->imageCoordinates[iPoint];
			double vDepth = this->virtualDepthValues[iImage][iPoint];

			Eigen::Vector3d projCamPt;			
			CameraModel::projectPointBack<double>(projCamPt,imagePt[0],imagePt[1],vDepth,spx,spy,fL,bL0,B,p_c,
				pRadialDist,this->nRadialDistParam,pTangentialDist);

			f_cam_proj << projCamPt[0] << " " << projCamPt[1] << " " << projCamPt[2] << " " << 0 << "\n";
		}
		f_cam_proj.close();
	}

	// Protocoll
	if(!storeProtocol()) return false;

	printf("Finished storing results.\n\n");
	return true;
}

bool CameraCalibration::storeCameraModel()
{
	std::string filePath = this->dirResults + "/CameraModel.xml";

	pugi::xml_document Doc;
	pugi::xml_node nodeDecl, nodeRoot, nodeModel, nodeImSize, nodeWidth, nodeHeight, nodePixelSize, nodePrincipalPoint, nodeFocalLength, nodeMainLensMlaDistance, nodeSensorMlaDistance, nodeRadDist, nodeTanDist, nodeDepthDist, nodeMlCenterAdjust;
	pugi::xml_node nodeVal;
			
	std::fstream File;

	File.open(filePath.c_str(),std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	File.close(); 

	Doc.load_file(filePath.c_str());
	nodeDecl=Doc.prepend_child(pugi::node_declaration);
	nodeDecl.append_attribute("version")="1.0";
	nodeDecl.append_attribute("encoding")="UTF-8";

	nodeRoot=Doc.append_child("Root");

	nodeModel = nodeRoot.append_child("CalibrationModel");

	nodeModel.text().set("Plenoptic");

	nodeImSize = nodeRoot.append_child("ImageSize");
	nodeImSize.append_attribute("units")="pix";
	nodeWidth = nodeImSize.append_child("Width");
	nodeWidth.text().set((boost::lexical_cast<std::string>(this->imageSize[0]).c_str()));
	nodeHeight = nodeImSize.append_child("Height");
	nodeHeight.text().set((boost::lexical_cast<std::string>(this->imageSize[1]).c_str()));

	nodePixelSize = nodeRoot.append_child("PixelSize");
	nodePixelSize.append_attribute("units")="mm";
	nodePixelSize.text().set((boost::str(boost::format("%.5f") % this->pixelSize).c_str()));

	nodePrincipalPoint = nodeRoot.append_child("PrincipalPoint");
	nodePrincipalPoint.append_attribute("units")="pix";
	nodeVal = nodePrincipalPoint.append_child("x");
	nodeVal.text().set((boost::lexical_cast<std::string>(this->c[0]).c_str()));
	nodeVal = nodePrincipalPoint.append_child("y");
	nodeVal.text().set((boost::lexical_cast<std::string>(this->c[1]).c_str()));

	nodeFocalLength = nodeRoot.append_child("FocalLength");
	nodeFocalLength.append_attribute("units")="mm";
	nodeFocalLength.text().set((boost::lexical_cast<std::string>(this->fL).c_str()));

	nodeMainLensMlaDistance = nodeRoot.append_child("MainLensMlaDistance");
	nodeMainLensMlaDistance.append_attribute("units")="mm";
	nodeMainLensMlaDistance.text().set((boost::lexical_cast<std::string>(this->bL0).c_str()));

	nodeSensorMlaDistance = nodeRoot.append_child("SensorMlaDistance");
	nodeSensorMlaDistance.append_attribute("units")="mm";
	nodeSensorMlaDistance.text().set((boost::lexical_cast<std::string>(this->B).c_str()));

	if(this->radialDist.rows()>0)
	{
		nodeRadDist = nodeRoot.append_child("RadialDistortion");
		nodeRadDist.append_attribute("units")="mm";

		for(int i=0; i<this->radialDist.rows(); i++)
		{
			std::string buff = "A"+std::to_string(i);
			nodeVal = nodeRadDist.append_child(buff.c_str());
			nodeVal.text().set((boost::lexical_cast<std::string>(this->radialDist[i]).c_str()));
		}
	}

	if(this->tangentialDistParam)
	{
		nodeTanDist = nodeRoot.append_child("TangentialDistortion");
		nodeTanDist.append_attribute("units")="mm";

		nodeVal = nodeTanDist.append_child("B0");
		nodeVal.text().set((boost::lexical_cast<std::string>(this->tangentialDist[0]).c_str()));
		nodeVal = nodeTanDist.append_child("B1");
		nodeVal.text().set((boost::lexical_cast<std::string>(this->tangentialDist[1]).c_str()));
	}
	
	nodeMlCenterAdjust = nodeRoot.append_child("MicroLensCenterAdjustment");
	if(this->mlCenterAdjustment)
		nodeMlCenterAdjust.text().set("true");
	else
		nodeMlCenterAdjust.text().set("false");

	Doc.save_file(filePath.c_str());

return true;
}

bool CameraCalibration::storeExtrinsicOrientations()
{
	double* views = this->views;

	std::string filePath = this->dirResults + "/extrinsicOrientations.xml";

	pugi::xml_document Doc;
	pugi::xml_node nodeRoot, nodeDecl, nodeFrame, nodeTrans, nodeRot, nodeCoeff;
	std::fstream File;

	File.open(filePath.c_str(),std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	File.close(); 

	Doc.load_file(filePath.c_str());
	nodeDecl=Doc.prepend_child(pugi::node_declaration);
	nodeDecl.append_attribute("version")="1.0";
	nodeDecl.append_attribute("encoding")="UTF-8";

	nodeRoot=Doc.append_child("Root");

	char sBuffer[10];
	for(int iImage=0;iImage<this->frames.size();iImage++)
	{
		frame* currentFrame = &frames[iImage];
		
		// get extrinsic orientation
		double* angle = views + 6*iImage;
		double* trans = views + 6*iImage + 3;

		int frameID = currentFrame->id;

		nodeFrame = nodeRoot.append_child("Frame");
		nodeFrame.append_attribute("id") = frameID;

		nodeRot = nodeFrame.append_child("Rotation");
		for(int i=0;i<3;i++)
		{
			nodeCoeff = nodeRot.append_child("Coeff");
			nodeCoeff.append_attribute("i") = i;
			nodeCoeff.text().set(boost::lexical_cast<std::string>(angle[i]).c_str());
		}

		nodeTrans = nodeFrame.append_child("Translation");
		for(int i=0;i<3;i++)
		{
			nodeCoeff = nodeTrans.append_child("Coeff");
			nodeCoeff.append_attribute("i") = i;
			nodeCoeff.text().set(boost::lexical_cast<std::string>(trans[i]).c_str());
		}
	}

	Doc.save_file(filePath.c_str());
	return true;
}

bool CameraCalibration::storeExtrinsicOrientationsTxt()
{
	double* views = this->views;
	std::string filePath = this->dirResults + "/ExtrinsicOrientations.txt";

	FILE* file = fopen(filePath.c_str(),"w+");
	if(file == NULL) return false;

	std::vector<int> id_vector(frames.size());
	std::vector<int> idx(frames.size());
	for(int i=0; i<id_vector.size(); i++)
	{
		id_vector[i] = frames[i].id;
		idx[i] = i;
	}

	std::sort(
		std::begin(idx), std::end(idx),
        [&](int a, int b) { return id_vector[a] < id_vector[b]; });

	for(int i=0; i<frames.size(); i++)
	{
		int iFrame = idx[i];
		frame* currentFrame = &frames[iFrame];		
		// get extrinsic orientation
		double* angle = views + 6*iFrame;
		double* trans = views + 6*iFrame + 3;

		Eigen::Vector3d rotAngle(angle[0],angle[1],angle[2]);
		Eigen::Vector3d transVec(trans[0],trans[1],trans[2]);

		Eigen::Matrix4d mat = RigidBody::getTransformationMatrix<double>(rotAngle,transVec);

		fprintf(file,"%05d",currentFrame->id);
		for(int y=0;y<4;y++)
			for(int x=0;x<4;x++)
				fprintf(file," %16.10f",mat(y,x));
		fprintf(file,"\n");
	}
	fclose(file);
	return true;
}

bool CameraCalibration::storeRawImagePointsCsv()
{
	std::string filePath = this->dirResults + "/rawImagePoints.csv";

	FILE* pFile = fopen(filePath.c_str(),"w");
	if(pFile == NULL) return false;

	// calculate projected coordinates
	double spx_raw = this->pixelSize_totFoc/((float)depth_to_raw_im_scale);
	double spy_raw = this->pixelSize_totFoc/((float)depth_to_raw_im_scale);

	double c_raw[2];
	c_raw[0] = (this->c[0]+0.5)*((float)depth_to_raw_im_scale)-0.5;
	c_raw[1] = (this->c[1]+0.5)*((float)depth_to_raw_im_scale)-0.5;

	double* pRadialDist = (double*)this->radialDist.data();
	if(this->nRadialDistParam<=0) pRadialDist = NULL;

	double* pTangentialDist = (double*)this->tangentialDist.data();
	if(!this->tangentialDistParam) pTangentialDist = NULL;

	for(unsigned int iImage = 0; iImage < frames.size(); iImage++)
	{
		frame* currentFrame;
		currentFrame = &frames[iImage];

		int frameID = currentFrame->id;

		unsigned int nImPts = currentFrame->rawImageCoordinates.size();
		for(unsigned int i=0; i<nImPts; i++)
		{
			Eigen::Vector2d imagePt = currentFrame->rawImageCoordinates[i];
			Eigen::Vector2d mlCenter = currentFrame->microLensCenter[i];
			Eigen::Vector3d p3d_w = *currentFrame->objectCoordinatesByRawID[i];

			
			
			Eigen::Vector3d rotationAngles, transVector;
			for(int k=0;k<3;k++)
			{
				rotationAngles[k] = views[6*iImage+k];
				transVector[k] = views[6*iImage+3+k];
			}

			Eigen::Matrix4d RT = RigidBody::getTransformationMatrix(rotationAngles,transVector);
			Eigen::Vector4d p3d_hc = RT*p3d_w.homogeneous();
			Eigen::Vector3d p3d_c = p3d_hc.hnormalized();

			double x_i, y_i;
			CameraModel::projectPoint<double>(x_i,y_i,p3d_c,spx_raw,spy_raw,fL,bL0,B,c_raw,mlCenter.data(),pRadialDist,this->nRadialDistParam,pTangentialDist,this->mlCenterAdjustment);

			// Add the index of the 3d point on the calibration target to establish correspondence
			Eigen::Vector3d* ptP3d_w = currentFrame->objectCoordinatesByRawID[i];
			int idx_p3d_w = std::distance(this->p3d_w.data(), ptP3d_w);
			fprintf(pFile,"%d,%d,%f,%f,%f,%f,%d\n",frameID,i,imagePt[0],imagePt[1],x_i,y_i,idx_p3d_w);
		}
	}

	fclose(pFile);
	return true;
}

bool CameraCalibration::storeProtocol()
{
	FILE *fProtocol;
	std::string dirProtocolFile = this->dirResults + "/calibrationProtocol.txt";
	fProtocol = fopen(dirProtocolFile.c_str(), "w+");
	if(fProtocol == NULL) return false;

	fprintf(fProtocol,
		"*******************************************************************************\n"
		"***   LiFCal: Online Light Field Camera Calibration via Bundle Adjustment   ***\n"
		"*******************************************************************************\n\n");

	fprintf(fProtocol,"*** Intrinsic Parameters ***\n");
	fprintf(fProtocol,"Pixel Size: %1.3f mm\n",this->pixelSize);
	
	fprintf(fProtocol,"\tfL   : %18.15f\n",fL);
	fprintf(fProtocol,"\tbL0  : %18.15f\n",bL0);
	fprintf(fProtocol,"\tB    : %18.15f\n",B);
	fprintf(fProtocol,"\tcx   : %18.15f\n",c[0]);
	fprintf(fProtocol,"\tcy   : %18.15f\n",c[1]);

	// Radial distortion parameters
	if(this->nRadialDistParam>0)
	{
		for(int i=0; i<this->radialDist.rows(); i++)
			fprintf(fProtocol,"\ta%d   : %18.15f\n",i,radialDist[i]);
	}

	// Tangetial distortion parameters
	if(this->tangentialDistParam)
	{
		fprintf(fProtocol,"\tb0   : %18.15f\n",tangentialDist[0]);
		fprintf(fProtocol,"\tb1   : %18.15f\n",tangentialDist[1]);
	}
	fprintf(fProtocol,"\n");

	// Depth distortion model
	if(this->mlCenterAdjustment)
		fprintf(fProtocol,"\tDid micro lens center adjustment\n");

	fprintf(fProtocol,"*** Additional Settings ***\n");
	fprintf(fProtocol,"\tDistortion defined on MLA plane.\n");
	fprintf(fProtocol,"\n");

	if(this->refinePoses)
		fprintf(fProtocol,"\tExtrinsic Orientations were refined.\n");
	else
		fprintf(fProtocol,"\tExtrinsic Orientations from COLMAP were kept.\n");
	fprintf(fProtocol,"\n");

	if(this->refine3Dpoints)
		fprintf(fProtocol,"\t3D Object coordinates were refined.\n");
	else
		fprintf(fProtocol,"\t3D Object coordinates from COLMAP were kept.\n");
	fprintf(fProtocol,"\n");

	if(this->useRobustCostFunction)
		fprintf(fProtocol,"\tRobust cost function was used for estimation.\n");
	else
		fprintf(fProtocol,"\tSquared cost function was used for estimation.\n");
	fprintf(fProtocol,"\n");

	fprintf(fProtocol,"*** Statistics ***\n");
	fprintf(fProtocol,"\tReprojection errors:\n");
	fprintf(fProtocol,"\tstd. Dev. x:           %8.5f\n", this->std_x);
	fprintf(fProtocol,"\tstd. Dev. y:           %8.5f\n", this->std_y);
	fprintf(fProtocol,"\tmae x:                 %8.5f\n", this->mae_x);
	fprintf(fProtocol,"\tmae y:                 %8.5f\n", this->mae_y);

	fclose(fProtocol);

	return true;
}
