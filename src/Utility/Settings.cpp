/**
 * @brief Definition of the class CalibrationSettings.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CalibrationSettings.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <iostream>

#include "Settings.h"
#include "FileManagement.h"


CalibrationSettings::CalibrationSettings(void)
{
}

CalibrationSettings::~CalibrationSettings(void)
{
    
}

void CalibrationSettings::setDefaultValues(int defVal_n3DPoints,
									int defVal_markerDict,
									int defVal_nRadialDistParam,
									bool defVal_tangentialDistParam,
									bool defVal_mlCenterAdjustment,
									bool defVal_refinePoses,
									bool defVal_refine3Dpoints,
									bool defVal_useRobustCostFunction)
{
    /// Number of 3D points used for the light field bundle adjustment
    this->numberPoints = defVal_n3DPoints;
    /// Number of the used OpenCV marker dictionary
    this->markerDictionary = defVal_markerDict;

    /// Number of radial distortion parameters
    this->nRadialDistParam = defVal_nRadialDistParam;
    /// Define tangential distorsion
    this->tangentialDistParam = defVal_tangentialDistParam;
    /// adjust micro lens centers (since micro lens centers are different from estimated micro image centers).
    this->mlCenterAdjustment = defVal_mlCenterAdjustment;
    /// Camera pose are optimized.
    this->refinePoses = defVal_refinePoses;
    /// 3D object points are optimized.
    this->refine3Dpoints = defVal_refine3Dpoints;
    /// Robust cost function has to be used.
    this->useRobustCostFunction = defVal_useRobustCostFunction;
}

template<>
float CalibrationSettings::readParameter<float>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
{
    cv::FileNode node = fSettings[name];
    if(node.empty()){
        if(required){
            std::cerr << name << " Required parameter does not exist, aborting." << std::endl;
            exit(-1);
        }
        else{
            std::cerr << name << " Optional parameter does not exist, using default value." << std::endl;
            found = false;
            return 0.0f;
        }
    }
    else if(!node.isReal()){
        std::cerr << name << " Parameter must be a real number, aborting." << std::endl;
        exit(-1);
    }
    else{
        found = true;
        return node.real();
    }
}

template<>
double CalibrationSettings::readParameter<double>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
{
    cv::FileNode node = fSettings[name];
    if(node.empty()){
        if(required){
            std::cerr << name << " Required parameter does not exist, aborting." << std::endl;
            exit(-1);
        }
        else{
            std::cerr << name << " Optional parameter does not exist, using default value." << std::endl;
            found = false;
            return 0.0f;
        }
    }
    else if(!node.isReal()){
        std::cerr << name << " Parameter must be a real number, aborting." << std::endl;
        exit(-1);
    }
    else{
        found = true;
        return node.real();
    }
}

template<>
int CalibrationSettings::readParameter<int>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
{
    cv::FileNode node = fSettings[name];
    if(node.empty()){
        if(required){
            std::cerr << name << " Required parameter does not exist, aborting." << std::endl;
            exit(-1);
        }
        else{
            std::cerr << name << " Optional parameter does not exist, using default value." << std::endl;
            found = false;
            return 0;
        }
    }
    else if(!node.isInt()){
        std::cerr << name << " Parameter must be an integer number, aborting." << std::endl;
        exit(-1);
    }
    else{
        found = true;
        return node.operator int();
    }
}

template<>
std::string CalibrationSettings::readParameter<std::string>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required)
{
    cv::FileNode node = fSettings[name];
    if(node.empty()){
        if(required){
            std::cerr << name << " Required parameter does not exist, aborting." << std::endl;
            exit(-1);
        }
        else{
            std::cerr << name << " Optional parameter does not exist, using default value." << std::endl;
            found = false;
            return std::string();
        }
    }
    else if(!node.isString()){
        std::cerr << name << " Parameter must be a string, aborting." << std::endl;
        exit(-1);
    }
    else{
        found = true;
        return node.string();
    }
}

void CalibrationSettings::parseFile(const std::string &configFile)
{
    //Open settings file
    cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
    if (!fSettings.isOpened())
    {
        std::cerr << "[ERROR]: could not open configuration file at: " << configFile << std::endl;
        std::cerr << "Aborting." << std::endl;

        exit(-1);
    }
    else
    {
        printf("Loading settings from %s.\n", configFile.c_str());
    }

    dataValid = readSettingsFromFile(fSettings);
}

bool CalibrationSettings::readSettingsFromFile(cv::FileStorage &fSettings)
{
    bool found;

    // Size of a pixel in mm
    pixelSize = readParameter<float>(fSettings, settingNameFile_pixelSize, found, true);
    if(pixelSize <= 0)
	{
		printf("The given pixel size (%f) is not possible. Please enter a positive decimal value.\n", pixelSize);
		return false;
	}

    // Width of the raw image in pixels
    rawImageWidth = readParameter<int>(fSettings, settingNameFile_rawImageWidth, found, true);
    if(rawImageWidth <= 0)
	{
		printf("The given raw image width (%d) is not possible. Please enter a positive integer value.\n", rawImageWidth);
		return false;
	}

    // Height of the raw image in pixels
    rawImageHeight = readParameter<int>(fSettings, settingNameFile_rawImageHeight, found, true);
    if(rawImageHeight <= 0)
	{
		printf("The given raw image height (%d) is not possible. Please enter a positive integer value.\n", rawImageHeight);
		return false;
	}

    // Path of the total focus images
    dirTotalFocusImages = readParameter<std::string>(fSettings, settingNameFile_dirTotalFocusImages, found, true);
    if(!fileManagement::Exists(dirTotalFocusImages))
	{
		printf("The given directory (%s) does not exist.\n", dirTotalFocusImages.c_str());
		return false;
	}

    // Micro lens calibration file path
    dirMlCalibFile = readParameter<std::string>(fSettings, settingNameFile_dirMlCalibFile, found, true);
	if(!fileManagement::Exists(dirMlCalibFile))
	{
		printf("The given directory (%s) does not exist.\n", dirMlCalibFile.c_str());
		return false;
	}

    // Path to the virtual depth data
    dirDepthData = readParameter<std::string>(fSettings, settingNameFile_dirDepthData, found, true);
    if(!fileManagement::Exists(dirDepthData))
	{
		printf("The given directory (%s) does not exist.\n", dirDepthData.c_str());
		return false;
	}

    int readIntVal;

    // Number of 3D points used for the light field bundle adjustment
    readIntVal = readParameter<int>(fSettings, settingNameFile_numberPoints, found, false);
    if (found)
    {
        numberPoints = readIntVal;
        if(numberPoints <= 0)
        {
            printf("The given number of used 3D points (%d) is not possible. Please enter a positive integer value.\n", numberPoints);
            return false;
        }
    }

    // Number of the used OpenCV marker dictionary
    readIntVal = readParameter<int>(fSettings, settingNameFile_markerDictionary, found, false);
    if (found)
    {
        markerDictionary = readIntVal;
        if((markerDictionary < 0 || markerDictionary > 20))
        {
            printf("The given marker dictionary (%d) does not exist. Please enter an integer value between 0 and 20.\n", markerDictionary);
            return false;
        }
    }

    // Number of radial distortion parameters (0 - 2)
    readIntVal = readParameter<int>(fSettings, settingNameFile_nRadialDistParam, found, false);
    if (found)
    {
        nRadialDistParam = readIntVal;
        if((nRadialDistParam < 0 || nRadialDistParam > 2))
        {
            printf("The given number of radial distortion parameters (%d) is not valid (integer between %d and %d).\n", nRadialDistParam, 0, 2);
            return false;
        }
    }

    std::string stringValBool;

    // Define tangential distortion
    stringValBool = readParameter<std::string>(fSettings, settingNameFile_tangentialDistParam, found, false);
    if(found)
    {
        if(!stringToBool(stringValBool, tangentialDistParam)) return false;
    }

    // Adjust the micro lens centers
    stringValBool = readParameter<std::string>(fSettings, settingNameFile_mlCenterAdjustment, found, false);
    if(found)
    {
        if(!stringToBool(stringValBool, mlCenterAdjustment)) return false;
    }

    // Refine extrinsic orientations
    stringValBool = readParameter<std::string>(fSettings, settingNameFile_refinePoses, found, false);
    if(found)
    {
        if(!stringToBool(stringValBool, refinePoses)) return false;
    }

    // Refine the coordinates of the 3D calibration points
    stringValBool = readParameter<std::string>(fSettings, settingNameFile_refine3Dpoints, found, false);
    if(found)
    {
        if(!stringToBool(stringValBool, refine3Dpoints)) return false;
    }

    // Use a robust cost function
    stringValBool = readParameter<std::string>(fSettings, settingNameFile_useRobustCostFunction, found, false);
    if(found)
    {
        if(!stringToBool(stringValBool, useRobustCostFunction)) return false;
    }
    
    return true;
}

bool CalibrationSettings::stringToBool(std::string inputString, bool &boolVal)
{
    bool success = true;

    if(inputString == "true") boolVal = true;
    else if(inputString == "false") boolVal = false;
    else
    {
        printf("The given value (%s) is not valid. Please write \"true\" or \"false\".\n", inputString.c_str());
        success = false;
    }

    return success;
}

void CalibrationSettings::displaySettings()
{
    printf("\nThe following parameters are used for the calibration:\n");
    printf("\t%-35s: %f\n", settingNameFile_pixelSize.c_str(), pixelSize);
    printf("\t%-35s: %d\n", settingNameFile_rawImageWidth.c_str(), rawImageWidth);
    printf("\t%-35s: %d\n", settingNameFile_rawImageHeight.c_str(), rawImageHeight);
    printf("\t%-35s: %s\n", settingNameFile_dirTotalFocusImages.c_str(), dirTotalFocusImages.c_str());
    printf("\t%-35s: %s\n", settingNameFile_dirMlCalibFile.c_str(), dirMlCalibFile.c_str());
    printf("\t%-35s: %s\n", settingNameFile_dirDepthData.c_str(), dirDepthData.c_str());
    printf("\t%-35s: %d\n", settingNameFile_numberPoints.c_str(), numberPoints);
    printf("\t%-35s: %d\n", settingNameFile_markerDictionary.c_str(), markerDictionary);
    printf("\t%-35s: %d\n", settingNameFile_nRadialDistParam.c_str(), nRadialDistParam);
    printf("\t%-35s: %s\n", settingNameFile_tangentialDistParam.c_str(), tangentialDistParam ? "true" : "false");
    printf("\t%-35s: %s\n", settingNameFile_mlCenterAdjustment.c_str(), mlCenterAdjustment ? "true" : "false");
    printf("\t%-35s: %s\n", settingNameFile_refinePoses.c_str(), refinePoses ? "true" : "false");
    printf("\t%-35s: %s\n", settingNameFile_refine3Dpoints.c_str(), refine3Dpoints ? "true" : "false");
    printf("\t%-35s: %s\n", settingNameFile_useRobustCostFunction.c_str(), useRobustCostFunction ? "true" : "false");
}
