/**
 * @brief Declaration of the class CalibrationSettings.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CalibrationSettings.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include "opencv2/opencv.hpp"


class CalibrationSettings
{
public:
    /// Constructor from file
    CalibrationSettings();
    ~CalibrationSettings();

    /// Set default values for the settings
    void setDefaultValues(int defVal_n3DPoints,
									int defVal_markerDict,
									int defVal_nRadialDistParam,
									bool defVal_tangentialDistParam,
									bool defVal_mlCenterAdjustment,
									bool defVal_refinePoses,
									bool defVal_refine3Dpoints,
									bool defVal_useRobustCostFunction);
    
    /// Parse the settings file
    void parseFile(const std::string &configFile);

    /// Output settings
    void displaySettings();

    /// Getter methods
    bool isDataValid() {return dataValid;}
    double getPixelSize() {return pixelSize;}
    int getRawImageWidth() {return rawImageWidth;}
    int getRawImageHeight() {return rawImageHeight;}
    std::string getDirTotalFocusImages() {return dirTotalFocusImages;}
    std::string getDirMlCalibFile() {return dirMlCalibFile;}
    std::string getDirDepthData() {return dirDepthData;}
    int getNumberPoints() {return numberPoints;}
    int getMarkerDictionary() {return markerDictionary;}
    int getNRadialDistParam() {return nRadialDistParam;}
    bool getTangentialDistParam() {return tangentialDistParam;}
    bool getMlCenterAdjustment() {return mlCenterAdjustment;}
    bool getRefinePoses() {return refinePoses;}
    bool getRefine3Dpoints() {return refine3Dpoints;}
    bool getUseRobustCostFunction() {return useRobustCostFunction;}

private:
    template<typename T>
    T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found,const bool required = true)
    {
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist." << std::endl;
                found = false;
                return T();
            }

        }
        else{
            found = true;
            return (T) node;
        }
    }

    bool readSettingsFromFile(cv::FileStorage& fSettings);
    bool stringToBool(std::string inputString, bool &boolVal);

    /// Specifies if all the data from the file are valid
    bool dataValid;

    /// Size of a pixel in mm
    double pixelSize;
    /// Width of the raw image in pixels
    int rawImageWidth;
    /// Height of the raw image in pixels
    int rawImageHeight;

    /// Path to total focus images.
    std::string dirTotalFocusImages;
    /// Path to micro lens calibration file
    std::string dirMlCalibFile;
    /// Path to virtual depth data
    std::string dirDepthData;

    /// Number of 3D points used for the light field bundle adjustment
    int numberPoints;
    /// Number of the used OpenCV marker dictionary
    int markerDictionary;

    /// Number of radial distortion parameters
    int nRadialDistParam;
    /// Define tangential distorsion
    bool tangentialDistParam;
    /// adjust micro lens centers (since micro lens centers are different from estimated micro image centers)
    bool mlCenterAdjustment;
    /// Camera pose are optimized
    bool refinePoses;
    /// 3D object points are optimized
    bool refine3Dpoints;
    /// Robust cost function has to be used
    bool useRobustCostFunction;

    const std::string settingNameFile_pixelSize = "Camera.pixelSize";
    const std::string settingNameFile_rawImageWidth = "Camera.rawImageWidth";
    const std::string settingNameFile_rawImageHeight = "Camera.rawImageHeight";
    const std::string settingNameFile_dirTotalFocusImages = "Path.totalFocusImages";
    const std::string settingNameFile_dirMlCalibFile = "Path.microLensCalibration";
    const std::string settingNameFile_dirDepthData = "Path.virtualDepthData";
    const std::string settingNameFile_numberPoints = "Config.numberPoints";
    const std::string settingNameFile_markerDictionary = "Config.markerDictionary";
    const std::string settingNameFile_nRadialDistParam = "Model.numberRadialDistParam";
    const std::string settingNameFile_tangentialDistParam = "Model.tangentialDist";
    const std::string settingNameFile_mlCenterAdjustment = "Model.adjustMicroLensCenters";
    const std::string settingNameFile_refinePoses = "Model.refineExtrinsicOrientations";
    const std::string settingNameFile_refine3Dpoints = "Model.refineCoordinatesPoints";
    const std::string settingNameFile_useRobustCostFunction = "Model.robustCostFunction";
};
