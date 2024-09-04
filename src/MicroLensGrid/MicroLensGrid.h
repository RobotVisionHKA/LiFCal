/**
 * @brief Declaration of the class MicroLensGrid.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file MicroLensGrid.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


struct LensTypeStruct
{
	float offset[2];
	float vDepthMin;
	float vDepthMax;
};

class MicroLens;

class MicroLensGrid
{
public:
	MicroLensGrid(void);
	~MicroLensGrid(void);

	bool readInGrid(std::string sMicroLensGridDataPath, int rawImWidth, int rawImHeight, bool doRotationOnGrid = true);
	void alignImageToGrid(cv::Mat rawImageInput, cv::Mat &rawImageAligned);
	void defineMlMaps();

	bool isRotationOnGrid(){return this->rotationOnGrid;};

	void debugPlotMlGrid(cv::Mat rawImage);
	void debugPlotMlType();

	cv::Mat debugImageMlGrid;
	cv::Mat debugImageMlType;
	
	bool validGridData;

	int width, height;
	float imCenter[2];

	/// Micro lens grid offset from image center (in pixels)
	float offset[2];
	float offsetOpenCV[2];
	/// Micro lens diameter (in pixels)
	float lensDiameter;
	/// Micro lens grid rotation angle (in radians)
	float rotation;
	/// Boarder around micro lens (in pixels)
	float lensBorder;
	/// Radius of valid pixels around the microlens center (in pixels)
	float lensValidityRadius;
	/// Squared validity radius
	float lensValidityRadius_2;
	/// Distance of total covering plane (in virtual depths)
	float totalCoveringPlane;
	/// Grid base (in micro lens diameters)
	float lensBaseX[2];
	float lensBaseY[2];
	float subGridBase[2];
	/// Lens type grids
	LensTypeStruct lensType[3];

	int nMicroLenses;
	std::vector<MicroLens*> mlLensList;

	int* mapMlId;
	int* mapMlType;
	bool* mapValidMlPixel;
	MicroLens** mapMlPointer;
	MicroLens** mapNextMl;

private:
	void createGrid(bool doRotationOnGrid);

	bool rotationOnGrid;
};
