/**
 * @brief Definition of the class MicroLensGrid.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file MicroLensGrid.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#define _USE_MATH_DEFINES

#include <stdio.h>
#include <string>
#include <math.h> 
#include <vector>

#include <thirdparty/pugixml-1.7/src/pugixml.hpp>
#include <thirdparty/pugixml-1.7/src/pugiconfig.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "MicroLensGrid/MicroLens.h"
#include "MicroLensGrid/MicroLensGrid.h"


MicroLensGrid::MicroLensGrid(void)
{
	validGridData = false;
	mapMlId = NULL;
	mapMlPointer = NULL;
	mapNextMl = NULL;
	mapMlType = NULL;
	mapValidMlPixel = NULL;
}


MicroLensGrid::~MicroLensGrid(void)
{
	for(int i=mlLensList.size()-1;i>=0;i--)
	{
		delete mlLensList[i];
		mlLensList.pop_back();
	}

	if(mapMlId != NULL) delete[] mapMlId;
	if(mapMlPointer != NULL) delete[] mapMlPointer;
	if(mapNextMl != NULL) delete[] mapNextMl;
	if(mapMlType != NULL) delete[] mapMlType;
	if(mapValidMlPixel != NULL) delete[] mapValidMlPixel;

}

bool MicroLensGrid::readInGrid(std::string sMicroLensGridDataPath, int rawImWidth, int rawImHeight, bool doRotationOnGrid)
{
	printf("Read in micro lens grid ...\n");

	width = rawImWidth;
	height = rawImHeight;
	imCenter[0] = ((float)width)/2.0f-0.5f;
	imCenter[1] = ((float)height)/2.0f-0.5f;

	this->rotationOnGrid = doRotationOnGrid;

	pugi::xml_document doc;
	doc.load_file(sMicroLensGridDataPath.c_str());
	if(doc == NULL)
	{
		printf("Could not open %s.\n", sMicroLensGridDataPath.c_str());
		return false;
	}

	// Search for ray-image calibration data
	pugi::xml_node rayCalibData = doc.child("RayCalibData");
	if(rayCalibData == NULL)
	{
		printf("Xml-file has no node named RayCalibData.\n");
		return false;
	}

	// Read grid offset
	pugi::xml_node offsetNode, offsetNodeX, offsetNodeY;
	offsetNode = rayCalibData.child("offset");
	offsetNodeX = offsetNode.child("x");
	offsetNodeY = offsetNode.child("y");
	offset[0] = offsetNodeX.text().as_float();
	offset[1] = offsetNodeY.text().as_float();

	// rRead micro lens diameter
	pugi::xml_node diameterNode;
	diameterNode = rayCalibData.child("diameter");
	lensDiameter = diameterNode.text().as_float();

	// Read rotation angle
	pugi::xml_node rotationNode;
	rotationNode = rayCalibData.child("rotation");
	rotation = rotationNode.text().as_float();

	// Read lens boarder
	pugi::xml_node lensBorderNode;
	float lensBorder;
	lensBorderNode = rayCalibData.child("lens_border");
	lensBorder = lensBorderNode.text().as_float();

	// Set lens boarder
	lensBorder = 1.0f;

	lensValidityRadius = lensDiameter*0.5f-lensBorder;
	lensValidityRadius_2 = lensValidityRadius*lensValidityRadius;

	// Read total covering plane
	pugi::xml_node tcpNode;
	tcpNode = rayCalibData.child("tcp");
	totalCoveringPlane = tcpNode.text().as_float();

	// Read micro lens grid bases
	// Lens base x
	pugi::xml_node lensBaseXNode, lensBaseXNodeX, lensBaseXNodeY;
	lensBaseXNode = rayCalibData.child("lens_base_x");
	lensBaseXNodeX = lensBaseXNode.child("x");
	lensBaseXNodeY = lensBaseXNode.child("y");
	lensBaseX[0] = lensBaseXNodeX.text().as_float();
	lensBaseX[1] = lensBaseXNodeY.text().as_float();

	// Lens base y
	pugi::xml_node lensBaseYNode, lensBaseYNodeX, lensBaseYNodeY;
	lensBaseYNode = rayCalibData.child("lens_base_y");
	lensBaseYNodeX = lensBaseYNode.child("x");
	lensBaseYNodeY = lensBaseYNode.child("y");
	lensBaseY[0] = lensBaseYNodeX.text().as_float();
	lensBaseY[1] = lensBaseYNodeY.text().as_float();

	// Sub grid base
	pugi::xml_node subGridNode, subGridNodeX, subGridNodeY;
	subGridNode = rayCalibData.child("sub_grid_base");
	subGridNodeX = subGridNode.child("x");
	subGridNodeY = subGridNode.child("y");
	subGridBase[0] = subGridNodeX.text().as_float();
	subGridBase[1] = subGridNodeY.text().as_float();

	pugi::xml_node lensTypeNode, lensTypeOffsetNode, lensTypeOffsetNodeX, lensTypeOffsetNodeY,
		lensTypeDepthRangeNode, lensTypeDepthRangeNodeMin, lensTypeDepthRangeNodeMax;
	lensTypeNode = rayCalibData.child("lens_type");
	for(int i=0; i<3; i++)
	{
		int id = lensTypeNode.attribute("id").as_int();
		lensTypeOffsetNode = lensTypeNode.child("offset");
		lensTypeOffsetNodeX = lensTypeOffsetNode.child("x");
		lensTypeOffsetNodeY = lensTypeOffsetNode.child("y");
		lensType[id].offset[0] = lensTypeOffsetNodeX.text().as_float();
		lensType[id].offset[1] = lensTypeOffsetNodeY.text().as_float();

		lensTypeDepthRangeNode = lensTypeNode.child("depth_range");
		lensTypeDepthRangeNodeMin = lensTypeDepthRangeNode.child("min");
		lensTypeDepthRangeNodeMax = lensTypeDepthRangeNode.child("max");
		lensType[id].vDepthMin = lensTypeDepthRangeNodeMin.text().as_float();
		lensType[id].vDepthMax = lensTypeDepthRangeNodeMax.text().as_float();

		lensTypeNode = lensTypeNode.next_sibling("lens_type");
	}

	// Grid offset in opencv coordinates
	offsetOpenCV[0] = offset[0]+imCenter[0];
	offsetOpenCV[1] = -offset[1]+imCenter[1];

	validGridData = true;

	createGrid(this->rotationOnGrid);

	printf("done.\n");
	return true;
}

void MicroLensGrid::alignImageToGrid(cv::Mat rawImageInput, cv::Mat &rawImageAligned)
{
	if(!this->rotationOnGrid) {
		cv::Mat rotMat = cv::getRotationMatrix2D(cv::Point2f(offsetOpenCV[0],offsetOpenCV[1]),-rotation*180.0f*M_1_PI,1.0f);
		cv::warpAffine(rawImageInput,rawImageAligned,rotMat,rawImageInput.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,0);
	}
	else rawImageAligned = rawImageInput.clone();

}

void MicroLensGrid::createGrid(bool doRotationOnGrid)
{
	// Defining lens grid
	// We allow micro lens centers to be outside the sensor region
	float xImMin = -imCenter[0]-offset[0]-lensDiameter/2.0f;
	float xImMax = imCenter[0]-offset[0]+lensDiameter/2.0f;

	float yImMin = -imCenter[1]-offset[1]-lensDiameter/2.0f;
	float yImMax = imCenter[1]-offset[1]+lensDiameter/2.0f;

	// Lens grid margins
	// Grid 1
	int xLensMinGrid1 = ceil(xImMin/lensDiameter);
	int xLensMaxGrid1 = xImMax/lensDiameter;

	int yLensMinGrid1 = ceil(yImMin/(2.0f*lensBaseY[1]*lensDiameter));
	int yLensMaxGrid1 = yImMax/(2.0f*lensBaseY[1]*lensDiameter);

	// Grid 2
	int xLensMinGrid2 = ceil(xImMin/lensDiameter-lensBaseY[0]-1.0f);
	int xLensMaxGrid2 = xImMax/lensDiameter-lensBaseY[0]-1.0f;

	int yLensMinGrid2 = ceil(yImMin/(2.0f*lensBaseY[1]*lensDiameter) -0.5f);
	int yLensMaxGrid2 = yImMax/(2.0f*lensBaseY[1]*lensDiameter) -0.5f;

	nMicroLenses = (xLensMaxGrid1-xLensMinGrid1+1)*(yLensMaxGrid1-yLensMinGrid1+1);
	nMicroLenses += (xLensMaxGrid2-xLensMinGrid2+1)*(yLensMaxGrid2-yLensMinGrid2+1);

	mlLensList.resize(nMicroLenses);

	int lensId = 0;

	unsigned int subGrid, subGridCoorX, subGridCorrY;
	int lensType;
	float centerX, centerY;
	float centerXTmp, centerYTmp;
	float cosAlpha, sinAlpha;

	if(doRotationOnGrid) {
		cosAlpha = cos(this->rotation);
		sinAlpha = sin(this->rotation);
	}

	for(int x=xLensMinGrid1; x<=xLensMaxGrid1; x++)
	{
		lensType = x%3;
		if(lensType<0)lensType+=3;

		if(doRotationOnGrid) centerXTmp = (float)x*lensDiameter;
		else centerX = offsetOpenCV[0] + (float)x*lensDiameter;

		for(int y=yLensMinGrid1; y<=yLensMaxGrid1; y++, lensId++)
		{
			if(doRotationOnGrid) {
				centerYTmp = (float)y*lensDiameter*2.0f*lensBaseY[1];
				centerX = offsetOpenCV[0] + (centerXTmp*cosAlpha - centerYTmp*sinAlpha);
				centerY = offsetOpenCV[1] - (centerXTmp*sinAlpha + centerYTmp*cosAlpha);
			}
			else centerY = offsetOpenCV[1] - (float)y*lensDiameter*2.0f*lensBaseY[1];
			
			mlLensList[lensId] = new MicroLens(lensId,(unsigned int)lensType,centerX,centerY);
		}
	}

	for(int x=xLensMinGrid2; x<=xLensMaxGrid2; x++)
	{
		lensType = x%3;
		if(lensType<0)lensType+=3;

		if(doRotationOnGrid) centerXTmp = ((float)x+1.0f+lensBaseY[0])*lensDiameter;
		else centerX = offsetOpenCV[0] +((float)x+1.0f+lensBaseY[0])*lensDiameter;		

		for(int y=yLensMinGrid2; y<=yLensMaxGrid2; y++, lensId++)
		{
			if(doRotationOnGrid) {
				centerYTmp = (((float)y*2.0f+1.0f)*lensBaseY[1])*lensDiameter;
				centerX = offsetOpenCV[0] + (centerXTmp*cosAlpha - centerYTmp*sinAlpha);
				centerY = offsetOpenCV[1] - (centerXTmp*sinAlpha + centerYTmp*cosAlpha);
			}
			else centerY = offsetOpenCV[1] -(((float)y*2.0f+1.0f)*lensBaseY[1])*lensDiameter;

			mlLensList[lensId] = new MicroLens(lensId,(unsigned int)lensType,centerX,centerY);
		}
	}
}


void MicroLensGrid::debugPlotMlGrid(cv::Mat rawImage)
{
	rawImage.convertTo(debugImageMlGrid, CV_8UC1);
	cv::cvtColor(debugImageMlGrid,debugImageMlGrid,cv::COLOR_GRAY2RGB);
	
	int bitShift = 16;
	int scale = 1<<bitShift;

	float radius = lensDiameter*0.5f;
	cv::Scalar color;

	cv::line(debugImageMlGrid,cv::Point2f(offsetOpenCV[0]*scale,0.0f),cv::Point2f(offsetOpenCV[0]*scale,height*scale),cv::Scalar(0,0,255),3,cv::LINE_AA,bitShift);
	cv::line(debugImageMlGrid,cv::Point2f(0.0f,offsetOpenCV[1]*scale),cv::Point2f(width*scale,offsetOpenCV[1]*scale),cv::Scalar(0,0,255),3,cv::LINE_AA,bitShift);

	for(int i=0;i<mlLensList.size();i++)
	{
		switch(mlLensList[i]->lensType)
		{
		case 0:
			color = cv::Scalar(0,0,255);
			break;
		case 1:
			color = cv::Scalar(0,255,0);
			break;
		case 2:
			color = cv::Scalar(255,0,0);
			break;
		default:
			break;
		}
		cv::circle(debugImageMlGrid,cv::Point2f(mlLensList[i]->centerX*scale,mlLensList[i]->centerY*scale),radius*scale,color,1,cv::LINE_AA,bitShift);
		cv::circle(debugImageMlGrid,cv::Point2f(mlLensList[i]->centerX*scale,mlLensList[i]->centerY*scale),scale,color,3,cv::LINE_AA,bitShift);
	}
}


void MicroLensGrid::debugPlotMlType()
{
	debugImageMlType = cv::Mat(height,width,CV_8UC3);

	for(int y=0;y<height;y++)
	{
		for(int x=0;x<width;x++)
		{
			int idx = x + y*width;

			switch(mapMlType[idx])
			{
			case 0:
				debugImageMlType.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,255);
				break;
			case 1:
				debugImageMlType.at<cv::Vec3b>(y,x) = cv::Vec3b(0,255,0);
				break;
			case 2:
				debugImageMlType.at<cv::Vec3b>(y,x) = cv::Vec3b(255,0,0);
				break;
			default:
				debugImageMlType.at<cv::Vec3b>(y,x) = cv::Vec3b(0,0,0);
				break;
			}
		}
	}
}

void MicroLensGrid::defineMlMaps()
{
	if(mapMlId==NULL) mapMlId = new int[width*height];
	if(mapMlPointer==NULL) mapMlPointer = new MicroLens*[width*height];
	if(mapNextMl==NULL) mapNextMl = new MicroLens*[width*height];
	if(mapMlType==NULL) mapMlType = new int[width*height];
	if(mapValidMlPixel==NULL) mapValidMlPixel = new bool[width*height];
	for(int i=0;i<width*height;i++)
	{
		mapMlId[i] = -1;
		mapMlPointer[i] = NULL;
		mapNextMl[i] = NULL;
		mapMlType[i] = -1;
		mapValidMlPixel[i] = false;
	}

	for(int i=0;i<mlLensList.size();i++)
	{
		double centerX = mlLensList[i]->centerX;
		double centerY = mlLensList[i]->centerY;
		int lensType = mlLensList[i]->lensType;
		
		for(int y = ceil(centerY-lensValidityRadius);y<=centerY+lensValidityRadius;y++)
		{
			// Check if the calculated y coordinate is outside of the image region
			if(y<0 || y>=height) continue;
			double y_2 = (y-centerY)*(y-centerY);
			for(int x = ceil(centerX-sqrt(lensValidityRadius_2-y_2));(x-centerX)*(x-centerX)<=lensValidityRadius_2-y_2;x++)
			{
				// Check if the calculated x coordinate is outside of the image region
				if(x<0 || x>=width) continue;
				int idx = x+y*width;
				mapMlId[idx] = i;
				mapMlPointer[idx] = mlLensList[i];
				mapNextMl[idx] = mlLensList[i];
				mapMlType[idx] = lensType;
				mapValidMlPixel[idx] = true;
			}
		}
	}

	for(int y=0; y<height; y++)
	{
		for(int x=0; x<width; x++)
		{
			int idx = x+y*width;
			MicroLens** pML_src = mapMlPointer+idx;
			MicroLens** pML_dst = mapNextMl+idx;
			if(*pML_dst!=NULL) continue;

			float dist_2 = -1;			
			for(int d=1;;d++)
			{
				for(int dx=-d;dx<=d;dx++)
				{
					if(x+dx<0) continue;
					if(x+dx>=width) break;

					int idx_tmp;

					for(int dy=-d;dy<=d;dy++)
					{
						if(dx!=-d && dx!=d && dy!=-d && dy!=d) continue;
						if(y+dy<0) continue;
						if(y+dy>=height) break;
						
						MicroLens** pML_new = pML_src + dx +dy*width;
						if(*pML_new != NULL) {
							float centerX = (*pML_new)->centerX;
							float centerY = (*pML_new)->centerY;
							float dist_2_new = (centerX-x)*(centerX-x)+(centerY-y)*(centerY-y);

							if(dist_2_new<dist_2 || dist_2<0) {
								*pML_dst = *pML_new;
								dist_2 = dist_2_new;
							}					
						}
					}
				}
				if(*pML_dst != NULL) break;
			}
		}
	}
}
