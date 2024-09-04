/**
 * @brief Definition of the class Images.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file Images.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>

#include "CalibrationData/ImagePoints/Image.h"
#include "CalibrationData/ImagePoints/Images.h"


Images::Images(void)
{
}


Images::~Images(void)
{
}

/**
 * @brief Load image coordinates data from the map.
 */
bool Images::LoadImageCoordinates(std::unordered_map<colmap::image_t, class colmap::Image> map_images)
{
	if(map_images.empty()) return false;
	else
	{
		int totalNumberOfCoordinates = 0;
		int totalNumberOfInliers = 0;
		int pointID, imageID;
		bool isOutlier;

		for(auto it = map_images.begin(); it != map_images.end(); it++)
		{
			imageID = it->second.ImageId();
			this->imagesIDOrder.push_back(imageID);

			for(int ipoint = 0; ipoint < it->second.Points2D().size(); ipoint++)
			{
				pointID = it->second.Point2D(ipoint).point3D_id;
				// Check if point was classified as outlier or not
				if(pointID == -1) isOutlier = true;
				else isOutlier = false;

				Image* currentImage;
				if(this->imagesByID.count(imageID) == 0)
				{
					currentImage = new Image(imageID);
					this->imagesByID[imageID] = currentImage;
					this->images.push_back(currentImage);
				}
				else currentImage = this->imagesByID[imageID];

				if(currentImage->imageCoordinatesByPointID.count(pointID) == 0)
				{
					ImageCoordinate* imPt = new ImageCoordinate(pointID, imageID, it->second.Point2D(ipoint).xy, isOutlier);
					currentImage->imageCoordinatesByPointID[pointID] = imPt;
					currentImage->imageCoordinates.push_back(imPt);

					if(!isOutlier)
					{
						currentImage->imageCoordinatesInliers.push_back(imPt);
						totalNumberOfInliers++;
					}

					totalNumberOfCoordinates++;
				}
				else
				{
					// This point can be readed if there occure outliers for the current point ID
					// if the existing point is an outlier and the new one not we replace the existing point by the new one.
					if(!isOutlier)
					{
						if(currentImage->imageCoordinatesByPointID[pointID]->isOutlier())
						{
							// replace old point
							ImageCoordinate imPt(pointID, imageID, it->second.Point2D(ipoint).xy, isOutlier);
							memcpy(currentImage->imageCoordinatesByPointID[pointID], &imPt, sizeof(ImageCoordinate));
							currentImage->imageCoordinatesInliers.push_back(currentImage->imageCoordinatesByPointID[pointID]);
							totalNumberOfInliers++;
						}
						else
						{
							printf("There are two coordinates which have the same ID and are both no outliers (Image ID: %d, Point ID: %d)\n One point is neglected.\n", imageID, pointID);
						}
					}
				}
			}

		}

		printf("\t-> %d points (%d inliers) were found.\n", totalNumberOfCoordinates, totalNumberOfInliers);
		return true;
	}
}
