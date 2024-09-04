/**
 * @brief Definition of the class ExtrinsicOrientations.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ExtrinsicOrientations.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>

#include "CalibrationData/ExtrinsicOrientation/ExtrinsicOrientations.h"
#include "CalibrationData/ExtrinsicOrientation/ExtrinsicOrientation.h"


ExtrinsicOrientations::ExtrinsicOrientations(void)
{
}

ExtrinsicOrientations::~ExtrinsicOrientations(void)
{
	for(int i=0; i< this->extOrientations.size(); i++)
		delete extOrientations[i];
}

/**
 * \brief Load extrinsic orientation data from the map.
 */
bool ExtrinsicOrientations::LoadExtrinsicOrientations(std::unordered_map<colmap::image_t, class colmap::Image> map_images)
{
	if(map_images.empty()) return false;
	else
	{
		for(auto it = map_images.begin(); it != map_images.end(); it++)
		{	
			colmap::image_t imageID = it->second.ImageId();
			if(this->extOrientationsByImageID.count(imageID) == 0)
			{
				ExtrinsicOrientation* extOr = new ExtrinsicOrientation(it->second.ImageId(),
																		it->second.CamFromWorld(),
																		it->second.CameraId(),
																		it->second.Name());
				extOrientationsByImageID[imageID] = extOr;
				extOrientations.push_back(extOr);
			}
			else
			{
				// This point should never be reached
				printf("something went wrong, exterior orientation for image id %d (%s) does already exist.\n", imageID, it->second.Name().c_str());
			}
		}

	}

	printf("\t-> %lu orientations were found.\n",extOrientations.size());
	return true;
}
