/**
 * @brief Definition of the class ObjectPoints.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ObjectPoints.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>

#include "CalibrationData/ObjectPoints/ObjectPoints.h"


ObjectPoints::ObjectPoints(void){}
ObjectPoints::~ObjectPoints(void){}

/**
 * @brief Load object point data from the map.
 */
bool ObjectPoints::LoadObjectPoints(std::unordered_map<colmap::point3D_t, class colmap::Point3D> map_point3D)
{
	if(map_point3D.empty()) return false;
	else
	{
		int pointID, nRays;

		for(auto it = map_point3D.begin(); it != map_point3D.end(); it++)
		{
			pointID = it->first;
			nRays = it->second.Track().Length();
			if(this->objPointsByPointID.count(pointID) == 0)
			{
				ObjectPoint* objPt = new ObjectPoint(pointID, it->second.XYZ(), it->second.Error(), nRays);
				objPointsByPointID[pointID] = objPt;
				objPoints.push_back(objPt);
			}
			else
			{
				// This point should never be reached
				printf("something went wrong, object point for the id %d does already exist.\n", pointID);
			}
		}
	}

	printf("\t-> %lu points were found.\n", objPoints.size());
	return true;
}
