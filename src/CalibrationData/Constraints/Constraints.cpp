/**
 * @brief Definition of the class Constraints.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file Constraints.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <stdio.h>

#include "CalibrationData/Constraints/Constraints.h"


Constraints::Constraints(std::string dirConstraintsFile)
{
	pathConstraintsFile = dirConstraintsFile;
}
Constraints::~Constraints(void){}

/**
 * @brief Read constraints from file.
 */
bool Constraints::ReadConstraints(void)
{
	int pointID_1, pointID_2;
	double distance, sigma;

	ConstraintList.clear();
	ConstraintList.reserve(10);

	std::string line;
	// Open file in read mode
	std::ifstream constraintsFile(pathConstraintsFile);

	if (!constraintsFile.is_open()) return false;
	else
	{
		// Read each line
		while (getline(constraintsFile, line))
		{
			if (line.empty() || line[0] == '#')
			{
				continue;
			}

			std::istringstream iss(line);
			iss >> pointID_1 >> pointID_2 >> distance >> sigma;
			this->ConstraintList.push_back(Constraint(pointID_1, pointID_2, distance, sigma));
			// Add id of the marker to the list if it is not allready in it
			if(std::find(this->ConstraintsPointsIds.begin(), this->ConstraintsPointsIds.end(), pointID_1) == this->ConstraintsPointsIds.end()) {this->ConstraintsPointsIds.push_back(pointID_1);}
			if(std::find(this->ConstraintsPointsIds.begin(), this->ConstraintsPointsIds.end(), pointID_2) == this->ConstraintsPointsIds.end()) {this->ConstraintsPointsIds.push_back(pointID_2);}
		}
		constraintsFile.close();
	}

	printf("\t-> %lu constraints were found.\n", ConstraintList.size());
	return true;
}
