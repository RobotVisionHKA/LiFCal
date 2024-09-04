/**
 * @brief Declaration of the classes Constraints and Constraint.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file Constraints.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <Eigen/Dense>


class Constraint;

class Constraints
{
public:
	Constraints(std::string dirConstraintsFile);
	~Constraints(void);

	/// Read constraints from file.
	bool ReadConstraints(void);
	/// List of the IDs of the constraint points
	std::vector<int> ConstraintsPointsIds;
	/// Vector of all object points.
	std::vector<Constraint> ConstraintList;

private:
	/// Path to the file defining the constraints
	std::string pathConstraintsFile;
};

class Constraint
{
public:
	Constraint(int pointID_1_in, int pointID_2_in, double distance_in, double sigma_in):
		pointID_1(pointID_1_in), pointID_2(pointID_2_in), distance(distance_in), sigma(sigma_in){};
	~Constraint(void){};

	/// object point ID
	int pointID_1, pointID_2;
	double distance;
	double sigma;
};

