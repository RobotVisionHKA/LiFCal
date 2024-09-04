/**
 * @brief Definition of the class EpiPolarLine.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file EpiPolarLine.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <math.h>

#include "EpiPolarLine.h"


EpiPolarLine::EpiPolarLine(double eplX, double eplY, double baseLineDist, float minVirtualDepth)
{
	epiLine[0] = eplX;
	epiLine[1] = eplY;

	double vecLength_2 = epiLine[0]*epiLine[0]+epiLine[1]*epiLine[1];
	if(vecLength_2 != 1.0f)
	{
		double vecLength = sqrt(vecLength_2);
		epiLine[0] /=vecLength;
		epiLine[1] /=vecLength;
	}

	this->baseLineDist = baseLineDist;
	this->minVirtualDepth = minVirtualDepth;
}


EpiPolarLine::~EpiPolarLine(void)
{
}

EpiPolarLine* EpiPolarLine::add(EpiPolarLine other)
{
	double x_new = this->epiLine[0]*this->baseLineDist+other.epiLine[0]*other.baseLineDist;
	double y_new = this->epiLine[1]*this->baseLineDist+other.epiLine[1]*other.baseLineDist;

	double baseLineDist_new = sqrt(x_new*x_new+y_new*y_new);

	return new EpiPolarLine(x_new,y_new,baseLineDist_new,2.0f);
}
