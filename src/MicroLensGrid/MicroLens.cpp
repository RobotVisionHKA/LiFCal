/**
 * @brief Definition of the class MicroLens.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file MicroLens.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include "MicroLens.h"


MicroLens::MicroLens(unsigned int idx, unsigned int lensType, float centerX, float centerY)
{
	this->idx = idx;
	this->lensType = lensType;
	this->centerX = centerX;
	this->centerY = centerY;
}


MicroLens::~MicroLens(void)
{
}
