/**
 * @brief Declaration of the class MicroLens.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file MicroLens.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once


class MicroLens
{
public:
	MicroLens(unsigned int idx, unsigned int lensType, float centerX, float centerY);
	~MicroLens(void);

	unsigned int idx;
	unsigned int lensType;
	float centerX;
	float centerY;
};

