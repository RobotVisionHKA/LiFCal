/**
 * @brief Declaration of the class EpiPolarLine.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file EpiPolarLine.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once


class EpiPolarLine
{
public:
	double epiLine[2];
	double baseLineDist;
	double minVirtualDepth;

	EpiPolarLine(double eplX, double eplY, double baseLineDist, float minVirtualDepth);
	~EpiPolarLine(void);

	EpiPolarLine* add(EpiPolarLine other);
};
