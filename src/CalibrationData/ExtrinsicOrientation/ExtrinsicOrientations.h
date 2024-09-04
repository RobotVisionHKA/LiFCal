/**
 * @brief Declaration of the class ExtrinsicOrientations.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ExtrinsicOrientations.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <vector>
#include <map>

#include <colmap/scene/reconstruction.h>

class ExtrinsicOrientation;

class ExtrinsicOrientations
{
public:
	ExtrinsicOrientations(void);
	~ExtrinsicOrientations(void);

	/// Load extrinsic orientation data from the map.
	bool LoadExtrinsicOrientations(std::unordered_map<colmap::image_t, class colmap::Image> map_images);
	/// Vector of extrinsic orientations.
	std::vector<ExtrinsicOrientation*> extOrientations;
	/// Extrinsic orientations by image ID.
	std::map<int,ExtrinsicOrientation*> extOrientationsByImageID;
};
