/**
 * @brief Declaration of the class Images.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file Images.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <vector>
#include <map>
#include <unordered_map>

#include "colmap/util/types.h"
#include "colmap/scene/image.h"


class Image;

class Images
{
public:
	Images(void);
	~Images(void);

	/// Load image coordinates data from the map
	bool LoadImageCoordinates(std::unordered_map<colmap::image_t, class colmap::Image> map_images);

	/// Vector of all images
	std::vector<Image*> images;
	/// Images by image ID
	std::map<int,Image*> imagesByID;

	std::vector<int> imagesIDOrder;
};
