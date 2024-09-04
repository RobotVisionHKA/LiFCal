/**
 * @brief Declaration of the class ColmapReconstructor.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ColmapReconstructor.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <iostream>

#include "colmap/controllers/automatic_reconstruction.h"
#include "colmap/util/misc.h"


class ColmapReconstructor
{
public:
    /// Set the variables of the class and create output directory.
	ColmapReconstructor();

	~ColmapReconstructor();

    /// Use of colmap for an initial reconstruction from total focus images.
    bool reconstructionColmap(std::string dirImages, std::string &dirOutput);

private:
    /// Saving colmap's estimated data in human-readable *.txt format.
    bool saveTextFiles();
    
    /// Options
    std::string dirWorkspace;
    std::string dirOutputFiles;
    std::string dirImages;
    std::string dataType;
    std::string quality;
    std::string mesher;
    std::string cameraModel;
    bool dense;
    bool singleCamera;
};
