/**
 * @brief Main file of the code.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file main.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include "CameraModel.h"
#include "CameraCalibration.h"
#include "Utility/myUtility/myUtility.h"


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        printf("Wrong number of parameters given.\n");
		myUtility::displayHelp();
        return 1;
    }

	std::string type_of_calibration = std::string(argv[1]);
	std::string settingsFile = std::string(argv[2]);
	std::string constraintsFile = std::string(argv[3]);
	// Run camera calibration
	// Parameters have to be entered in the yaml file
	CameraCalibration(type_of_calibration, settingsFile, constraintsFile);

	return 0;
}
