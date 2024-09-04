/**
 * @brief Definition of the class myUtility.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file myUtility.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <iostream>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "myUtility.h"


bool myUtility::askYesNo( std::string sQuestion )
{
	sQuestion += " (y/n)\n";
	printf("%s", sQuestion.c_str());
	std::string sYesNo;
	std::cin >> sYesNo;

	while((sYesNo!="n") && (sYesNo!="N") && (sYesNo!="y") && (sYesNo!="Y"))
	{
		printf("Invalid input! Please enter again. (y/n)\n");
		std::cin >> sYesNo;
	}

	printf("\n");

	if((sYesNo=="y") || (sYesNo=="Y")){
		return true;
	}
	else{
		return false;
	}
}

std::string myUtility::getTimeStampString()
{
	boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
	facet->format("%Y_%m_%d_%H%M%S");
	boost::posix_time::ptime boostTimeStap = boost::posix_time::second_clock::local_time();
	std::stringstream streamTimeStamp;
	streamTimeStamp.imbue(std::locale(std::locale::classic(), facet));
	streamTimeStamp << boostTimeStap;
	std::string sTimeStamp = streamTimeStamp.str();

	return sTimeStamp;
}

void myUtility::displayHelp()
{
	printf("\nUsage:\n\n"
			"   For a calibration with Aruco markers:\n"
			"      ./LiFCal calib_marker path_to_settings path_to_constraints\n\n"
			"   For an online recalibration:\n"
			"      ./LiFCal recalib path_to_settings path_to_fixed_parameters\n\n");
}

void myUtility::createOrClearDirectory(const std::string& path)
{
	// If directory allread exist replace clear it
	if (boost::filesystem::is_directory(path))
	{
		boost::filesystem::remove_all(path);
	}
	boost::filesystem::create_directory(path);
}
