/**
 * @brief Declaration of the class myUtility.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file myUtility.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <string>
#include <boost/filesystem.hpp>


namespace myUtility
{
	bool askYesNo(std::string sQuestion);

	std::string getTimeStampString();

	void displayHelp();

	void createOrClearDirectory(const std::string& path);
}
