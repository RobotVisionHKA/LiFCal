/**
 * @brief Declaration of the namespace fileManagement.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file fileManagement.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/algorithm/string.hpp"
#include <iostream>
#include <vector>


namespace fileManagement
{
	enum CopyOption
	{
		OverwriteExistingFile,
		KeepExistingFile
	};

	bool Exists(std::string sPath);

	void DisplayFiles(std::string sPath, std::string sExtension);
	bool SelectFile(std::string sPath, std::string sExtension, std::string &sOutputPath );
	bool GetFileList(std::string sPath, std::string sExtension, std::vector<std::string> &vFileList);
	bool CreateDir(std::string sPath );
	
	std::string getFilePathWithoutExt(std::string sPath);
	std::string getExtension(std::string sPath);
	bool checkExtension(std::string sPath, std::string sExtRef);

	bool FileCopy(std::string sSource, std::string sDestination, fileManagement::CopyOption option);
}
