/**
 * @brief Definition of the functions in the namespace fileManagement.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file fileManagement.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include "Utility/FileManagement.h"


bool fileManagement::Exists( std::string sPath )
{
	return boost::filesystem::exists(sPath);
}

void fileManagement::DisplayFiles( std::string sPath, std::string sExtension)
{
	if( !boost::filesystem::exists(sPath) || !boost::filesystem::is_directory(sPath) )
	{
		std::cout << "The given path is not a valid directory." << std::endl << std::endl;
		return;
	}
	
	boost::filesystem::directory_iterator end_iter;

	std::cout << sExtension <<"-Files in the directory: " << std::endl << sPath << ":" << std::endl;
	int i = 0;
	for( boost::filesystem::directory_iterator dir_iter( sPath ) ; dir_iter != end_iter ; ++dir_iter)
	{
		if( boost::iequals( sExtension, dir_iter->path().extension().string() ) )
		{
			std::cout << i << ":\t" << dir_iter->path().filename().string() << std::endl;
			i++;
		}
    }
	if( i == 0)
		std::cout << "There is no " << sExtension <<"-Files in the given directory."  << std::endl;

	std::cout << std::endl;
}


bool fileManagement::SelectFile( std::string sPath, std::string sExtension, std::string &sOutputPath )
{
	if( !boost::filesystem::exists(sPath) || !boost::filesystem::is_directory(sPath) )
	{
		std::cout << "The given path is not a valid directory." << std::endl << std::endl;
		return false;
	}
	
	std::vector<std::string> vFiles;
	boost::filesystem::directory_iterator end_iter;
	std::cout << sExtension <<"-Files in the directory: " << std::endl << sPath << ":" << std::endl;
	int i = 0;
	for( boost::filesystem::directory_iterator dir_iter( sPath ) ; dir_iter != end_iter ; ++dir_iter)
	{
		if( boost::iequals( sExtension, dir_iter->path().extension().string() ) )
		{
			vFiles.push_back(dir_iter->path().filename().string());
			std::cout << "\t" << i+1 << ":   " << dir_iter->path().filename().string() << std::endl;
			i++;
		}
    }
	if( i == 0)
	{
		std::cout << "There is no " << sExtension <<"-Files in the given directory."  << std::endl << std::endl;
		return false;
	}
	
	int iFile;
	std::cout << "Please select one of the given files by number." << std::endl;
	std::cin >> iFile;
	while( (iFile < 1) || (iFile > vFiles.size()) )
	{
		std::cout << "Your input is not valid, please select again." << std::endl;
		std::cin >> iFile;
	}

	sOutputPath = sPath + "\\" + vFiles[iFile-1];
	std::cout << std::endl;
	return true;
}


bool fileManagement::GetFileList( std::string sPath, std::string sExtension, std::vector<std::string> &vFileList )
{
	vFileList.clear();

	if( !boost::filesystem::exists(sPath) || !boost::filesystem::is_directory(sPath) )
	{
		std::cout << "The given path is not a valid directory." << std::endl << std::endl;
		return false;
	}
	
	boost::filesystem::directory_iterator end_iter;
	for( boost::filesystem::directory_iterator dir_iter( sPath ) ; dir_iter != end_iter ; ++dir_iter)
	{
		if( boost::iequals( sExtension, dir_iter->path().extension().string() ) )
		{
			vFileList.push_back(dir_iter->path().filename().string());
		}
    }
	std::sort(vFileList.begin(),vFileList.end());
	return true;
}


bool fileManagement::CreateDir( std::string sDir )
{
	return boost::filesystem::create_directories(sDir);
}


std::string fileManagement::getFilePathWithoutExt( std::string sPath )
{
	boost::filesystem::path pPath( sPath );

	return sPath.substr(0, sPath.length() - pPath.extension().string().length());
}


std::string fileManagement::getExtension( std::string sPath )
{
	boost::filesystem::path pPath( sPath );

	return pPath.extension().string();
}


bool fileManagement::checkExtension( std::string sPath, std::string sExtRef )
{
	std::string sExt = fileManagement::getExtension( sPath );

	if( sExtRef == sExt )
		return true;
	else
		return false;
}


bool fileManagement::FileCopy( std::string sSource, std::string sDestination, fileManagement::CopyOption option )
{
	boost::filesystem::path pSource( sSource );
	boost::filesystem::path pDestination( sDestination );

	std::string sDestParentPath;

	if(!fileManagement::Exists(sSource) || !pSource.has_filename()) {
		printf("Invalid source path.\n");
		return false;
	}

	if(pDestination.has_filename()) {
		sDestParentPath = pDestination.parent_path().string();

		if(!fileManagement::Exists(sDestParentPath))
			fileManagement::CreateDir(sDestParentPath);

		switch(option)
		{
		case fileManagement::CopyOption::KeepExistingFile:
			if(fileManagement::Exists(sDestination))
			{
				printf("File does already exist.\n");
				return false;
			}
			else
				boost::filesystem::copy_file(sSource,sDestination,boost::filesystem::copy_option::fail_if_exists);
			break;
		case fileManagement::CopyOption::OverwriteExistingFile:
			boost::filesystem::copy_file(sSource,sDestination,boost::filesystem::copy_option::overwrite_if_exists);
			break;
		default:
			printf("Invalid option parameter.\n");
			return false;
			break;
		}
		return true;
	}
	else
	{
		printf("Invalid destination path.\n");
		return false;
	}
}
