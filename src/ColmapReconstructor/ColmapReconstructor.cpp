/**
 * @brief Definition of the class ColmapReconstructor.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ColmapReconstructor.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <cuda_runtime.h>

#include "ColmapReconstructor/ColmapReconstructor.h"

#include "Utility/myUtility/myUtility.h"


/**
 * @brief Constructor.
 *
 * Set the variables of the class and create output directory.
 */
ColmapReconstructor::ColmapReconstructor()
{
    this->dirWorkspace = "../../colmap_output";
    this->dirOutputFiles = this->dirWorkspace + "/sparse/0";
    this->dataType = "individual";
    this->quality = "low";
    this->mesher = "poisson";
    this->cameraModel = "OPENCV";
    this->dense = false;
    this->singleCamera = true;

    myUtility::createOrClearDirectory(this->dirWorkspace);
}

ColmapReconstructor::~ColmapReconstructor()
{
}

/**
 * @brief Use of colmap for an initial reconstruction from total focus images.
 */
bool ColmapReconstructor::reconstructionColmap(std::string dirImages, std::string &dirOutput)
{
    this->dirImages = dirImages;

    dirOutput = this->dirOutputFiles;

	colmap::AutomaticReconstructionController::Options reconstruction_options;
	
	reconstruction_options.workspace_path = this->dirWorkspace;
	reconstruction_options.image_path = this->dirImages;
	reconstruction_options.camera_model = this->cameraModel;
	reconstruction_options.single_camera = this->singleCamera;
	reconstruction_options.dense = this->dense;

	colmap::StringToLower(&this->dataType);
	if (this->dataType == "individual") {
	    reconstruction_options.data_type = colmap::AutomaticReconstructionController::DataType::INDIVIDUAL;
	} else if (this->dataType == "video") {
	    reconstruction_options.data_type = colmap::AutomaticReconstructionController::DataType::VIDEO;
	} else if (this->dataType == "internet") {
	    reconstruction_options.data_type = colmap::AutomaticReconstructionController::DataType::INTERNET;
	} else {
	    LOG(FATAL) << "Invalid data type provided";
	}

	colmap::StringToLower(&this->quality);
	if (this->quality == "low") {
	    reconstruction_options.quality = colmap::AutomaticReconstructionController::Quality::LOW;
	} else if (this->quality == "medium") {
	    reconstruction_options.quality = colmap::AutomaticReconstructionController::Quality::MEDIUM;
	} else if (this->quality == "high") {
	    reconstruction_options.quality = colmap::AutomaticReconstructionController::Quality::HIGH;
	} else if (this->quality == "extreme") {
	    reconstruction_options.quality = colmap::AutomaticReconstructionController::Quality::EXTREME;
	} else {
	    LOG(FATAL) << "Invalid quality provided";
	}

	colmap::StringToLower(&this->mesher);
	if (this->mesher == "poisson") {
	    reconstruction_options.mesher = colmap::AutomaticReconstructionController::Mesher::POISSON;
	} else if (this->mesher == "delaunay") {
	    reconstruction_options.mesher = colmap::AutomaticReconstructionController::Mesher::DELAUNAY;
	} else {
	    LOG(FATAL) << "Invalid mesher provided";
	}

	int cuda_device_count;
	cudaGetDeviceCount(&cuda_device_count);
	if(cuda_device_count != 0)
	{
		printf("GPU found for CUDA.\n");
		reconstruction_options.use_gpu = true;
		for (int device = 0; device < cuda_device_count; ++device)
		{
			cudaDeviceProp deviceProp;
			cudaGetDeviceProperties(&deviceProp, device);
			printf("Device %d has compute capability %d.%d.\n",
			device, deviceProp.major, deviceProp.minor);
		}
	}
	else
	{
		printf("No GPU found for CUDA.\n");
		reconstruction_options.use_gpu = false;
	}


	auto reconstruction_manager = std::make_shared<colmap::ReconstructionManager>();

	colmap::AutomaticReconstructionController controller(reconstruction_options, reconstruction_manager);

	controller.Start();
	controller.Wait();

    if(!saveTextFiles()) return false;
	
	return true;

}

/**
 * @brief Saving colmap's estimated data in human-readable *.txt format.
 */
bool ColmapReconstructor::saveTextFiles()
{
	std::string input_path = this->dirOutputFiles;
	std::string output_path = this->dirOutputFiles;
	bool skip_distortion = false;

	colmap::Reconstruction reconstruction;
	reconstruction.Read(input_path);
    reconstruction.WriteText(output_path);

	return true;
}
