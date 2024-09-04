/**
 * @brief Definition of the class ArucoDetection.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file ArucoDetection.cpp
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#include <opencv2/aruco.hpp>

#include "CalibrationData/ArucoDetection/ArucoDetection.h"
#include "CalibrationData/ImagePoints/Image.h"


ArucoDetection::ArucoDetection(int indexDictionary, std::string folderPathTotalFocus)
{
    this->indexDictionary = indexDictionary;
    this->folderPathTotalFocus = folderPathTotalFocus;
}

ArucoDetection::~ArucoDetection(void)
{
}

/**
 * \brief Detect the aruco markers in the total focus images.
 */
void ArucoDetection::detectArucoMarkers(std::vector<int> imagesIDOrder)
{
    cv::aruco::DetectorParameters detectorParams;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PredefinedDictionaryType(indexDictionary));
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    // Get the images of the folder of the total focused images
    cv::String path(folderPathTotalFocus + "/*.png");
    std::vector<cv::String> fn;
    std::vector<cv::Mat> data;
    cv::glob(path,fn,false);

    // Go through all images in the same order as the one defined by colmap
    for(std::vector<int>::iterator iter = imagesIDOrder.begin(); iter < imagesIDOrder.end(); iter++)
    {
        int imageID = *iter;
        cv::Mat image = cv::imread(fn[imageID - 1]);
        if (image.empty()) continue;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        // Detect markers
        detector.detectMarkers(image, corners, ids);

        size_t  nMarkers = corners.size();

        // If no markers are detected in the image
        if (nMarkers <= 0)
        {
            Image* currentImage;
            currentImage = new Image(imageID);
            this->imagesByIDAruco[imageID] = currentImage;
            this->imagesAruco.push_back(currentImage);
        }

        // Add each marker to the vector
        for (size_t  markerId = 0; markerId < nMarkers; markerId++)
        {
            Eigen::Vector2d markerCenter = getCenterMarker(corners.at(markerId));
            int pointID = ids.at(markerId);

            // Add id of the marker to the list if it is not allready in it
            if(std::find(this->markerIds.begin(), this->markerIds.end(), pointID) == this->markerIds.end()) {this->markerIds.push_back(pointID);}

            Image* currentImage;
            if(this->imagesByIDAruco.count(imageID) == 0)
            {
                currentImage = new Image(imageID);
                this->imagesByIDAruco[imageID] = currentImage;
                this->imagesAruco.push_back(currentImage);
            }
            else currentImage = this->imagesByIDAruco[imageID];

            if(currentImage->imageCoordinatesByPointID.count(pointID) == 0)
            {
                ImageCoordinate* imPt = new ImageCoordinate(pointID, imageID, markerCenter, 0);
                currentImage->imageCoordinatesByPointID[pointID] = imPt;
                currentImage->imageCoordinates.push_back(imPt);

                currentImage->imageCoordinatesInliers.push_back(imPt);
            }
            else
            {
                // This point can be readed if there occure outliers for the current point ID
                // if the existing point is an outlier and the new one not we replace the existing point by the new one.

                if(currentImage->imageCoordinatesByPointID[pointID]->isOutlier())
                {
                    // Replace old point
                    ImageCoordinate imPt(pointID, imageID, markerCenter, 0);
                    memcpy(currentImage->imageCoordinatesByPointID[pointID], &imPt, sizeof(ImageCoordinate));
                    currentImage->imageCoordinatesInliers.push_back(currentImage->imageCoordinatesByPointID[pointID]);
                }
                else
                {
                    printf("There are two coordinates which have the same ID and are both no outliers (Image ID: %d, Point ID: %d)\n One point is neglected.\n", imageID, pointID);
                }
            }
        }
    }
    GetNumberOfInliersForEachImage();
}

/**
 * \brief Get the number of aruco markers detected in each image.
 */
void ArucoDetection::GetNumberOfInliersForEachImage(void)
{
	for(int i = 0; i < this->imagesAruco.size(); i++)
	{
		this->NumberInliersAruco.push_back(this->imagesAruco[i]->imageCoordinatesInliers.size());
	}
}

/**
 * \brief Get the coordinates of the center of the aruco markers from the coordinates of the angles.
 */
Eigen::Vector2d ArucoDetection::getCenterMarker(std::vector<cv::Point2f> cornersMarker)
{
	Eigen::Vector2d p;
	
	float x1 = cornersMarker.at(0).x;
	float y1 = cornersMarker.at(0).y;
	
	float x2 = cornersMarker.at(1).x;
	float y2 = cornersMarker.at(1).y;
	
	float x3 = cornersMarker.at(2).x;
	float y3 = cornersMarker.at(2).y;
	
	float x4 = cornersMarker.at(3).x;
	float y4 = cornersMarker.at(3).y;
	
	Eigen::Matrix2f matrixDiag1; matrixDiag1 << x1, y1, x3, y3;
	Eigen::Matrix2f matrixDiag2; matrixDiag2 << x2, y2, x4, y4;
	Eigen::Matrix2f matrixDiag1X; matrixDiag1X << x1, 1, x3, 1;
	Eigen::Matrix2f matrixDiag1Y; matrixDiag1Y << y1, 1, y3, 1;
	Eigen::Matrix2f matrixDiag2X; matrixDiag2X << x2, 1, x4, 1;
	Eigen::Matrix2f matrixDiag2Y; matrixDiag2Y << y2, 1, y4, 1;
	
	Eigen::Matrix2f matrixNumX; matrixNumX << matrixDiag1.determinant(), matrixDiag1X.determinant(), matrixDiag2.determinant(), matrixDiag2X.determinant();
	Eigen::Matrix2f matrixNumY; matrixNumY << matrixDiag1.determinant(), matrixDiag1Y.determinant(), matrixDiag2.determinant(), matrixDiag2Y.determinant();
	Eigen::Matrix2f matrixDen; matrixDen << matrixDiag1X.determinant(), matrixDiag1Y.determinant(), matrixDiag2X.determinant(), matrixDiag2Y.determinant();

	p << matrixNumX.determinant() / matrixDen.determinant(), matrixNumY.determinant() / matrixDen.determinant();
	
	return p;
}
