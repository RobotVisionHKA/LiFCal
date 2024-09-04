SET(PROJECT_NAME LiFCal)

CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(${PROJECT_NAME})

set(BUILD_TYPE Release)
#set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# required libraries
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem system thread)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(colmap REQUIRED)
find_package(CUDA  REQUIRED)

# flags
#set(CMAKE_CXX_FLAGS
#   "${SSE_FLAGS} -O3 -g -std=c++17 -march=native"
#)

# Sources files
set(lfCalibration_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CameraCalibration.cpp
  ${PROJECT_SOURCE_DIR}/src/main.cpp
)

set(CalibrationData_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/CalibrationData.cpp
)

set(CalibrationData_ArucoDetection_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/ArucoDetection/ArucoDetection.cpp
)

set(CalibrationData_Constraints_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/Constraints/Constraints.cpp
)

set(CalibrationData_ExtrinsicOrientation_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/ExtrinsicOrientation/ExtrinsicOrientation.cpp
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/ExtrinsicOrientation/ExtrinsicOrientations.cpp
)

set(CalibrationData_ImagePoints_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/ImagePoints/Images.cpp
)

set(CalibrationData_IntrinsicOrientation_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/IntrinsicOrientation/IntrinsicOrientation.cpp
)

set(CalibrationData_ObjectPoints_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/CalibrationData/ObjectPoints/ObjectPoints.cpp
)

set(ColmapReconstructor_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/ColmapReconstructor/ColmapReconstructor.cpp
)

set(MicroLensGrid_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/MicroLensGrid/EpiPolarLine.cpp
  ${PROJECT_SOURCE_DIR}/src/MicroLensGrid/MicroLens.cpp
  ${PROJECT_SOURCE_DIR}/src/MicroLensGrid/MicroLensGrid.cpp
)

set(Utility_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/Utility/FileManagement.cpp
)

set(Settings_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/Utility/Settings.cpp
)

set(Utility_myUtility_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/Utility/myUtility/myUtility.cpp
)

set(thirdparty_pugixml_SOURCE_FILES
  ${PROJECT_SOURCE_DIR}/src/thirdparty/pugixml-1.7/src/pugixml.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

add_library(LibLiFCal ${lfCalibration_SOURCE_FILES} ${CalibrationData_SOURCE_FILES} ${CalibrationData_ArucoDetection_SOURCE_FILES} ${CalibrationData_Constraints_SOURCE_FILES} ${CalibrationData_ExtrinsicOrientation_SOURCE_FILES} ${CalibrationData_ImagePoints_SOURCE_FILES} ${CalibrationData_IntrinsicOrientation_SOURCE_FILES} ${CalibrationData_ObjectPoints_SOURCE_FILES} ${ColmapReconstructor_SOURCE_FILES} ${MicroLensGrid_SOURCE_FILES} ${Parser_SOURCE_FILES} ${Utility_SOURCE_FILES} ${Settings_SOURCE_FILES} ${Utility_myUtility_SOURCE_FILES} ${thirdparty_pugixml_SOURCE_FILES})
add_executable(LiFCal ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(LiFCal PRIVATE LibLiFCal Eigen3::Eigen ${Boost_LIBRARIES} Ceres::ceres ${OpenCV_LIBS} colmap::colmap)

