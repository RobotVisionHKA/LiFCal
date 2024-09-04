<h1 align="center">LiFCal: Online Light Field Camera Calibration via Bundle Adjustment</h1>

<p align="center">
    <span>Aymeric Fleith<sup>1,2</sup></span> &emsp;&emsp;
    <span>Doaa Ahmed<sup>2</sup></span> &emsp;&emsp;
    <span>Daniel Cremers<sup>1</sup></span> &emsp;&emsp;
    <span>Niclas Zeller<sup>2</sup></span>
</p>

<p align="center">
    <span><sup>1</sup>Technical University of Munich, Munich, Germany</span>
	<br>
    <span><sup>2</sup>Karlsruhe University of Applied Sciences, Karlsruhe, Germany</span>
</p>

<p align="center">
    Website: <a href="https://lifcal.github.io/">https://lifcal.github.io/</a>
</p>

## Installation

### Clone the repository
```bash
git clone https://github.com/RobotVisionHKA/LiFCal.git
```

### Install dependencies

The program requires the following dependencies:
- Nvidia Driver;
- Cuda;
- Ceres Solver;
- COLMAP;
- OpenCV.

Dependencies can be installed by following the instructions in the `installation` folder in this project.

## Build
```bash
mkdir build
cd build
cmake .. -GNinja -DCMAKE_CUDA_ARCHITECTURES=native -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
ninja
```

The build commands create an executable file "LiFCal" in the folder `build/bin`.

## Usage

This section explains how to use the program, describing the structure of the different inputs and outputs.

### Call of the compiled file

The executable file "LiFCal" is created in the `build/bin` folder. It takes several arguments:
```bash
cd bin
./LiFCal mode settings data
```


`mode`: The first argument selects the calibration mode between full calibration or recalibration.

`settings`: The second argument is a *.yaml file containing the settings required for calibration.

`data`: The third argument is a *.txt file containing the constraints relating to the chosen mode.


Depending on the mode chosen, the arguments are as follows:
- complete calibration:
```bash
./LiFCal calib_marker path_to_settings path_to_constraints
```
- recalibration:
```bash
./LiFCal recalib path_to_settings path_to_fixed_parameters
```

### Format of the input files

The files used vary according to the mode used. The files described below correspond to the arguments to be given to the executable file.

#### Settings file

The settings file is common to all modes. It is a *.yaml file which defines the settings and input data for the calibration. The parameters are as follows:
- `Camera.pixelSize`\
	Data type: float\
	Required: Yes\
	Description: Size of a pixel on the camera sensor in mm.

- `Camera.rawImageWidth`\
	Data type: int\
	Required: Yes\
	Description: Raw image width in pixels (usually twice the width of the totally focused image).

- `Camera.rawImageHeight`\
	Data type: int\
	Required: Yes\
	Description: Raw image height in pixels (usually twice the height of the totally focused image).

- `Path.totalFocusImages`\
	Data type: path\
	Required: Yes\
	Description: Path to folder containing the totally focused images for the calibration.

- `Path.microLensCalibration`\
	Data type: path\
	Required: Yes\
	Description: Path to the micro lens calibration file.

- `Config.numberPoints`\
	Data type: int (greater than 0)\
	Required: No\
	Default value: 500\
	Description: Number of 3D points used for the light field bundle adjustment.

- `Config.markerDictionary`\
	Data type: int (between 0 and 20)\
	Required: No\
	Default value: 10\
	Description: Number of the used OpenCV marker dictionary (see section "Constraint file" below for more details).

- `Model.numberRadialDistParam`\
	Data type: int (between 0 and 2)\
	Required: No\
	Default value: 2\
	Description: Number of radial distortion parameters used in the calibration.

- `Model.tangentialDist`\
	Data type: bool (`true` or `false`)\
	Required: No\
	Default value: `true`\
	Description: Use of tangential distortion.

- `Model.adjustMicroLensCenters`\
	Data type: bool (`true` or `false`)\
	Required: No\
	Default value: `true`\
	Description: Adjustment of micro lens centers during the optimization.

- `Model.refineExtrinsicOrientations`\
	Data type: bool (`true` or `false`)\
	Required: No\
	Default value: `true`\
	Description: Refinement of extrinsic orientation parameters of the frames.

- `Model.refineExtrinsicOrientations`\
	Data type: bool (`true` or `false`)\
	Required: No\
	Default value: `true`\
	Description: Refine the coordinates of the 3D calibration points.

- `Model.robustCostFunction`\
	Data type: bool (`true` or `false`)\
	Required: No\
	Default value: `true`\
	Description: Use of a robust cost function.

#### Constraint file

The constraints file is a *.txt file. It is only used in the "calib\_aruco" mode, which allows full calibration of all plenoptic camera parameters.

The constraint file is used to specify one or more known distances in the scene to provide scale information for calibration. Aruco markers are used. OpenCV dictionary 10 (DICT\_6X6\_250) is used as defalut but other dictionaries can be set with the parameter "Config.markerDictionary" in the settings file (predefined dictionaries are available at this <a href="https://docs.opencv.org/4.x/de/d67/group__objdetect__aruco.html#ga4e13135a118f497c6172311d601ce00d" target="_blank">link</a>).

At least two markers from this dictionary must be visible in the calibration scene. Their ID in the dictionary must be known. This data is entered in the *.txt constraints file:

```txt
# List of the constraints
# Format: pointID_1 pointID_2 distance sigma
pointID_1 pointID_2 distance sigma
```

Lines preceded by the hash symbol (#) are comments and are ignored by the program. The other lines are constructed with four parameters:

- `pointID_1`: The ID of the first Aruco marker visible in the scene.

- `pointID_2`: The ID of the second Aruco marker visible in the scene.

- `distance`: The real distance between the center of the first Aruco marker and the center of the second Aruco marker in mm.

- `sigma`: Value used to set a weight for the various distances (if the weight is identical for all distances, the value can be set to 0.1 for each line, for example).

Each line defines a distance between two Aruco markers. The number of lines is not limited. An Aruco marker can be involved in several distances.

#### Fixed parameters file

The fixed parameter file is a *.txt file. It is only used in the "recalib" mode, which allows to recalibrate certain parameters by fixing those that cannot change over time.

The file is used to set the focal length of the main lens (f) and the distance between the micro lens array and the sensor (B):

```txt
# Fix parameters
# Format: f B
f B
```

Lines preceded by the hash symbol (#) are comments and are ignored by the program. The line read and used must contain two parameters which have been previously obtained by a first complete calibration:

- `f`: The focal length of the main lens in mm.

- `B`: The distance between the micro lens array and the sensor in mm.


## Output data

At the end of the calibration, the program displays the camera parameters and enables to save the calibration. The user then has the option of specifying a path where to save the calibration results. This creates a folder with the name "Calibration_Results\_YYYY\_MM\_DD\_hhmmss" where "YYYY" is the year, "MM" the month, "DD" the day, "hh" the hour, "mm" the minutes, "ss" the seconds.

The light-field camera calibration is saved as a folder containing several subfolders and files:

- `projectedCameraCoordinates`: Point cloud in camera coordinates for the points visible in each view.

- `refCameraCoordinates`: Back projected camera coordinates for the points visible in each view.

- `calibrationProtocol.txt`: Calibration result with all parameters set for the calibration protocol.

- `CameraModel.xml`: The complete camera model with the five parameters estimated during the bundle adjustment.

- `cameraOrientations.ply`: Pyramid display of camera positions for all frames (for viewing purposes only).

- `extrinsicOrientations.xml`: Camera poses for each frame in the form of rotation by Euler angles and translation vector with respect to the world coordinate frame.

- `ExtrinsicOrientations.txt`: Camera poses for each frame as a homogeneous matrix in SE(3).

- `objectCoordinates.ply`: Coordinates of 3D points in the scene at the end of the optimization.

- `rawImagePoints.csv`: The data for the points in the raw image. Each line contains the following information: image ID, point ID in the frame, raw x-coordinate of the point, raw y-coordinate of the point, projected x-coordinate, projected y-coordinate, 3D point ID.




## Example

Example data with associated explanations are available at this link: <a href="https://bwsyncandshare.kit.edu/s/Hmn9rNYs7DLkA8A" target="_blank">https://bwsyncandshare.kit.edu/s/Hmn9rNYs7DLkA8A</a>.

## Recommendations for correct use and good results

Parameters can be enabled or disabled in the settings file. They are enabled by default. We recommend to leave all parameters enabled for best results. They can, however, be disabled for test purposes.

LiFCal can be used on a wide variety of scenes. However, a few precautions must be taken to ensure good clalibration results in the output:
- The chosen scene must contain a sufficient number of features. As calibration relies on SIFT features, a completely homogeneous scene will not work well.
- When taking images, the camera must perform sufficient movement in all degrees of freedom (translation and rotation). This will ensure more accurate results.
- When taking images, camera movement should not be too quick to avoid motion blur, which would affect calibration accuracy.

## License Terms

LiFCal was developed in collaboration between the Technical University of Munich and the Karlsruhe University of Applied Sciences. The code is open-source under a GNU General Public License Version 3 (GPLv3).

## BibTeX

```
@inproceedings{fleith2024LiFCal,
	title     = {LiFCal: Online Light Field Camera Calibration via Bundle Adjustment},
	author    = {Fleith, Aymeric and Ahmed, Doaa and Cremers, Daniel and Zeller, Niclas},
	booktitle = {German Conference on Pattern Recognition (GCPR)},
	year      = {2024},
}
```

