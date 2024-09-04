/**
 * @brief Definition of the cost function.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file BundleAdjustment.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <vector>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>
#include <Eigen/Dense>

#include "CameraModel.h"
#include "CameraCalibration.h"


class frame;

struct OurCostFunctionBundle {
	OurCostFunctionBundle(int config, double observed_x, double observed_y, double spx, double spy, double scale, Eigen::Vector2d mlCenter, Eigen::Vector3d objectPoint = Eigen::Vector3d(0,0,0), double* view = NULL)
	{
		// Decode config variable
		this->config = config;

		// Minimum number of camera parameters is 5 (fL,B,bL0,cx,cy)
		this->nCameraParameters = 5;

		// Radial distortion
		this->nRadialDistParam = (config & 0x000003);
		if(this->nRadialDistParam>0)
		{
			this->idxRadialDistParam = this->nCameraParameters;
			this->nCameraParameters += nRadialDistParam;
		}
		else
			this->idxRadialDistParam = -1;		

		// Tangential distortion
		if((config & (0x000004)) != 0)
		{
			this->tangentialDistParam = true;
			this->idxTangetialDistParam = this->nCameraParameters;
			this->nCameraParameters += 2;
		}
		else
		{
			this->tangentialDistParam = false;
			this->idxTangetialDistParam = -1;
		}

		// Refine frame poses
		if(((config & 0x000100) != 0) && (view == NULL))
			this->refinePoses = true;
		else
			this->refinePoses = false;

		// Use robust const function
		if((config & 0x000200) != 0)
			this->useRobustCostFunction = true;
		else
			this->useRobustCostFunction = false;

		// Refine 3D points
		if((config & 0x000400) != 0)
			this->refine3Dpoints = true;
		else
			this->refine3Dpoints = false;
		
		// Micro-lens center adjustment
		if((config & 0x000800) != 0)
			this->mlCenterAdjustment = true;
		else
			this->mlCenterAdjustment = false;

		// Observed image coordinates
		this->observed_x = observed_x;
		this->observed_y = observed_y;
	
		// Calculate raw image pixel size
		this->spx = spx/scale;
		this->spy = spy/scale;
		this->scale = scale;

		this->mlCenter = mlCenter;

		this->objectPoint = objectPoint;

		if(view != NULL)
		{
			Eigen::Matrix<double, 3, 1> rotAngles(view[0],view[1],view[2]);
			Eigen::Matrix<double, 3, 1> transVec(view[3],view[4],view[5]);
			
			this->RT = RigidBody::getTransformationMatrix<double>(rotAngles, transVec);
			this->hCamCoord = RT * this->objectPoint.homogeneous();
		}
	}

	template <typename T> bool operator()(const T* const camera, const T* const view, T* residuals) const
	{
		return operator_function<T>(camera,view,NULL,residuals);
	}

	template <typename T> bool operator()(const T* const camera, const T* const view, const T* const objectPoint, T* residuals) const
	{
		return operator_function<T>(camera,view,objectPoint,residuals);
	}


	template <typename T> bool operator()(const T* const camera, T* residuals) const
	{
		return operator_function<T>(camera,NULL,NULL,residuals);
	}

	template <typename T>
	bool operator_function(const T* const camera, const T* const view, const T* const objPt, T* residuals) const
	{
		T fL = camera[0];
		if(fL < T(0.0)) fL = -fL;
		T bL0 = camera[1];
		if(bL0 < T(0.0)) bL0 = -bL0;
		T B = camera[2];
		if(B < T(0.0)) B = -B;
		T c_raw[2];
		c_raw[0] = (camera[3]+T(0.5))*T(this->scale)-T(0.5);
		c_raw[1] = (camera[4]+T(0.5))*T(this->scale)-T(0.5);
		if(c_raw[0] < T(0.0)) c_raw[0] = -c_raw[0];
		if(c_raw[1] < T(0.0)) c_raw[1] = -c_raw[1];

		const T* radialDist;

		if(nRadialDistParam>0)
			radialDist = camera + this->idxRadialDistParam;
		else
			radialDist = NULL;

		const T* tangeltialDist;
		if(tangentialDistParam)
			tangeltialDist = camera + this->idxTangetialDistParam;
		else
			tangeltialDist = NULL;

		Eigen::Matrix<T, 4, 1> p3d_w;
		if(this->refine3Dpoints)
		{
			for(int i=0;i<3;i++)
				p3d_w[i] = objPt[i];
		}
		else
		{
			for(int i=0;i<3;i++)
				p3d_w[i] = T(this->objectPoint[i]);
		}
		p3d_w[3] = T(1.0);

		// Calculate transformation from world to camrea cordinates
		Eigen::Matrix<T, 3, 1> p3d_c;
		if(this->refinePoses)
		{
			Eigen::Matrix<T, 3, 1> rotAngles(view[0],view[1],view[2]);
			Eigen::Matrix<T, 3, 1> transVec(view[3],view[4],view[5]);
			Eigen::Matrix<T, 4, 4> RT = RigidBody::getTransformationMatrix<T>(rotAngles, transVec);

			Eigen::Matrix<T, 4, 1> hom_p3d_c = RT*p3d_w;
		
			for(int i=0;i<3;i++)
				p3d_c[i] = hom_p3d_c[i];
		}
		else
		{
			for(int i=0;i<3;i++)
				p3d_c[i] = T(this->hCamCoord[i]);
		}
		
		// *** apply camera model ***
		T projected_x, projected_y;

		T mlCenter[2];
		mlCenter[0] = T(this->mlCenter[0]);
		mlCenter[1] = T(this->mlCenter[1]);

		CameraModel::projectPoint<T>(projected_x,projected_y,p3d_c,T(this->spx),T(this->spy),fL,bL0,B,c_raw,mlCenter,radialDist,nRadialDistParam,tangeltialDist,mlCenterAdjustment);

		// Calculate residuals
		// The error is the difference between the projected and observed position.
		residuals[0] = (projected_x - T(observed_x));
		residuals[1] = (projected_y - T(observed_y));

		return true;
	}

	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction* Create(const int config, const double observed_x, const double observed_y, const double spx, const double spy, const double scale, Eigen::Vector2d mlCenter, Eigen::Vector3d* objectPoint = NULL, double* view = NULL)
	{
		if(view == NULL)
		{
			if(objectPoint == NULL)
			{
				// Extrinsic orientation is not given as input parameter since it is estimated during the optimization
				return (new ceres::AutoDiffCostFunction<OurCostFunctionBundle, 2, MAX_NUMBER_OF_CAMERA_PARAMETERS, 6, 3>(
					new OurCostFunctionBundle(config,observed_x,observed_y,spx,spy,scale,mlCenter)));
			}
			else
			{
				// Extrinsic orientation is not given as input parameter since it is estimated during the optimization
				return (new ceres::AutoDiffCostFunction<OurCostFunctionBundle, 2, MAX_NUMBER_OF_CAMERA_PARAMETERS, 6>(
					new OurCostFunctionBundle(config,observed_x,observed_y,spx,spy,scale,mlCenter,*objectPoint)));
			}
		}
		else
		{
			// Extrinsic orientant and 3d object coordinates are not optimized
			return (new ceres::AutoDiffCostFunction<OurCostFunctionBundle, 2, MAX_NUMBER_OF_CAMERA_PARAMETERS>(
				new OurCostFunctionBundle(config,observed_x,observed_y,spx,spy,scale,mlCenter,*objectPoint,view)));
		}
	}

	int config;

	int nRadialDistParam;
	int idxRadialDistParam;

	bool tangentialDistParam;
	int idxTangetialDistParam;

	bool mlCenterAdjustment;

	bool refinePoses;
	bool refine3Dpoints;
	bool useRobustCostFunction;

	int nCameraParameters;

	double observed_x;
	double observed_y;

	// Pixel size
	double spx,spy;
	// Scale from virtual image to raw image
	double scale;

	Eigen::Vector2d mlCenter;
	Eigen::Vector3d objectPoint;
	Eigen::Matrix4d RT;
	Eigen::Vector4d hCamCoord;
};


struct OurConstraintFunctionBundle {
	OurConstraintFunctionBundle(double distance, double sigma)
	{
		this->distance = distance;
		this->sigma = sigma;
	}

	template <typename T> bool operator()(const T* const point1, const T* const point2, T* residuals) const
	{
		residuals[0] = (ceres::pow((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1])+(point1[2]-point2[2])*(point1[2]-point2[2]),T(0.5))-T(distance))/(T(sigma)+T(0.000001));
		
		return true;
	}	

	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction* Create(const double distance, const double sigma)
	{
		return (new ceres::AutoDiffCostFunction<OurConstraintFunctionBundle, 1, 3, 3>(
			new OurConstraintFunctionBundle(distance,sigma)));
	}

	double distance;
	double sigma;
};

struct OurOrientationConstraintFunctionBundle {
	OurOrientationConstraintFunctionBundle(Eigen::Vector3d position)
	{
		this->position = position;
		norm = position.norm();
	}

	template <typename T> bool operator()(const T* const point, T* residuals) const
	{
		T norm_2 =  (point[0]*point[0]+point[1]*point[1]+point[2]*point[2]);
		T norm = T(ceres::pow(norm_2,T(0.5)));
		if(norm<T(0.00001))norm=T(0.00001);

		if(this->norm <= 0.01)
		{
			residuals[0] = (point[0]-T(position[0]))*T(0.001);
			residuals[1] = (point[1]-T(position[1]))*T(0.001);
			residuals[2] = (point[2]-T(position[2]))*T(0.001);
		}
		else
		{
			residuals[0] = (point[0]/norm-T(position[0]))*T(0.001);
			residuals[1] = (point[1]/norm-T(position[1]))*T(0.001);
			residuals[2] = (point[2]/norm-T(position[2]))*T(0.001);
		}
		return true;
	}	

	// Factory to hide the construction of the CostFunction object from
	// the client code.
	static ceres::CostFunction* Create(const Eigen::Vector3d position)
	{
		return (new ceres::AutoDiffCostFunction<OurOrientationConstraintFunctionBundle, 3, 3>(
			new OurOrientationConstraintFunctionBundle(position)));
	}

	Eigen::Vector3d position;
	double norm;
};
