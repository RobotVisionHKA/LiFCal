/**
 * @brief Declaration of the namespace CameraModel.
 *
 * @authors Aymeric Fleith, Doaa Ahmed, Daniel Cremers, Niclas Zeller
 * @date 2024
 * @file CameraModel.h
 *
 * @copyright 2024 Technical University of Munich / Karlsruhe University of Applied Sciences
 */

#pragma once

#include <Eigen/Dense>


namespace CameraModel
{
	/// Apply radial distortion
	template <typename T> inline void radialDistortion(T x, T y, T &delta_x, T &delta_y, const T* param, int nParam);
	/// Apply tangential distortion
	template <typename T> inline void tangentialDistortion(T x, T y, T &delta_x, T &delta_y, const T* param);

	/**
	 * @brief Project point back from virtual image to 3D camera coordinates.
	 */
	template <typename T>
	inline void projectPointBack(Eigen::Matrix<T,3,1> &p3d_c_out, T x_v, T y_v, T v_depth,
		T spx, T spy, T fL, T bL0, T B, T* c, const T* radialDist, int nRadialDist, const T* tangeltialDist)
	{
		T c_x = c[0];
		T c_y = c[1];

		T projected_x = (x_v-c_x)*spx;
		T projected_y = (y_v-c_y)*spy;
		T projected_z = v_depth*B;

		// Project on mla plane
		projected_x = ( projected_x / ( bL0 + projected_z ) ) * bL0;
		projected_y = ( projected_y / ( bL0 + projected_z ) ) * bL0;

		// Apply distortion in mla plane
		{
			T projected_x_dist = projected_x;
			T projected_y_dist = projected_y;
			T projected_z_dist = projected_z;

			if(nRadialDist>0 ||  tangeltialDist!=NULL)
			{
				// Apply distortion
				T delta_rad_x = T(0.0);
				T delta_rad_y = T(0.0);
				T delta_tan_x = T(0.0);
				T delta_tan_y = T(0.0);

				for(int i=0; i<10; i++)
				{
					// Calculate radial distortion
					if(nRadialDist>0)
					{
						radialDistortion<T>(projected_x,projected_y,delta_rad_x,delta_rad_y,radialDist,nRadialDist);	
					}
					// Calculate tangential distortion
					if(tangeltialDist != NULL)
					{
						tangentialDistortion<T>(projected_x,projected_y,delta_tan_x,delta_tan_y,tangeltialDist);
					}

					projected_x = projected_x_dist - delta_rad_x - delta_tan_x;
					projected_y = projected_y_dist - delta_rad_y - delta_tan_y;
					projected_z = projected_z_dist;
				}
			}
		}

		// Project point to 3D camera coordinates
		projected_z = projected_z + bL0;
		p3d_c_out[2] = fL*projected_z / (projected_z - fL);

		p3d_c_out[0] = projected_x/bL0*p3d_c_out[2];
		p3d_c_out[1] = projected_y/bL0*p3d_c_out[2];
	}

	/**
	 * @brief Project point from 3D camera coordinates to a certain micro lens on the sensor.
	 */
	template <typename T>
	inline void projectPoint(T &projected_x_out, T &projected_y_out,  Eigen::Matrix<T,3,1> camera_coordinate_point,
		T spx, T spy, T fL, T bL0, T B, T* c_raw, T* mlCenter, const T* radialDist, int nRadialDist, const T* tangeltialDist,
		bool mlCenterAdjustment, bool print = false)
	{	
		// Undistort ml center
		T c_dist[2];
		c_dist[0] = (T(mlCenter[0])-c_raw[0])*T(spx);
		c_dist[1] = (T(mlCenter[1])-c_raw[1])*T(spy);
 
		T c_undist[2];
		c_undist[0] = c_dist[0];
		c_undist[1] = c_dist[1];

		// Calculate radial distortion
		if(nRadialDist>0 ||  tangeltialDist)
		{
			// Apply distortion
			T delta_rad_x = T(0.0);
			T delta_rad_y = T(0.0);
			T delta_tan_x = T(0.0);
			T delta_tan_y = T(0.0);
			
			for(int i=0; i<10; i++)
			{
				// Calculate radial distortion
				if(nRadialDist>0)
				{
					CameraModel::radialDistortion<T>(c_undist[0],c_undist[1],delta_rad_x,delta_rad_y,radialDist,nRadialDist);	
				}
				// Calculate tangential distortion
				if(tangeltialDist)
				{
					CameraModel::tangentialDistortion<T>(c_undist[0],c_undist[1],delta_tan_x,delta_tan_y,tangeltialDist);
				}
				
				c_undist[0] = c_dist[0] - delta_rad_x - delta_tan_x;
				c_undist[1] = c_dist[1] - delta_rad_y - delta_tan_y;
			}
		}

		if(mlCenterAdjustment)
		{
			c_undist[0] = c_undist[0]/(bL0+B)*bL0;
			c_undist[1] = c_undist[1]/(bL0+B)*bL0;
		}

		T zC0 = fL*bL0/(fL-bL0);

		T pML[2];
		pML[0] = -c_undist[0]*fL/(fL-bL0);
		pML[1] = -c_undist[1]*fL/(fL-bL0);

		Eigen::Matrix<T, 3, 1> p3d_p;
		p3d_p[0] = camera_coordinate_point[0] - pML[0];
		p3d_p[1] = camera_coordinate_point[1] - pML[1];
		p3d_p[2] = camera_coordinate_point[2] + zC0;

		p3d_p /= p3d_p[2];

		T pMl[2];
		pMl[0] = (p3d_p[0] - c_undist[0]/fL)*fL*B/(fL-bL0);
		pMl[1] = (p3d_p[1] - c_undist[1]/fL)*fL*B/(fL-bL0);

		T projected_x, projected_y;

		if(mlCenterAdjustment)
		{
			projected_x = pMl[0] + c_undist[0];
			projected_y = pMl[1] + c_undist[1];

			if(nRadialDist>0 ||  tangeltialDist)
			{
				// Apply distortion
				T delta_rad_x = T(0.0);
				T delta_rad_y = T(0.0);
				T delta_tan_x = T(0.0);
				T delta_tan_y = T(0.0);
		
				// Calculate radial distortion
				if(nRadialDist>0)
					CameraModel::radialDistortion<T>(projected_x,projected_y,delta_rad_x,delta_rad_y,radialDist,nRadialDist);	
				// Calculate tangential distortion
				if(tangeltialDist)
					CameraModel::tangentialDistortion<T>(projected_x,projected_y,delta_tan_x,delta_tan_y,tangeltialDist);

				projected_x += delta_rad_x + delta_tan_x;
				projected_y += delta_rad_y + delta_tan_y;

			}
		}
		else
		{
			T scale_x, scale_y;
			scale_x = T(1.0);
			scale_y = T(1.0);
			if(mlCenterAdjustment)
			{
				projected_x = (pMl[0]*scale_x + c_undist[0] - c_undist[0]*(bL0+B)/bL0 + c_dist[0]);
				projected_y = (pMl[1]*scale_y + c_undist[1] - c_undist[1]*(bL0+B)/bL0 + c_dist[1]);
			}
			else
			{
				projected_x = pMl[0]*scale_x + c_dist[0];
				projected_y = pMl[1]*scale_y + c_dist[1];
			}
		}

		projected_x = projected_x/spx + c_raw[0];
		projected_y = projected_y/spy + c_raw[1];

		projected_x_out = projected_x;
		projected_y_out = projected_y;
	}


	/**
	 * @brief Apply radial distortion.
	 */
	template <typename T> inline void radialDistortion(T x, T y, T &delta_x, T &delta_y, const T* param, int nParam)
	{
		T r[5];		
		if(nParam>5)
		{
			for(int i=5;i<nParam;i++) r[i]=T(0.0);
			nParam=5;
		}
		
		r[0] =x*x+y*y;
		T delta_r = param[0]*r[0];
		for(int i=1;i<nParam;i++)
		{
			r[i] = r[i-1]*r[0];
			delta_r += param[i]*r[i];
		}
		delta_x = x*delta_r;
		delta_y = y*delta_r;
	}

	/**
	 * @brief Apply tangential distortion.
	 */
	template <typename T> inline void tangentialDistortion(T x, T y, T &delta_x, T &delta_y, const T* param)
	{
		if(param == NULL)
		{
			delta_x = T(0.0);
			delta_y = T(0.0);
		}
		else
		{
			T r_2 = x*x+y*y;
			delta_x = param[0]*(r_2+T(2.0)*x*x)+T(2.0)*param[1]*x*y;
			delta_y = param[1]*(r_2+T(2.0)*y*y)+T(2.0)*param[0]*x*y;
		}
	}
};

namespace RigidBody
{
	template <typename T>
	inline Eigen::Matrix<T,4,4> getTransformationMatrix(Eigen::Matrix<T,3,1> rotAngle, Eigen::Matrix<T,3,1> transVec)
	{
		Eigen::Matrix<T, 4, 4> RT = Eigen::Matrix<T, 4, 4>::Identity();

		Eigen::Matrix<T,3,3> R = (
				Eigen::AngleAxis<T>(rotAngle[0], Eigen::Matrix<T,3,1>::UnitX()) *
				Eigen::AngleAxis<T>(rotAngle[1], Eigen::Matrix<T,3,1>::UnitY()) *
				Eigen::AngleAxis<T>(rotAngle[2], Eigen::Matrix<T,3,1>::UnitZ())).toRotationMatrix();

		for(int i=0;i<3;i++)
			for(int j=0;j<3;j++)
				RT(i,j) = R(i,j);
		for(int i=0;i<3; i++)
			RT(i,3) = transVec[i];

		return RT;

	}
};
