#ifndef IREGISTRATION_H
#define IREGISTRATION_H

#include "stdafx.h"

class IRegistration3D
{
	public:
	virtual void Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform) = 0;
	virtual bool SetParameters(const std::string config_file) = 0;
};

__declspec(dllexport) IRegistration3D* APIENTRY GetRegistrationLMICP();
__declspec(dllexport) IRegistration3D* APIENTRY GetRegistrationSACIA();

#endif
