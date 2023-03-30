#ifndef IREGISTRATION3D_H
#define IREGISTRATION3D_H

#include "stdafx.h"


#ifdef REGISTRATION3D_EXPORTS
#define REGISTRATION3D__API __declspec(dllexport)
#else
#define REGISTRATION3D__API __declspec(dllimport)
#endif

class IRegistration3D
{
	public:
	virtual void Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform) = 0;
	virtual bool SetParameters(const std::string config_file) = 0;
};

REGISTRATION3D__API IRegistration3D* APIENTRY GetRegistrationLMICP();
REGISTRATION3D__API IRegistration3D* APIENTRY GetRegistrationSACIA();

#endif
