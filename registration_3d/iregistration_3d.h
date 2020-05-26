#ifndef IREGISTRATION_H
#define IREGISTRATION_H

#include "stdafx.h"

class IRegistration3D
{
	public:
	virtual void SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, bool downsample) = 0;
  virtual void LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample) = 0;
};

extern "C" __declspec(dllexport) IRegistration3D* APIENTRY GetRegistration3D();

#endif
