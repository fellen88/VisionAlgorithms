#ifndef IREGISTRATION_H
#define IREGISTRATION_H

#include "stdafx.h"

class IRegistration3D
{
	public:
	virtual void SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, float downsample, bool debug_v) = 0;
  virtual void LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample = 0.1, bool debug_v = true) = 0;
	virtual void DCP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample , bool debug_v) = 0;
	virtual void ComputeTransformation(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, float downsample, bool debug_v) = 0;
	virtual Eigen::Matrix4f GetTransformation() = 0;
};

__declspec(dllexport) IRegistration3D* APIENTRY GetRegistration3D(const std::string config_file);

#endif
