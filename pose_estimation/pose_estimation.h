#pragma once
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"

#ifdef __DLLEXPORT
#define __DLL_INTERFACE _declspec(dllexport)    // 导出函数 - 生成dll文件时使用
#else
#define __DLL_INTERFACE _declspec(dllimport)    // 导入函数 -使用dll是使用
#endif

class  __DLL_INTERFACE PoseEstimation
{
	public:

	ICameraData* p_camera_data_;
	IRegistration3D *p_registration_;
	/*
	PointCloud::Ptr object_model;*/

	PoseEstimation();
	//~PoseEstimation();

	//void Init();
	//Eigen::Matrix4f Compute(PointCloud::Ptr source, PointCloud::Ptr target);
	std::string Compute();
};

__DLL_INTERFACE  PoseEstimation *GetPoseEstimation();
