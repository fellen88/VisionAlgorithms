#pragma once
#include "ipose_estimation.h"
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"

#ifdef __DLLEXPORT
#define __DLL_INTERFACE _declspec(dllexport)    // 导出函数 - 生成dll文件时使用
#else
#define __DLL_INTERFACE _declspec(dllimport)    // 导入函数 -使用dll是使用
#endif

class PoseEstimation: public IPoseEstimation
{
	public:

	ICameraData* p_realsense_;
	IRegistration3D *p_registration_;

	bool pose_flag;
	bool debug_visualization;
	float sample_3d;
	std::string pose_object;
	
	cv::Mat object_depth;
	cv::Mat object_color;
	cv::Mat object_mask;
	std::string object_label;
	PointCloud::Ptr object_model;
	PointCloud::Ptr object_scan;
	Eigen::Matrix4f object_transform;

	PoseEstimation();
	~PoseEstimation();

	bool GetTransformation();
};
