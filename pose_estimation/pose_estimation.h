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

	std::shared_ptr<ICameraData> p_realsense_;
	std::shared_ptr<IRegistration3D> p_registration_;

	bool pose_state;
	bool debug_visualization;
	float sample_3d;
	std::string pose_object;
	
	cv::Mat object_depth;
	cv::Mat object_color;
	cv::Mat object_mask;
	cv::Mat object_mask_collision;
	std::string object_label;
	PointCloud::Ptr object_model;
	PointCloud::Ptr object_scan;
	PointCloud::Ptr object_scan_collision;
	Eigen::Matrix4f object_transform;
	
	std::string output_json;

	PoseEstimation();
	~PoseEstimation();

	std::string MatrixToString();
	std::string GetTransformation(std::string parameters);
};
