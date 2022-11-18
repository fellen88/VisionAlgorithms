#pragma once
#include "ipose_estimation.h"
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"
#include"../recognition/irecognition.h"
#include"../segmentation/isegmentation.h"

#ifdef __DLLEXPORT
#define __DLL_INTERFACE _declspec(dllexport)    // 导出函数 - 生成dll文件时使用
#else
#define __DLL_INTERFACE _declspec(dllimport)    // 导入函数 -使用dll是使用
#endif

class PoseEstimation: public IPoseEstimation
{
	public:

	ICameraData* p_realsense_;
	ICameraData* p_sensor_;
	IRegistration3D* p_registration_;
	IRecognition* p_recognition_;
	ISegmentation* p_seg_sac_;

	bool pose_flag;
	bool debug_visualization;
	bool sensor_offline;
	float sample_3d;
	std::string pose_object;
	
	cv::Mat object_depth;
	cv::Mat object_color;
	cv::Mat object_mask;
	std::string object_label;
	PointCloud::Ptr object_model;
	PointCloud::Ptr object_scan;
	PointCloudWithNormals::Ptr object_scene_normal;
	Eigen::Matrix4f object_transform;
	Eigen::Vector3f object_eulerangle;

	std::string seg_sac_config;
	std::string ppf_config;
	std::vector<PointCloud::Ptr> cloud_models;

	PoseEstimation(char algorithm_version);
	~PoseEstimation();

	void SetParameters_A();
	bool Algorithm_Test();
	bool Algorithm_A(std::vector<double> object_points, unsigned char viewpoint, std::vector<double>& object_pose);
	void SetParameters_B();
	bool Algorithm_B(std::vector<double> object_points, unsigned char view_point, std::vector<double>& object_pose);
	bool Algorithm_C();
};
