#pragma once
#include "ipose_estimation.h"
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"

#ifdef __DLLEXPORT
#define __DLL_INTERFACE _declspec(dllexport)    // �������� - ����dll�ļ�ʱʹ��
#else
#define __DLL_INTERFACE _declspec(dllimport)    // ���뺯�� -ʹ��dll��ʹ��
#endif

class PoseEstimation: public IPoseEstimation
{
	public:

	ICameraData* p_camera_data_;
	IRegistration3D *p_registration_;

	bool pose_flag;
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

	std::string GetTransformation(std::string parameters);
};
