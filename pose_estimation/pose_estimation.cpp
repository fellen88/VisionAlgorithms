// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

PoseEstimation::PoseEstimation()
{
	p_camera_data_ = GetCameraData();
	p_registration_ = GetRegistration3D();
	
	pose_flag = false;
}

PoseEstimation::~PoseEstimation()
{
}

std::string PoseEstimation::GetTransformation(std::string parameters)
{
	if (false == p_camera_data_->SetParameters())
	{
		pose_flag = false;
		return "pose_flag false";
	}
	return parameters + "has been passed to plugin!";
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

