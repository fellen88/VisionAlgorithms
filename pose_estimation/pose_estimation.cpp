// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string JsonFilePath = "plugins//PoseEstimation//pose_estimation.json";
std::string JsonString = "{\"key\":\"value\",\"array\":[{\"arraykey\":1},{\"arraykey\":2}]}"; 

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
	std::string test = p_camera_data_->ReadJsonFile(JsonFilePath, "Visualization", "string").json_string;
	std::string test1 = p_camera_data_->ReadJsonString(JsonString, "key", "string").json_string;
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

