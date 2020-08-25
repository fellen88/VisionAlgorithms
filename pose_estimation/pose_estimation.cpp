// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "object_model.pcd";
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
	cout << "Visualization ：";
	std::string test = p_camera_data_->ReadJsonFile(JsonFilePath, "Visualization", "string").json_string;
	//std::string test1 = p_camera_data_->ReadJsonString(JsonString, "key", "string").json_string;

	std::string output_string = "{\"pose_flag\":";
	//if (false == p_camera_data_->GetSharedMemImages(object_depth, object_color, object_mask, object_label))
	//{
	//	LOG(ERROR) << "GetSharedMemImages Error!";
	//	return output_string + "\"false\"";
	//}
	//if (false == p_camera_data_->SetParameters())
	//{
	//	return output_string + "\"false\"";
	//}
	//if (false == p_camera_data_->DepthtoPointCloud(object_depth, object_mask, object_scan));
	//{
	//	return output_string + "\"false\"";
	//}
	//if (false == p_camera_data_->LoadPointCloud(ModelFileName, object_model))
	//{
	//	return output_string + "\"false\"";
	//}

	p_registration_->ComputeTransformation(object_model, object_scan);
	//object_transform = p_registration_->GetTransformation();
	return output_string;
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

