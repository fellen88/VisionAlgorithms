// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "plugins//PoseEstimation//object_model.pcd";
std::string JsonFilePath = "plugins//PoseEstimation//pose_estimation.json";
std::string JsonString = "{\"key\":\"value\",\"array\":[{\"arraykey\":1},{\"arraykey\":2}]}"; 

PoseEstimation::PoseEstimation():
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>)
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

	if (false == p_camera_data_->SetParameters())
	{
		LOG(ERROR) << "SetParameters Error!";
		return output_string + "\"false\"" + "}";
	}

	//if (false == p_camera_data_->GetSharedMemImages(object_depth, object_color, object_mask, object_label))
	//{
	//	LOG(ERROR) << "GetSharedMemImages Error!";
	//	return output_string + "\"false\"" + "}";
	//}

	if (false == p_camera_data_->LoadPointCloud(ModelFileName, object_model))
	{
		return output_string + "\"false\"" + "}";
	}
	
	//if (false == p_camera_data_->DepthtoPointCloud(object_depth, object_mask, object_scan));
	//{
	//	return output_string + "\"false\"" + "}";
	//}
	//p_camera_data_->ShowPointCloud(object_model, "object");

	p_registration_->ComputeTransformation(object_model, object_model);
	object_transform = p_registration_->GetTransformation();
	cout << object_transform << endl;
	return output_string;
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

