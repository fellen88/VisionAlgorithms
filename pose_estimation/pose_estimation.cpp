// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "plugins//PoseEstimation//object_model.pcd";
std::string JsonFileName = "plugins//PoseEstimation//pose_estimation.json";
//std::string TestJsonString = "{\"key\":\"value\",\"array\":[{\"arraykey\":1},{\"arraykey\":2}]}"; 

PoseEstimation::PoseEstimation():
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_realsense_ = GetCameraData();
	p_registration_ = GetRegistration3D();

	pose_flag = false;
}

PoseEstimation::~PoseEstimation()
{
}

std::string PoseEstimation::GetTransformation(std::string input_string)
{
	cout << "Visualization ：";
	bool debug_visualization = p_realsense_->ReadJsonString(input_string, "Visualization", "bool").json_bool;
	std::string output_string = "{\"pose_flag\":";

	if (false == p_realsense_->SetParameters(JsonFileName))
	{
		LOG(ERROR) << "SetParameters Error!";
		return output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->GetSharedMemImages(object_depth, object_color, object_mask, object_label))
	{
		LOG(ERROR) << "GetSharedMemImages Error!";
		return output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->LoadPointCloud(ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
		return output_string + "\"false\"" + "}";
	}
	
	//if (false == p_realsense_->DepthtoPointCloud(object_depth, object_mask, object_scan));
	//{
	//	LOG(ERROR) << "DepthtoPointCloud Error!";
	//	return output_string + "\"false\"" + "}";
	//}
	//if (debug_visualization)
	{
		p_realsense_->ShowImage(object_color, "object_color");
		p_realsense_->ShowImage(object_color, "object_depth");
		p_realsense_->ShowPointCloud(object_model, "object_model");
	}

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

