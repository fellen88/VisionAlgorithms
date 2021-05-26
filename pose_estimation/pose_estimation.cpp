﻿// pose_estimation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "Model_3D/object_model.pcd";
std::string JsonFileName = "plugins/DCP/config/pose_estimation.json";
std::string TestOutput = "\"array\":[[1, 2, 3, 4], [5, 6, 7, 8], [9, 10, 11, 12], [13, 14, 15 ,16]]}"; 

PoseEstimation::PoseEstimation():
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_realsense_ = GetCameraData();
	p_registration_ = GetRegistration3D();

	pose_flag = false;
	debug_visualization = false;
	sample_3d = 0.005;
}

PoseEstimation::~PoseEstimation()
{
}

std::string PoseEstimation::GetTransformation(std::string input_string)
{
	//if(p_realsense_->ReadJsonString(input_string, "Visualization", "bool").success)
	//debug_visualization = p_realsense_->ReadJsonString(input_string, "Visualization", "bool").json_bool;

	//if(p_realsense_->ReadJsonString(input_string, "Sample3D", "float").success)
	//debug_visualization = p_realsense_->ReadJsonString(input_string, "Sample3D", "float").json_float;

	std::string output_string = "{\"pose_flag\":";

	//if (false == p_realsense_->SetParameters(JsonFileName))
	//{
	//	LOG(ERROR) << "SetParameters Error!";
	//	return output_string + "\"false\"" + "}";
	//}

	//if (false == p_realsense_->GetSharedMemImages(object_color, object_depth, object_mask, object_label))
	//{
	//	LOG(ERROR) << "GetSharedMemImages Error!";
	//	return output_string + "\"false\"" + "}";
	//}

	//if (false == p_realsense_->LoadPointCloud(ModelFileName, object_model))
	//{
	//	LOG(ERROR) << "LoadPointCloud Error!";
	//	return output_string + "\"false\"" + "}";
	//}
	//
	//if (false == p_realsense_->DepthtoPointCloud(object_depth, object_mask, object_scan))
	//{
	//	LOG(ERROR) << "DepthtoPointCloud Error!";
	//	return output_string + "\"false\"" + "}";
	//}
	//if (debug_visualization)
	//{
	//	p_realsense_->ShowImage(object_color, "object_color");
	//	p_realsense_->ShowImage(object_depth, "object_depth");
	//	p_realsense_->ShowPointCloud(object_model, "object_model");
	//	p_realsense_->ShowPointCloud(object_scan, "object_scan");
	//}

	if (false == p_realsense_->LoadPointCloud("Model_3D/lion.pcd", object_model))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
		return output_string + "\"false\"" + "}";
	}
	if (false == p_realsense_->LoadPointCloud("Model_3D/lion_trans.pcd", object_scan))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
		return output_string + "\"false\"" + "}";
	}
	p_registration_->ComputeTransformation(object_scan, object_model, sample_3d);
	//object_transform = p_registration_->GetTransformation();
	//cout << object_transform << endl;
	return output_string + "\"true\"," + TestOutput;
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

