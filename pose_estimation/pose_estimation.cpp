// pose_estimation.cpp
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "plugins//PoseEstimation//Model_3D//object_model.pcd";
std::string JsonFileName = "plugins//PoseEstimation//Config//realsense_d435.json";


PoseEstimation::PoseEstimation():
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_realsense_ = GetCameraData();
	p_registration_ = GetRegistration3D();

	pose_flag = false;
	debug_visualization = true;
	sample_3d = 0.005;
}

PoseEstimation::~PoseEstimation()
{
}

std::string PoseEstimation::GetTransformation(std::string input_string)
{

	std::string output_string = "{\"pose_flag\":";

	if (false == p_realsense_->SetParameters(JsonFileName))
	{
		LOG(ERROR) << "SetParameters Error!";
		return output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->GetCameraImages(object_color, object_depth))
	{
		LOG(ERROR) << "GetCameraImages Error!";
		return output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->LoadPointCloud(ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
		return output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->GetMaskAndLabel(object_mask, object_label))
	{
		LOG(ERROR) << "GetMaskAndLabel Error!";
		return output_string + "\"false\"" + "}";
	}
	
	if (false == p_realsense_->DepthtoPointCloud(object_depth, object_mask, object_scan))
	{
		LOG(ERROR) << "DepthtoPointCloud Error!";
		return output_string + "\"false\"" + "}";
	}
	if (debug_visualization)
	{
		LOG(INFO) << "Object label : " << object_label;
		//p_realsense_->ShowImage(object_color, "object_color");
		//p_realsense_->ShowImage(object_depth, "object_depth");
		//p_realsense_->ShowImage(object_mask, "object_mask");
		p_realsense_->ShowPointCloud(object_model, "object_model");
		p_realsense_->ShowPointCloud(object_scan, "object_scan");
	}

	p_registration_->ComputeTransformation(object_model, object_scan, sample_3d, debug_visualization);
	object_transform = p_registration_->GetTransformation();
	cout << object_transform << endl;
	return output_string + "\"true\",";
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

