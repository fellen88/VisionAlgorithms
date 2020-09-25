// pose_estimation.cpp
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "plugins//PoseEstimation//Model_3D//object_model.pcd";

PoseEstimation::PoseEstimation():
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_realsense_ = GetCameraData();
	p_registration_ = GetRegistration3D();

	debug_visualization = true;
	sample_3d = 0.005;
}

PoseEstimation::~PoseEstimation()
{
}

bool PoseEstimation::GetTransformation()
{

	if (false == p_realsense_->LoadPointCloud(ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
	}
	if (false == p_realsense_->LoadPointCloud(ModelFileName, object_scan))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
	}

	if (debug_visualization)
	{
		p_realsense_->ShowPointCloud(object_model, "object_model");
		p_realsense_->ShowPointCloud(object_scan, "object_scan");
	}

	p_registration_->ComputeTransformation(object_model, object_scan, sample_3d, debug_visualization);
	object_transform = p_registration_->GetTransformation();
	cout << object_transform << endl;
	return true;
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

