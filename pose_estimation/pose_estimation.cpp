﻿// pose_estimation.cpp
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "plugins//PoseEstimation//Model_3D//object_model.pcd";
std::string JsonFileName = "plugins//PoseEstimation//Config//realsense_d435.json";

std::string TestJsonInput = "{\"CmdCode\": 1015,\"MessageBody\": {\"ScaleFactor3D\": 1000,\"Sample3D\": 0.005, \"Visualization\": true}, \"Plugin\": \"PoseEstimation.pln\"}";

PoseEstimation::PoseEstimation():
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan_collision(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_realsense_ = GetCameraData();
	p_registration_ = GetRegistration3D();

	pose_flag = false;
	debug_visualization = true;
	sample_3d = 0.005;

	object_transform = Eigen::Matrix4f::Ones();
	object_mask = cv::Mat::ones(cv::Size(1280, 720), CV_8UC1);
	object_mask_collision = cv::Mat::ones(cv::Size(1280, 720), CV_8UC1);
}

PoseEstimation::~PoseEstimation()
{
}

std::string PoseEstimation::MatrixToString()
{
	std::string output_str = "\"array\":\"[\n[";
	std::string matrix4f_str;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			matrix4f_str = matrix4f_str + std::to_string(object_transform(i, j));
			if (j < 3)
			{
				matrix4f_str = matrix4f_str + ",";
			}
		}
		if(i < 3)
		{
			matrix4f_str = matrix4f_str + "],\n[";
		}
	}
	output_str = output_str + matrix4f_str + "]\n]\"";
	//LOG(INFO) << output_str;
	return output_str;
}

std::string PoseEstimation::GetTransformation(std::string input_string)
{
	//if (p_realsense_->ReadJsonString(TestJsonInput, "MessageBody", "string").success)
	//{
	//	std::string message_body = p_realsense_->ReadJsonString(TestJsonInput, "MessageBody", "string").json_string;

	//	if (p_realsense_->ReadJsonString(message_body, "Visualization", "bool").success)
	//	{
	//		debug_visualization = p_realsense_->ReadJsonString(message_body, "Visualization", "bool").json_bool;
	//		LOG(INFO) << "Visualization(json input) : " << debug_visualization;
	//	}

	//	if (p_realsense_->ReadJsonString(message_body, "Sample3D", "float").success)
	//	{
	//		sample_3d = p_realsense_->ReadJsonString(message_body, "Sample3D", "float").json_float;
	//		LOG(INFO) << "Sample3D(json input) : " << sample_3d;
	//	}
	//}
	//else
	//	LOG(ERROR) << "Read input json error!";

	std::string output_string = "\"pose_flag\":";

	if (false == p_realsense_->SetParameters(JsonFileName))
	{
		LOG(ERROR) << "SetParameters Error!";
		return "{" + output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->GetCameraImages(object_color, object_depth))
	{
		LOG(ERROR) << "GetCameraImages Error!";
		return "{" + output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->LoadPointCloud(ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadPointCloud Error!";
		return "{" + output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->GetMaskAndLabel(object_mask, object_mask_collision, object_label))
	{
		LOG(ERROR) << "GetMaskAndLabel Error!";
		return "{" + output_string + "\"false\"" + "}";
	}
	
	if (false == p_realsense_->DepthtoPointCloud(object_depth, object_mask, object_scan))
	{
		LOG(ERROR) << "DepthtoPointCloud Error!";
		return "{" + output_string + "\"false\"" + "}";
	}

	if (false == p_realsense_->DepthtoPointCloud_Collision(object_depth, object_mask_collision, object_scan_collision))
	{
		LOG(ERROR) << "DepthtoPointCloud_Collision Error!";
		return "{" + output_string + "\"false\"" + "}";
	}

	if (debug_visualization)
	{
		LOG(INFO) << "Object label : " << object_label;
		//p_realsense_->ShowImage(object_color, "object_color");
		//p_realsense_->ShowImage(object_depth, "object_depth");
		//p_realsense_->ShowImage(object_mask, "object_mask");
		p_realsense_->ShowPointCloud(object_model, "object_model");
		p_realsense_->ShowPointCloud(object_scan, "object_scan");
		p_realsense_->ShowPointCloud(object_scan_collision, "object_collision");
	}

	p_registration_->ComputeTransformation(object_model, object_scan, sample_3d, debug_visualization);
	object_transform = p_registration_->GetTransformation();
	LOG(INFO)<<"\n"<< object_transform;
	output_json = "{" + output_string + "\"true\",\n" + MatrixToString() + "}";
	LOG(INFO)<<"\n"<< output_json;
	return output_json;
}

IPoseEstimation * GetPoseEstimation()
{
  IPoseEstimation* p_pose_estimation_ = new PoseEstimation(); 
  return p_pose_estimation_;
}

