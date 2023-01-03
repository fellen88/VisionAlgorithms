﻿// pose_estimation.cpp
//
#include "stdafx.h"

#define __POSE_ESTIMATION_EXPORT
#include "pose_estimation.h"

PoseEstimation::PoseEstimation(char algorithm_vision) :
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>),
	sac_output(new pcl::PointCloud<pcl::PointXYZ>),
	obb_output(new pcl::PointCloud<pcl::PointXYZ>),
	object_output(new pcl::PointCloud<pcl::PointXYZ>)
{
	switch (algorithm_vision)
	{
		debug_visualization = false;
		sensor_offline = true;
		sample_3d = 0.01;
		ModelFileName = "Model_3D//model.ply";
		ScanFileName = "PointCloud//scene.ply";

		case 'A':
			p_sensor_ = GetCameraData();
			SetParameters_A();
			p_registration_ = GetRegistration3D(lmicp_config);
			p_seg_sac_ = GetSegmentationSAC(seg_sac_config);
			p_recognition_ = GetRecognition3DPPF(ppf_config);
			//load ply model
			if (false == p_sensor_->LoadPLY(ModelFileName, object_model))
			{
				LOG(ERROR) << "LoadModel Error!";
			}
			else
				p_sensor_->ConvertPointsMMtoM(object_model);

			cloud_models.push_back(object_model);
			//p_recognition_->TrainPPFModel(cloud_models);
			break;
		
		case 'B':
			p_sensor_ = GetCameraData();
			SetParameters_B();
			p_registration_ = GetRegistration3D(lmicp_config);
			p_seg_sac_ = GetSegmentationSAC(seg_sac_config);
			p_seg_obb_ = GetSegmentationOBB(seg_obb_config);
			p_recognition_ = GetRecognition3DPPF(ppf_config);

			sac_transform = Eigen::Matrix4f::Identity();
			object_transform = Eigen::Matrix4f::Identity();
			//load ply model
			if (false == p_sensor_->LoadPLY(ModelFileName, object_model))
			{
				LOG(ERROR) << "LoadModel Error!";
			}
			else
				p_sensor_->ConvertPointsMMtoM(object_model);

			cloud_models.push_back(object_model);
			p_recognition_->TrainPPFModel(cloud_models);
			break;

		default:
			break;
	}
}

PoseEstimation::~PoseEstimation()
{
	LOG(INFO) << "~PoseEstimation()";
	if (p_sensor_ != nullptr)
	{
	  delete p_sensor_;
	}
	//TODO:DELETE
}

void PoseEstimation::SetParameters_A()
{
  const std::string JsonFileName = "Config//Algorithm_A//algorithm_a.json";
	JsonOutType json_reader;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "DebugVisualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SensorOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Sample3D", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "ObjectModelPath", "string");
	if (json_reader.success)
		ModelFileName = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PointCloudPath", "string");
	if (json_reader.success)
		ScanFileName = json_reader.json_string;

	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegSAC_Config", "string");
	if (json_reader.success)
		seg_sac_config = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PPF_Config", "string");
	if (json_reader.success)
		ppf_config = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "LMICP_Config", "string");
	if (json_reader.success)
		lmicp_config = json_reader.json_string;
}

bool PoseEstimation::Algorithm_A(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose)
{
	if (sensor_offline)
	{
		if (false == p_sensor_->LoadPLY(ScanFileName, object_scan))
		{
			LOG(ERROR) << "LoadPointCloud Error!";
			return false;
		}
		else
			LOG(INFO) << "Load PLY on sensor off mode";
			p_sensor_->ConvertPointsMMtoM(object_scan);
	}
	else
	{
		if (object_points.size() < 1000)
		{
			LOG(ERROR) << "input pointcloud size < 1000";
			return false;
		}
		else
		{
			LOG(INFO) << "Read pointcloud data on sensor on mode";
			pcl::copyPointCloud(object_points, *object_scan);
			p_sensor_->ConvertPointsMMtoM(object_scan);
		}
	}
	if (debug_visualization)
	{
		p_sensor_->ShowPointCloud(object_model, "object_model");
		p_sensor_->ShowPointCloud(object_scan, "object_scan");
	}

	if (1 == view_point)
	{
		LOG(INFO) << "start algorithm at viewpoint 1";

	  p_seg_sac_->segment(object_scan);
		if (debug_visualization)
		{
			p_sensor_->ShowPointCloud(object_scan, "object_seg");
		}
		p_registration_->ComputeTransformation(object_model, object_scan, sample_3d, debug_visualization);
		object_transform = p_registration_->GetTransformation();
		cout << object_transform << endl;
		p_sensor_->Matrix2EulerAngle(object_transform, object_eulerangle);

		object_pose.clear();
		object_pose.push_back(object_transform(0,3));
		object_pose.push_back(object_transform(1,3));
		object_pose.push_back(object_transform(2,3));
		object_pose.push_back(object_eulerangle[2]);
		object_pose.push_back(object_eulerangle[1]);
		object_pose.push_back(object_eulerangle[0]);
		return true;
	}
	else if (2 == view_point)
	{
		LOG(INFO) << "start algorithm at viewpoint 2";
		return false;
	}
	else
	{
		LOG(ERROR) << "viewpoint error !";
		return false;
	}
}

void PoseEstimation::SetParameters_B()
{
  const std::string JsonFileName = "Config//Algorithm_B//algorithm_b.json";
	JsonOutType json_reader;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "DebugVisualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SensorOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Sample3D", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "ObjectModelPath", "string");
	if (json_reader.success)
		ModelFileName = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PointCloudPath", "string");
	if (json_reader.success)
		ScanFileName = json_reader.json_string;

	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PPF_Config", "string");
	if (json_reader.success)
		ppf_config = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegSAC_Config", "string");
	if (json_reader.success)
		seg_sac_config = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "LMICP_Config", "string");
	if (json_reader.success)
		lmicp_config = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegOBB_Config", "string");
	if (json_reader.success)
		seg_obb_config = json_reader.json_string;
}

bool PoseEstimation::Algorithm_B(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose)
{
	if (sensor_offline)
	{
		if (false == p_sensor_->LoadPLY(ScanFileName, object_scan))
		{
			LOG(ERROR) << "LoadPointCloud Error!";
			return false;
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_scan);
	}
	else
	{
		if (object_points.size() < 1000)
		{
			LOG(ERROR) << "input pointcloud size < 1000";
			return false;
		}
		else
		{
			LOG(INFO) << "Read pointcloud data on sensor on mode";
			pcl::copyPointCloud(object_points, *object_scan);
			p_sensor_->ConvertPointsMMtoM(object_scan);
		}
	}

	if (1 == view_point)
	{
		LOG(INFO) << "start algorithm_b at viewpoint 1";
		if (debug_visualization)
		{
			p_sensor_->ShowPointCloud(object_model, "object_model");
			p_sensor_->ShowPointCloud(object_scan, "object_scan");
		}
		p_seg_sac_->segment(object_scan);
		if (debug_visualization)
		{
			p_sensor_->ShowPointCloud(object_scan, "object_seg");
		}
		//p_recognition_->Compute(object_scan, cloud_models);
		p_registration_->SAC_IA(object_model, object_scan, sac_output, sac_transform, sample_3d, debug_visualization);
		p_seg_obb_->segment(object_scan, sac_output, obb_output);
		p_registration_->SAC_IA(object_model, obb_output, sac_output, sac_transform, sample_3d, debug_visualization);
		p_registration_->LM_ICP(obb_output, sac_output, object_output, object_transform);

		object_transform = object_transform * sac_transform;
		cout << object_transform << endl;
		p_sensor_->Matrix2EulerAngle(object_transform, object_eulerangle);
		object_pose.clear();
		object_pose.push_back(object_transform(0, 3));
		object_pose.push_back(object_transform(1, 3));
		object_pose.push_back(object_transform(2, 3));
		object_pose.push_back(object_eulerangle[2]);
		object_pose.push_back(object_eulerangle[1]);
		object_pose.push_back(object_eulerangle[0]);
		return true;
	}
	else
		return false;
}

IPoseEstimation * GetInstance(char algothrim_version)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(algothrim_version);
	return p_pose_estimation_;
}

