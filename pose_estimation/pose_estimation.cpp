// pose_estimation.cpp
//
#include "stdafx.h"
#define __DLLEXPORT
#include "pose_estimation.h"

std::string ModelFileName = "";
std::string ScanFileName = "";

PoseEstimation::PoseEstimation(char algorithm_vision) :
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>)
{
	JsonOutType json_reader;
  std::string JsonFileName = "Config//pose_estimation.json";

	switch (algorithm_vision)
	{
		debug_visualization = false;
		sensor_offline = true;
		sample_3d = 0.01;

		case 'A':
			p_sensor_ = GetCameraData();
			p_registration_ = GetRegistration3D();
			p_recognition_ = GetRecognition3DPPF();
			p_seg_sac_ = GetSegmentationSAC();


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

			if (false == p_sensor_->LoadPLY(ModelFileName, object_model))
			{
				LOG(ERROR) << "LoadModel Error!";
			}
			else
				p_sensor_->ConvertPointsMMtoM(object_model);
			break;
		
		case 'B':
			break;

		default:
			break;
	}
}

PoseEstimation::~PoseEstimation()
{
}

bool PoseEstimation::Algorithm_Test()
{

	if (false == p_sensor_->LoadPointCloud(ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadModel Error!";
	}
	if (false == p_sensor_->LoadPointCloud(ScanFileName, object_scan))
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

bool PoseEstimation::Algorithm_A(PointCloud::Ptr cloud_scene, unsigned char view_point, Eigen::Matrix4f pose_object)
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
		if (cloud_scene->size() < 1000)
		{
			LOG(ERROR) << "input pointcloud size < 1000";
			return false;
		}
		else
		{
			object_scan = cloud_scene;
			p_sensor_->ConvertPointsMMtoM(object_scan);
		}
	}

	if (1 == view_point)
	{
		LOG(INFO) << "start algorithm at viewpoint 1";
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
		p_registration_->ComputeTransformation(object_model, object_scan, sample_3d, debug_visualization);
		object_transform = p_registration_->GetTransformation();
		cout << object_transform << endl;
	}
	else if (2 == view_point)
	{
		LOG(INFO) << "start algorithm at viewpoint 2";

	}
	else
	{
		LOG(ERROR) << "viewpoint error !";
		return false;
	}
	return true;
}

bool PoseEstimation::Algorithm_B()
{
	return false;
}

bool PoseEstimation::Algorithm_C()
{
	return false;
}

IPoseEstimation * GetInstance(char algothrim_version)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(algothrim_version);
	return p_pose_estimation_;
}

