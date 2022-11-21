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
	switch (algorithm_vision)
	{
		debug_visualization = false;
		sensor_offline = true;
		sample_3d = 0.01;

		case 'A':
			p_sensor_ = GetCameraData();
			SetParameters_A();
			p_registration_ = GetRegistration3D();
			p_seg_sac_ = GetSegmentationSAC(seg_sac_config);

			//load ply model
			if (false == p_sensor_->LoadPLY(ModelFileName, object_model))
			{
				LOG(ERROR) << "LoadModel Error!";
			}
			else
				p_sensor_->ConvertPointsMMtoM(object_model);

			break;
		
		case 'B':
			p_sensor_ = GetCameraData();
			SetParameters_B();
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
			p_recognition_->TrainPPFModel(cloud_models);

			break;

		default:
			break;
	}
}

PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::SetParameters_A()
{
  std::string JsonFileName = "Config//Algorithm_A//algorithm_a.json";
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
}

bool PoseEstimation::Algorithm_A(std::vector<double> object_points, unsigned char view_point, std::vector<double>& object_pose)
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
			p_sensor_->VectorPointstoPCL(object_points, object_scan, object_scene_normal);
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
		//cout << object_transform << endl;
		p_sensor_->Matrix2EulerAngle(object_transform, object_eulerangle);

		object_pose.clear();
		object_pose.push_back(object_transform(0,3));
		object_pose.push_back(object_transform(1,3));
		object_pose.push_back(object_transform(2,3));
		object_pose.push_back(object_eulerangle[2]);
		object_pose.push_back(object_eulerangle[1]);
		object_pose.push_back(object_eulerangle[0]);
		for (auto it = object_pose.begin(); it != object_pose.end(); ++it) {
			std::cout << *it << " ";
		}
		std::cout << std::endl;
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

void PoseEstimation::SetParameters_B()
{
  std::string JsonFileName = "Config//Algorithm_B//algorithm_b.json";
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
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegmSAC_Config", "string");
	if (json_reader.success)
		seg_sac_config = json_reader.json_string;
}

bool PoseEstimation::Algorithm_B(std::vector<double> object_points, unsigned char view_point, std::vector<double>& object_pose)
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
			p_sensor_->VectorPointstoPCL(object_points, object_scan, object_scene_normal);
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
		p_recognition_->Compute(object_scan,cloud_models);
		return true;
	}
	return false;
}

bool PoseEstimation::Algorithm_C()
{
	return false;
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

IPoseEstimation * GetInstance(char algothrim_version)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(algothrim_version);
	return p_pose_estimation_;
}

