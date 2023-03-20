#include "stdafx.h"

#define __POSE_ESTIMATION_EXPORT
#include "pose_estimation.h"

using namespace val;

PoseEstimation::PoseEstimation(unsigned char algorithm, std::string config_file) :
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_part1(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_part2(new pcl::PointCloud<pcl::PointXYZ>),
	obb_part1(new pcl::PointCloud<pcl::PointXYZ>),
	obb_part2(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan(new pcl::PointCloud<pcl::PointXYZ>),
	sac_output(new pcl::PointCloud<pcl::PointXYZ>),
	obb_output(new pcl::PointCloud<pcl::PointXYZ>),
	object_output(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan_segsac(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_segsac(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan_downsample(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_downsample(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan_segeucli(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_segeucli(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan_instance(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_instance(new pcl::PointCloud<pcl::PointXYZ>),
	object_scan_preprocess(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_preprocess(new pcl::PointCloud<pcl::PointXYZ>),
	object_scene_curvature(new PointCloudWithCurvatures()),
	object_model_curvature(new PointCloudWithCurvatures()),
	object_model_normal(new PointCloudWithNormals()),
	object_scene_normal(new PointCloudWithNormals()),
	object_model_edge(new PointCloud()),
	object_scene_edge(new PointCloud()),
	p_sensor_(GetCameraData()),
	p_recog_ppf_(nullptr),
	p_seg_sac_(nullptr),
	p_refine_seg_obb_(nullptr),
	p_refine_seg_bound_(nullptr),
	p_instance_seg_bound_(nullptr),
	p_seg_eucli_(nullptr),
	p_refine_seg_eucli_(nullptr),
	p_regist_lmicp_(nullptr),
	p_refine_regist_lmicp_(nullptr)
{
	grasp_method = algorithm;

	switch (algorithm)
	{
	case BinPicking:
		Init_BinPicking(config_file);
		break;

	default:
		LOG(ERROR) << "GraspName Error !";
		break;
	}
}

PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::UpdateParameters(std::string config)
{
	std::string JsonFileName = config;
	JsonOutType json_reader;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Visualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SensorOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Sample3D", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "InstanceSeg", "string");
	if (json_reader.success)
		instance_seg = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "InstanceBoundary", "bool");
	if (json_reader.success)
		edge_normal = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Refine_1", "bool");
	if (json_reader.success)
		refine_1 = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Refine_2", "bool");
	if (json_reader.success)
		refine_2 = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "ObjectModelPath", "string");
	if (json_reader.success)
		ModelFileName = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PointCloudPath", "string");
	if (json_reader.success)
		ScanFileName = json_reader.json_string;

	std::string config_file;
	std::string config_path;
	std::string project_file;
	p_sensor_->GetSubPath(config, config_file, 1);
	p_sensor_->GetSubPath(config, project_file, 3);
	config_path = config.erase(config.find(config_file), config_file.size());
	LOG(INFO) << "********************************************";
	LOG(INFO) << "project name: " << project_file;
	LOG(INFO) << "********************************************";

	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegSAC_Config", "string");
	if (json_reader.success)
		seg_sac_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Recog_PPF_Config", "string");
	if (json_reader.success)
		recog_ppf_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "LMICP_Config", "string");
	if (json_reader.success)
		lmicp_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "LMICP_Refine_Config", "string");
	if (json_reader.success)
		lmicp_refine_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegOBB_Config", "string");
	if (json_reader.success)
		seg_obb_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegOBB_Instance_Config", "string");
	if (json_reader.success)
		seg_obb_instance_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegBoundary_Config", "string");
	if (json_reader.success)
		seg_bound_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Instance_SegBoundary_Config", "string");
	if (json_reader.success)
		instance_seg_bound_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegEucli_Config", "string");
	if (json_reader.success)
		seg_eucli_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegEucli_Refine_Config", "string");
	if (json_reader.success)
		seg_eucli_refine_config = config_path + json_reader.json_string;
}

void PoseEstimation::Init_BinPicking(std::string config)
{
	//reset shared_ptr
	p_refine_seg_obb_.reset(GetSegmentationOBB());
	p_seg_obb_instance_.reset(GetSegmentationOBB());
	p_refine_seg_bound_.reset(GetSegmentationBoundary());
	p_instance_seg_bound_.reset(GetSegmentationBoundary());
	p_seg_eucli_.reset(GetSegmentationEuclidean());
	p_refine_seg_eucli_.reset(GetSegmentationEuclidean());
	p_recog_ppf_.reset(GetRecognition3DPPF());
	p_regist_lmicp_.reset(GetRegistrationLMICP());
	p_refine_regist_lmicp_.reset(GetRegistrationLMICP());

	//init variable
	sac_transform = Eigen::Matrix4f::Identity();
	object_transform = Eigen::Matrix4f::Identity();
	object_transform_init = Eigen::Matrix4f::Identity();
	object_transform_refine = Eigen::Matrix4f::Identity();

	//set parameters
	UpdateParameters(config);
	p_regist_lmicp_->SetParameters(lmicp_config);
	if (refine_1 || refine_2)
	{
		p_refine_seg_obb_->SetParameters(seg_obb_config);
		p_refine_seg_bound_->SetParameters(seg_bound_config);
		p_refine_seg_eucli_->SetParameters(seg_eucli_refine_config);
		p_refine_regist_lmicp_->SetParameters(lmicp_refine_config);
	}

	//load ply model
	std::string project_file;
	p_sensor_->GetSubPath(config, project_file, 3);
	if (false == p_sensor_->LoadPLY(project_file + "\\" + ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadModel Error!";
	}
	else
		p_sensor_->ConvertPointsMMtoM(object_model);
	if (true == refine_1)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + ModelFileName.erase(ModelFileName.find("."), 4) + "_refine_1.ply", object_model_part1))
		{
			LOG(ERROR) << "Load" + project_file + "\\" + ModelFileName.erase(ModelFileName.find("."), 4) + "_refine_1.ply" + " Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model_part1);
	}
	if (true == refine_2)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + ModelFileName + "_refine_2.ply", object_model_part2))
		{
			LOG(ERROR) << "Load" + project_file + "\\" + ModelFileName + "_refine_2.ply" + "Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model_part2);
	}

	//downsampling
	pcl::copyPointCloud(*object_model, *object_model_downsample);
	p_sensor_->DownSample(object_model_downsample, Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f));
	//instance
	if ("Euclidean" == instance_seg)
	{
		p_seg_eucli_->SetParameters(seg_eucli_config);
		p_seg_eucli_->Segment(object_model_downsample, nullptr, object_model_segeucli);
		pcl::copyPointCloud(*object_model_segeucli, *object_model_instance);
	}
	else if ("PPF_OBB" == instance_seg)
	{
		p_recog_ppf_->SetParameters(recog_ppf_config);
		p_seg_obb_instance_->SetParameters(seg_obb_instance_config);
		cloud_models.push_back(object_model);
		p_recog_ppf_->TrainPPFModel(cloud_models);
	}
	else
	{
		pcl::copyPointCloud(*object_model_downsample, *object_model_instance);
	}
	//edge normal
	if (true == edge_normal)
	{
		p_instance_seg_bound_->SetParameters(instance_seg_bound_config);
		p_instance_seg_bound_->Segment(object_model_instance, nullptr, object_model_edge);
		pcl::copyPointCloud(*object_model_edge, *object_model_preprocess);
	}
	else
		pcl::copyPointCloud(*object_model_instance, *object_model_preprocess);
}

bool PoseEstimation::Compute_BinPicking(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f& object_pose)
{
	//downsampling
	pcl::copyPointCloud(*object_scan, *object_scan_downsample);
	p_sensor_->DownSample(object_scan_downsample, Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f));
	//object instance
	if ("Euclidean" == instance_seg)
	{
		p_seg_eucli_->Segment(object_scan_downsample, nullptr, object_scan_segeucli);
		pcl::copyPointCloud(*object_scan_segeucli, *object_scan_instance);
	}
	else if ("PPF_OBB" == instance_seg)
	{
		p_recog_ppf_->Compute(object_scan_downsample, cloud_models, object_transform);
		PointCloud::Ptr model_instance_transformed(new PointCloud());
		pcl::transformPointCloud(*object_model_instance, *model_instance_transformed, object_transform);
		p_seg_obb_instance_->Segment(object_scan_downsample, model_instance_transformed, object_scan_instance);
	}
	else
		pcl::copyPointCloud(*object_scan_downsample, *object_scan_instance);

	//object edge 
	if (true == edge_normal)
	{
		p_instance_seg_bound_->Segment(object_scan_instance, nullptr, object_scene_edge);
		pcl::copyPointCloud(*object_scene_edge, *object_scan_preprocess);
	}
	else
		pcl::copyPointCloud(*object_scan_instance, *object_scan_preprocess);

	if (debug_visualization)
	{
		p_sensor_->ShowPointCloud(object_scene_edge, "object_scene_edge");
		p_sensor_->ShowPointCloud(object_model_edge, "object_model_edge");
	}
	//registration
	p_regist_lmicp_->SAC_IA(object_model_preprocess, object_scan_preprocess, sac_output, sac_transform, 0.003, debug_visualization);
	p_regist_lmicp_->LM_ICP(object_scan_preprocess, sac_output, object_output, object_transform);
	object_transform = object_transform * sac_transform;

	//registration refine
	if (refine_1 || refine_2)
	{
		PointCloud::Ptr model_part1_transformed(new PointCloud());
		PointCloud::Ptr model_part2_transformed(new PointCloud());
		PointCloud::Ptr model_part_transformed(new PointCloud());
		PointCloud::Ptr euclidean_part1(new PointCloud());
		PointCloud::Ptr euclidean_part2(new PointCloud());
		PointCloud::Ptr euclidean_part(new PointCloud());
		PointCloud::Ptr model_part_boundary(new PointCloud());
		PointCloud::Ptr obb_part_boundary(new PointCloud());
		if (refine_1)
		{
			//OBB Segmentation
			pcl::transformPointCloud(*object_model_part1, *model_part1_transformed, object_transform);
			p_refine_seg_obb_->Segment(object_scan, model_part1_transformed, obb_part1);
		}
		if (refine_2)
		{
			//OBB Segmentation
			pcl::transformPointCloud(*object_model_part2, *model_part2_transformed, object_transform);
			p_refine_seg_obb_->Segment(object_scan, model_part2_transformed, obb_part2);
		}
		*model_part_transformed = *model_part1_transformed + *model_part2_transformed;
		*obb_output = *obb_part1 + *obb_part2;
		//Euclidean
		if(refine_1)
			p_refine_seg_eucli_->Segment(obb_part1, nullptr, euclidean_part1);
		if(refine_2)
			p_refine_seg_eucli_->Segment(obb_part2, nullptr, euclidean_part2);
		*euclidean_part = *euclidean_part1 + *euclidean_part2;
		//Boundary  
		p_refine_seg_bound_->Segment(model_part_transformed, nullptr, model_part_boundary);
		p_refine_seg_bound_->Segment(euclidean_part, nullptr, obb_part_boundary);
		//LM-ICP Refine
		if (refine_1&&refine_2)
		{
			p_refine_regist_lmicp_->LM_ICP(obb_part_boundary, model_part_boundary, object_output, object_transform_refine);
			object_transform = object_transform_refine * object_transform;
		}
		else
		{
			p_refine_regist_lmicp_->SAC_IA(model_part_boundary, obb_part_boundary, sac_output, sac_transform, 0.002, debug_visualization);
			p_refine_regist_lmicp_->LM_ICP(obb_part_boundary, sac_output, object_output, object_transform_refine);
			object_transform = object_transform_refine * object_transform * sac_transform;
		}
	}
	//output
	LOG(INFO) << "transformation matrix: \n \0.000000f" << object_transform;
	object_pose = object_transform;

	return true;
}

bool PoseEstimation::Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose)
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

	Eigen::Matrix4f object_pose_matrix;
	switch (grasp_method)
	{
	case AccuracyGrasp:
		break;

	case BinPicking:
		Compute_BinPicking(object_points, object_pose_matrix);
		break;

	default:
		LOG(ERROR) << "GraspName Error !";
		break;
	}

	p_sensor_->Matrix2EulerAngle(object_pose_matrix, object_eulerangle);

	object_pose.clear();
	object_pose.push_back(object_pose_matrix(0, 3));
	object_pose.push_back(object_pose_matrix(1, 3));
	object_pose.push_back(object_pose_matrix(2, 3));
	object_pose.push_back(object_eulerangle[2] * 180 / M_PI);
	object_pose.push_back(object_eulerangle[1] * 180 / M_PI);
	object_pose.push_back(object_eulerangle[0] * 180 / M_PI);

	return true;
}

IPoseEstimation * GetInstance(char algothrim_version, std::string config_file)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(algothrim_version, config_file);
	return p_pose_estimation_;
}

