#include "stdafx.h"

#define __POSE_ESTIMATION_EXPORT
#include "pose_estimation.h"

using namespace val;

PoseEstimation::PoseEstimation(char algorithm_vision, std::string config_file) :
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
	object_scene_curvature(new PointCloudWithCurvatures()),
	object_model_curvature(new PointCloudWithCurvatures()),
	object_model_normal(new PointCloudWithNormals()),
	object_scene_normal(new PointCloudWithNormals()),
	object_model_edge(new PointCloud()),
	object_scene_edge(new PointCloud()),
	p_sensor_(GetCameraData()),
	p_seg_sac_(nullptr),
	p_seg_obb_(nullptr),
	p_seg_bound_(nullptr),
	p_seg_eucli_(nullptr)
{
	debug_visualization = false;
	sensor_offline = true;
	sample_3d = 0.01;
	ModelFileName = "Model_3D//model.ply";
	ScanFileName = "PointCloud//scene.ply";

	switch (algorithm_vision)
	{
		case AccuracyGrasp:
			Init_AccuracyGrasp(config_file);
			break;
		
		case BinPicking:
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
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "DebugVisualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SensorOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PartRefine", "bool");
	if (json_reader.success)
		part_refine = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Sample3D", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Normal_Search_Radius", "float");
	if (json_reader.success)
		normal_search_radius = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Curvature_Thredhold", "float");
	if (json_reader.success)
		curvature_thredhold = json_reader.json_float;
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

	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegSAC_Config", "string");
	if (json_reader.success)
		seg_sac_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PPF_Config", "string");
	if (json_reader.success)
		ppf_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "LMICP_Config", "string");
	if (json_reader.success)
		lmicp_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "LMICP_Refine_Config", "string");
	if (json_reader.success)
		lmicp_refine_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegOBB_Config", "string");
	if (json_reader.success)
		seg_obb_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegBoundary_Config", "string");
	if (json_reader.success)
		seg_bound_config = config_path + json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SegEuclidean_Config", "string");
	if (json_reader.success)
		seg_eucli_config = config_path + json_reader.json_string;

	p_seg_sac_->SetParameters(seg_sac_config);
	p_seg_obb_->SetParameters(seg_obb_config);
	p_seg_bound_->SetParameters(seg_bound_config);
	p_seg_eucli_->SetParameters(seg_eucli_config);
}

void PoseEstimation::Init_AccuracyGrasp(std::string config)
{

	p_seg_sac_.reset(GetSegmentationSAC());
	p_seg_obb_.reset(GetSegmentationOBB());
	p_seg_bound_.reset(GetSegmentationBoundary());
	p_seg_eucli_.reset(GetSegmentationEuclidean());

	UpdateParameters(config);
	p_registration_ = GetRegistration3D(lmicp_config);
	p_registration_refine_ = GetRegistration3D(lmicp_refine_config);

	//init variable
	sac_transform = Eigen::Matrix4f::Identity();
	object_transform = Eigen::Matrix4f::Identity();
	object_transform_init = Eigen::Matrix4f::Identity();
	object_transform_refine = Eigen::Matrix4f::Identity();
	//load ply model
	//if (nullptr == object_model)
	{
		std::string project_file;
		p_sensor_->GetSubPath(config, project_file, 3);
		if (false == p_sensor_->LoadPLY(project_file + "\\" + ModelFileName, object_model))
		{
			LOG(ERROR) << "LoadModel Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model);
		if (false == p_sensor_->LoadPLY(project_file + "\\" + ModelFileName.erase(ModelFileName.find("."), 4) + "_part_1.ply", object_model_part1))
		{
			LOG(ERROR) << "Load Model Part1 Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model_part1);
		if (false == p_sensor_->LoadPLY(project_file + "\\" + ModelFileName + "_part_2.ply", object_model_part2))
		{
			LOG(ERROR) << "Load Model Part2 Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model_part2);
	}
  #pragma region ModelPose
	std::string JsonFileName = config;
	JsonOutType json_reader;
	float X, Y, Z, RX, RY, RZ;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "X", "float");
	if (json_reader.success)
		X = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Y", "float");
	if (json_reader.success)
		Y = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Z", "float");
	if (json_reader.success)
		Z = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "RX", "float");
	if (json_reader.success)
		RX = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "RY", "float");
	if (json_reader.success)
		RY = json_reader.json_float;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "RZ", "float");
	if (json_reader.success)
		RZ = json_reader.json_float;
	object_transform_init(0, 3) = X;
	object_transform_init(1, 3) = Y;
	object_transform_init(2, 3) = Z;
  #pragma endregion

	//model preprocessing
	pcl::transformPointCloud(*object_model, *object_model, object_transform_init.inverse()); //将目标点云 变换回到 源点云帧
	pcl::transformPointCloud(*object_model_part1, *object_model_part1, object_transform_init.inverse()); //将目标点云 变换回到 源点云帧
	pcl::transformPointCloud(*object_model_part2, *object_model_part2, object_transform_init.inverse()); //将目标点云 变换回到 源点云帧
	pcl::copyPointCloud(*object_model, *object_model_downsample);
	p_sensor_->DownSample(object_model_downsample, Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f));
	p_seg_eucli_->segment(object_model_downsample, nullptr, object_model_segeucli);
	p_sensor_->EdgeWithNormal(object_model_segeucli, normal_search_radius, curvature_thredhold, object_model_normal, object_model_edge);
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

	if (1 == view_point)
	{
		LOG(INFO) << "start algorithm at viewpoint 1";

		pcl::transformPointCloud(*object_scan, *object_scan, object_transform_init.inverse()); //将目标点云 变换回到 源点云帧
	  pcl::copyPointCloud(*object_scan, *object_scan_downsample);
	  p_sensor_->DownSample(object_scan_downsample, Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f));
		p_seg_eucli_->segment(object_scan_downsample, nullptr, object_scan_segeucli);
	  p_sensor_->EdgeWithNormal(object_scan_segeucli, normal_search_radius, curvature_thredhold, object_scene_normal, object_scene_edge);
		if (debug_visualization)
		{
			p_sensor_->ShowPointCloud(object_scene_edge, "object_scene_edge");
			p_sensor_->ShowPointCloud(object_model_edge, "object_model_edge");
		}
	
		p_registration_->SAC_IA(object_model_edge, object_scene_edge, sac_output, sac_transform, 0.004, debug_visualization);
		p_registration_->LM_ICP(object_scene_edge, sac_output, object_output, object_transform);
		object_transform = object_transform * sac_transform;
		if (part_refine)
		{
			//OBB Segmentation
			PointCloud::Ptr model_part1_transformed(new PointCloud());
			pcl::transformPointCloud(*object_model_part1, *model_part1_transformed, object_transform);
			p_seg_obb_->segment(object_scan, model_part1_transformed, obb_part1);
			PointCloud::Ptr model_part2_transformed(new PointCloud());
			pcl::transformPointCloud(*object_model_part2, *model_part2_transformed, object_transform);
			p_seg_obb_->segment(object_scan, model_part2_transformed, obb_part2);
			PointCloud::Ptr model_part_transformed(new PointCloud());
			*model_part_transformed = *model_part1_transformed + *model_part2_transformed;
			*obb_output = *obb_part1 + *obb_part2;
			//Boundary  
			PointCloud::Ptr model_part_boundary(new PointCloud());
			p_seg_bound_->segment(model_part_transformed, nullptr, model_part_boundary);
			PointCloud::Ptr obb_part_boundary(new PointCloud());
			p_seg_bound_->segment(obb_output, nullptr, obb_part_boundary);
			//LM-ICP Refine
			p_registration_refine_->LM_ICP(obb_part_boundary, model_part_boundary, object_output, object_transform_refine);
			object_transform = object_transform_refine * object_transform;
		}
		cout << object_transform << endl;
		object_transform = object_transform * object_transform_init;
		cout << object_transform << endl;

		p_sensor_->Matrix2EulerAngle(object_transform, object_eulerangle);

		{
			object_pose.clear();
			object_pose.push_back(object_transform(0, 3));
			object_pose.push_back(object_transform(1, 3));
			object_pose.push_back(object_transform(2, 3));
			object_pose.push_back(object_eulerangle[2] * 180 / M_PI);
			object_pose.push_back(object_eulerangle[1] * 180 / M_PI);
			object_pose.push_back(object_eulerangle[0] * 180 / M_PI);
		}
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

IPoseEstimation * GetInstance(char algothrim_version, std::string config_file)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(algothrim_version, config_file);
	return p_pose_estimation_;
}

