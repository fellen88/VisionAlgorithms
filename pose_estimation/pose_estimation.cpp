#include "stdafx.h"

#include "pose_estimation.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace gpd;

PoseEstimation::PoseEstimation(std::string config_file) :
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
	p_sensor_(GetCameraData())
{
	Init_Compute(config_file);
}

PoseEstimation::~PoseEstimation()
{
}

void PoseEstimation::EulerAngle2Matrix(Eigen::Vector3f & euler_angle, Eigen::Matrix4f & transformation_matrix)
{
	euler_angle = euler_angle * M_PI / 180;
	Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
	Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
	Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));

	Eigen::Matrix3d rotation_matrix;
	rotation_matrix = yawAngle * pitchAngle*rollAngle;

	for(size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
		{
			transformation_matrix(i, j) = rotation_matrix(i, j);
		}
}

void PoseEstimation::UpdateParameters(std::string config)
{
	std::string JsonFileName = config;
	JsonOutType json_reader;
	p_sensor_->GetSubPath(config, project_file, 4);
	p_sensor_->GetSubPath(config, object_file, 3);
	LOG(INFO) << "********************************************";
	LOG(INFO) << "project name: " << project_file;
	LOG(INFO) << "object class: " << object_file;
	LOG(INFO) << "********************************************";
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Visualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "SensorOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "InstanceSeg", "string");
	if (json_reader.success)
		instance_seg = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "InstanceKeyPoint", "string");
	if (json_reader.success)
		instance_keypoint = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "Registrarion", "string");
	if (json_reader.success)
		registration = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "RefineModelNum", "int");
	if (json_reader.success)
		refine_model_num = json_reader.json_int;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "ObjectModelPath", "string");
	if (json_reader.success)
		ModelFileName = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "PointCloudPath", "string");
	if (json_reader.success)
		ScanFileName = json_reader.json_string;
	
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "UseModelPose", "bool");
	if (json_reader.success)
		use_model_pose = json_reader.json_bool;
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

	p_sensor_->GetSubPath(config, config_file, 1);
	config_path = config.erase(config.find(config_file), config_file.size());
}

void PoseEstimation::Init_Compute(std::string config)
{
	//reset shared_ptr
	p_refine_seg_obb_.reset(GetSegmentationOBB());
	p_seg_obb_instance_.reset(GetSegmentationOBB());
	p_refine_seg_bound_.reset(GetSegmentationBoundary());
	p_instance_seg_bound_.reset(GetSegmentationBoundary());
	p_seg_eucli_.reset(GetSegmentationEuclidean());
	p_refine_seg_eucli_.reset(GetSegmentationEuclidean());
	p_recog_ppf_.reset(GetRecognition3DPPF());
	p_recog_cg_.reset(GetRecognition3DCG());
	p_regist_lmicp_.reset(GetRegistrationLMICP());
	p_refine_regist_lmicp_.reset(GetRegistrationLMICP());
	p_regist_sacia_.reset(GetRegistrationSACIA());
	p_refine_regist_sacia_.reset(GetRegistrationSACIA());
	p_recog_cg_.reset(GetRecognition3DCG());


	//init variable
	sac_transform = Eigen::Matrix4f::Identity();
	object_transform = Eigen::Matrix4f::Identity();
	object_transform_init = Eigen::Matrix4f::Identity();
	object_transform_refine = Eigen::Matrix4f::Identity();
	object_instance_number = 0;

	//set parameters
	UpdateParameters(config);

	instance_recog_ppf_config = config_path + "instance_recog_ppf.json";
	instance_seg_obb_config = config_path + "instance_seg_obb.json";
	keypoint_seg_boundary_config = config_path + "keypoint_seg_boundary.json";
	refine_regist_lmicp_config = config_path + "refine_regist_lmicp.json";
	refine_regist_sacia_config = config_path + "refine_regist_sacia.json";
	refine_seg_boundary_config = config_path + "refine_seg_boundary.json";
	refine_seg_euclidean_config = config_path + "refine_seg_euclidean.json";
	refine_seg_obb_config = config_path + "refine_seg_obb.json";
	regist_lmicp_config = config_path + "regist_lmicp.json";
	regist_sacia_config = config_path + "regist_sacia.json";
	seg_euclidean_config = config_path + "seg_euclidean.json";
	seg_sac_config = config_path + "seg_sac.json";
	recog_cg_config = config_path + "recognition_cg.json";

	p_regist_lmicp_->SetParameters(regist_lmicp_config);
	p_regist_sacia_->SetParameters(regist_sacia_config);

	if (refine_model_num > 0)
	{
		p_refine_seg_obb_->SetParameters(refine_seg_obb_config);
		p_refine_seg_bound_->SetParameters(refine_seg_boundary_config);
		p_refine_seg_eucli_->SetParameters(refine_seg_euclidean_config);
		p_refine_regist_lmicp_->SetParameters(refine_regist_lmicp_config);
		//p_refine_regist_sacia_->SetParameters(refine_regist_sacia_config);
	}

	//load ply model
	if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" + ModelFileName, object_model))
	{
		LOG(ERROR) << "LoadModel Error!";
	}
	else
		p_sensor_->ConvertPointsMMtoM(object_model);
	if (refine_model_num > 0)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\"+ object_file+"\\" + ModelFileName.erase(ModelFileName.find("."), 4) + "_refine_a.ply", object_model_part1))
		{
			LOG(ERROR) << "Load" + project_file + "\\" + ModelFileName.erase(ModelFileName.find("."), 4) + "_refine_a.ply" + " Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model_part1);
	}
	if (refine_model_num > 1)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" + ModelFileName + "_refine_b.ply", object_model_part2))
		{
			LOG(ERROR) << "Load" + project_file + "\\" + ModelFileName + "_refine_b.ply" + "Error!";
		}
		else
			p_sensor_->ConvertPointsMMtoM(object_model_part2);
	}

	if (use_model_pose)
	{
		object_transform_init(0, 3) = X;
		object_transform_init(1, 3) = Y;
		object_transform_init(2, 3) = Z;
		// 初始化欧拉角（rpy）,对应绕x轴，绕y轴，绕z轴的旋转角度
		Eigen::Vector3f euler_angle(RX, RY, RZ);
		EulerAngle2Matrix(euler_angle, object_transform_init);
	}

	//instance
	if ("Euclidean" == instance_seg)
	{
		p_seg_eucli_->SetParameters(seg_euclidean_config);
		p_seg_eucli_->Segment(object_model, nullptr, object_model_segeucli);
		pcl::copyPointCloud(*object_model_segeucli, *object_model_instance);
	}
	//else if ("PPF_OBB" == instance_seg)
	//{
	//	p_recog_ppf_->SetParameters(instance_recog_ppf_config);
	//	p_seg_obb_instance_->SetParameters(instance_seg_obb_config);
	//	cloud_models.push_back(object_model);
	//	p_recog_ppf_->TrainModel(cloud_models);
	//}
	else if ("CG_OBB" == instance_seg)
	{
		p_recog_cg_->SetParameters(recog_cg_config);
		//p_seg_obb_instance_->SetParameters(instance_seg_obb_config);
		cloud_models.push_back(object_model);
	  //p_recog_ppf_->TrainModel(cloud_models);
		pcl::copyPointCloud(*object_model, *object_model_instance);
	}
	else
	{
		pcl::copyPointCloud(*object_model, *object_model_instance);
	}
	//edge normal
	if ("Boundary" == instance_keypoint)
	{
		p_instance_seg_bound_->SetParameters(keypoint_seg_boundary_config);
		p_instance_seg_bound_->Segment(object_model_instance, nullptr, object_model_edge);
		pcl::copyPointCloud(*object_model_edge, *object_model_preprocess);
	}
	else
		pcl::copyPointCloud(*object_model_instance, *object_model_preprocess);
}

bool PoseEstimation::Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f& object_pose)
{
	//sensor offine/online mode
	if (sensor_offline)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" + ScanFileName, object_scan))
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

	//downsampling and transformation
	if (use_model_pose)
	{
		pcl::transformPointCloud(*object_scan, *object_scan, object_transform_init.inverse());
	}

	//object instance
	if ("Euclidean" == instance_seg)
	{
		p_seg_eucli_->Segment(object_scan, nullptr, object_scan_segeucli);
		pcl::copyPointCloud(*object_scan_segeucli, *object_scan_instance);
	}
	//else if ("PPF_OBB" == instance_seg)
	//{
	//	p_recog_ppf_->Recognize(object_scan, cloud_models, object_transform);
	//	PointCloud::Ptr model_instance_transformed(new PointCloud());
	//	pcl::transformPointCloud(*object_model_instance, *model_instance_transformed, object_transform);
	//	p_seg_obb_instance_->Segment(object_scan, model_instance_transformed, object_scan_instance);
	//}
	else if ("CG_OBB" == instance_seg)
	{
		p_recog_cg_->Recognize(object_scan, cloud_models, object_transform, object_instance_number);
		if (object_instance_number > 0)
		{
			PointCloud::Ptr model_instance_transformed(new PointCloud());
			pcl::transformPointCloud(*object_model_instance, *model_instance_transformed, object_transform);
			LOG(INFO) << "transformation matrix after cg: \n " << object_transform;
			cout << endl << object_transform << endl;
			//p_seg_obb_instance_->Segment(object_scan, model_instance_transformed, object_scan_instance);
		}
		else
			return false;
	}
	else
		pcl::copyPointCloud(*object_scan, *object_scan_instance);

	//object edge 
	if ("Boundary" == instance_keypoint)
	{
		p_instance_seg_bound_->Segment(object_scan_instance, nullptr, object_scene_edge);
		pcl::copyPointCloud(*object_scene_edge, *object_scan_preprocess);
	}
	else
		pcl::copyPointCloud(*object_scan_instance, *object_scan_preprocess);

	//TODO:visulation
	if (debug_visualization)
	{
		p_sensor_->ShowPointCloud(object_scene_edge, "object_scene_edge");
		p_sensor_->ShowPointCloud(object_model_edge, "object_model_edge");
	}

	//registration
	if ("IA_ICP" == registration)
	{
		p_regist_sacia_->Align(object_model_preprocess, object_scan_preprocess, sac_output, sac_transform);
		p_regist_lmicp_->Align(object_scan_preprocess, sac_output, object_output, object_transform);
		object_transform = object_transform * sac_transform;
		LOG(INFO) << "transformation matrix after registration: \n " << object_transform;
		cout << endl << object_transform << endl;
	}

	//registration refine
	if (refine_model_num > 0)
	{
		PointCloud::Ptr model_part1_transformed(new PointCloud());
		PointCloud::Ptr model_part2_transformed(new PointCloud());
		PointCloud::Ptr model_part_transformed(new PointCloud());
		PointCloud::Ptr euclidean_part1(new PointCloud());
		PointCloud::Ptr euclidean_part2(new PointCloud());
		PointCloud::Ptr euclidean_part(new PointCloud());
		PointCloud::Ptr euclidean_model1(new PointCloud());
		PointCloud::Ptr euclidean_model2(new PointCloud());
		PointCloud::Ptr euclidean_model(new PointCloud());
		PointCloud::Ptr model_part_boundary(new PointCloud());
		PointCloud::Ptr obb_part_boundary(new PointCloud());
		//OBB Segmentation
		pcl::transformPointCloud(*object_model_part1, *model_part1_transformed, object_transform);
		p_refine_seg_obb_->Segment(object_scan, model_part1_transformed, obb_part1);
		if (refine_model_num > 1)
		{
			//OBB Segmentation
			pcl::transformPointCloud(*object_model_part2, *model_part2_transformed, object_transform);
			p_refine_seg_obb_->Segment(object_scan, model_part2_transformed, obb_part2);
		}
		*model_part_transformed = *model_part1_transformed + *model_part2_transformed;
		*obb_output = *obb_part1 + *obb_part2;
		//Euclidean
		p_refine_seg_eucli_->Segment(model_part1_transformed, nullptr, euclidean_model1);
		p_refine_seg_eucli_->Segment(obb_part1, nullptr, euclidean_part1);
		if (refine_model_num > 1)
		{
			p_refine_seg_eucli_->Segment(model_part2_transformed, nullptr, euclidean_model2);
			p_refine_seg_eucli_->Segment(obb_part2, nullptr, euclidean_part2);
		}
		*euclidean_model = *euclidean_model1 + *euclidean_model2;
		*euclidean_part = *euclidean_part1 + *euclidean_part2;
		//Boundary  
		p_refine_seg_bound_->Segment(euclidean_model, nullptr, model_part_boundary);
		p_refine_seg_bound_->Segment(euclidean_part, nullptr, obb_part_boundary);
		//LM-ICP Refine
			//p_refine_regist_sacia_->Align(model_part_boundary, obb_part_boundary, sac_output, sac_transform);
		p_refine_regist_lmicp_->Align(obb_part_boundary, model_part_boundary, object_output, object_transform_refine);
		object_transform = object_transform_refine * object_transform;
		//output
		LOG(INFO) << "transformation matrix after refine: \n " << object_transform;
		cout << endl << object_transform << endl;
	}
	if (use_model_pose)
	{
		object_transform = object_transform_init * object_transform;
		LOG(INFO) << "transformation matrix after model pose: \n " << object_transform;
		cout << endl <<object_transform << endl;
		object_pose = object_transform;
	}

	return true;
}

IPoseEstimation * GetInstance(std::string config_file)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(config_file);
	return p_pose_estimation_;
}

