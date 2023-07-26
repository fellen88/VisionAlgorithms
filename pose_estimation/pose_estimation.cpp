#include "stdafx.h"

#include "pose_estimation.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace gpd;

PoseEstimation::PoseEstimation(std::string config_file) :
	object_model(new pcl::PointCloud<pcl::PointXYZ>),
	model_refine_a(new pcl::PointCloud<pcl::PointXYZ>),
	model_refine_b(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_transformed(new pcl::PointCloud<pcl::PointXYZ>),
	cloud_scene(new pcl::PointCloud<pcl::PointXYZ>),
	sac_output(new pcl::PointCloud<pcl::PointXYZ>),
	icp_output(new pcl::PointCloud<pcl::PointXYZ>),
	cloud_object_instance(new pcl::PointCloud<pcl::PointXYZ>),
	object_model_instance(new pcl::PointCloud<pcl::PointXYZ>),
	model_refine_a_transformed(new pcl::PointCloud<pcl::PointXYZ>),
	model_refine_b_transformed(new pcl::PointCloud<pcl::PointXYZ>),
	obb_a(new pcl::PointCloud<pcl::PointXYZ>),
	obb_b(new pcl::PointCloud<pcl::PointXYZ>),
	boundary_obb_a(new pcl::PointCloud<pcl::PointXYZ>),
	boundary_obb_b(new pcl::PointCloud<pcl::PointXYZ>),
	boundary_refine_a(new pcl::PointCloud<pcl::PointXYZ>),
	boundary_refine_b(new pcl::PointCloud<pcl::PointXYZ>),
	euclidean_obb_a(new pcl::PointCloud<pcl::PointXYZ>),
	euclidean_obb_b(new pcl::PointCloud<pcl::PointXYZ>),
	euclidean_refine_a(new pcl::PointCloud<pcl::PointXYZ>),
	euclidean_refine_b(new pcl::PointCloud<pcl::PointXYZ>),
	euclidean_obb(new pcl::PointCloud<pcl::PointXYZ>),
	euclidean_refine(new pcl::PointCloud<pcl::PointXYZ>),
	object_instance_number(0),
	grasp_count(0),
	sac_transform(Eigen::Matrix4f::Identity()),
	object_transform(Eigen::Matrix4f::Identity()),
	object_transform_init(Eigen::Matrix4f::Identity()),
	object_transform_refine(Eigen::Matrix4f::Identity()),
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
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "CameraOffline", "bool");
	if (json_reader.success)
		sensor_offline = json_reader.json_bool;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "InstanceMethod", "string");
	if (json_reader.success)
		instance_method = json_reader.json_string;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "RefineModelNum", "int");
	if (json_reader.success)
		refine_model_num = json_reader.json_int;
	json_reader = p_sensor_->ReadJsonFile(JsonFileName, "RefineRegistrarion", "string");
	if (json_reader.success)
		refine_registration = json_reader.json_string;
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

	//set parameters
	UpdateParameters(config);

	instance_cg_config = config_path + "instance_cg.json";
	refine_seg_obb_config = config_path + "refine_seg_obb.json";
	refine_seg_euclidean_config = config_path + "refine_seg_euclidean.json";
	refine_seg_boundary_config = config_path + "refine_seg_boundary.json";
	refine_regist_sacia_config = config_path + "refine_regist_sacia.json";
	refine_regist_lmicp_config = config_path + "refine_regist_lmicp.json";
	
	//load ply model
	if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" + ModelFileName, object_model))
	{
		LOG(ERROR) << "Load " + project_file + "\\" + object_file + "\\" + ModelFileName + " error!";
	}
	else
	{
		LOG(INFO) << "Load " + project_file + "\\" + object_file + "\\" + ModelFileName + " success!";
		p_sensor_->ConvertPointsMMtoM(object_model);
	}
	if (refine_model_num > 0)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" +
			ModelFileName.erase(ModelFileName.find("."), 4) + "_refine_a.ply", model_refine_a))
		{
			LOG(ERROR) << "Load " + project_file + "\\" + object_file + "\\" + ModelFileName.erase(ModelFileName.find("."), 4) + "_refine_a.ply" + " error!";
		}
		else
		{
			LOG(INFO) << "Load " + project_file + "\\" + object_file + "\\" + ModelFileName + "_refine_a.ply" + " success!";
			p_sensor_->ConvertPointsMMtoM(model_refine_a);
		}
	}
	if (refine_model_num > 1)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" +
			ModelFileName + "_refine_b.ply", model_refine_b))
		{
			LOG(ERROR) << "Load " + project_file + "\\" + ModelFileName + "_refine_b.ply" + " error!";
		}
		else
		{
			LOG(INFO) << "Load " + project_file + "\\" + ModelFileName + "_refine_b.ply" + " success!";
			p_sensor_->ConvertPointsMMtoM(model_refine_b);
		}
	}

	if (use_model_pose)
	{
		object_transform_init(0, 3) = X;
		object_transform_init(1, 3) = Y;
		object_transform_init(2, 3) = Z;
		// 初始化欧拉角（rpy）,对应绕x轴，绕y轴，绕z轴的旋转角度
		Eigen::Vector3f euler_angle(RZ, RY, RX);
		EulerAngle2Matrix(euler_angle, object_transform_init);	
		//model pose transformation
		pcl::transformPointCloud(*object_model, *object_model_transformed, object_transform_init);
		cloud_models.push_back(object_model_transformed);
	}
	else
	{
		cloud_models.push_back(object_model);
	}

	//recognition
	if ("CG" == instance_method)
	{
		LOG(INFO) << "---------> instance cg parameters:";
		p_recog_cg_->SetParameters(instance_cg_config);
		//p_recog_ppf_->TrainModel(cloud_models);
		pcl::copyPointCloud(*object_model, *object_model_instance);
	}
	else if ("Mask" == instance_method)
	{
		LOG(INFO) << "---------> instance mask parameters:";
		p_sensor_->SetParameters(config);
		pcl::copyPointCloud(*object_model, *object_model_instance);
	}
	else
	{
		pcl::copyPointCloud(*object_model, *object_model_instance);
	}
	
	//refine
	if (refine_model_num > 0)
	{
		LOG(INFO) << "---------> seg_obb parameters:";
		p_refine_seg_obb_->SetParameters(refine_seg_obb_config);
		LOG(INFO) << "---------> seg_euclidean parameters:";
		p_refine_seg_eucli_->SetParameters(refine_seg_euclidean_config);
		LOG(INFO) << "---------> seg_boundary parameters:";
		p_refine_seg_bound_->SetParameters(refine_seg_boundary_config);
		LOG(INFO) << "---------> sac_ia parameters:";
		p_refine_regist_sacia_->SetParameters(refine_regist_sacia_config);
		LOG(INFO) << "---------> lm_icp  parameters:";
		p_refine_regist_lmicp_->SetParameters(refine_regist_lmicp_config);
	}
}

bool PoseEstimation::Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f& object_pose)
{
	LOG(INFO) << "******** grasp count: " << ++grasp_count << " ********";
	//sensor offine/online mode
	if (sensor_offline)
	{
		if (false == p_sensor_->LoadPLY(project_file + "\\" + object_file + "\\" + ScanFileName, cloud_scene))
		{
			LOG(ERROR) << "LoadPointCloud Error!";
			return false;
		}
		else
			LOG(INFO) << "Load PLY on sensor off mode";
		p_sensor_->ConvertPointsMMtoM(cloud_scene);
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
			pcl::copyPointCloud(object_points, *cloud_scene);
			p_sensor_->ConvertPointsMMtoM(cloud_scene);
		}
	}

	//object instance
	if ("CG" == instance_method)
	{
		p_recog_cg_->Recognize(cloud_scene, cloud_models, object_transform, object_instance_number);
		if (object_instance_number > 0)
		{
			LOG(INFO) << "transformation matrix after cg: \n " << object_transform;
			cout << endl << object_transform << endl;
			pcl::copyPointCloud(*cloud_scene, *cloud_object_instance);
			//PointCloud::Ptr model_instance_transformed(new PointCloud());
			//pcl::transformPointCloud(*object_model_instance, *model_instance_transformed, object_transform);
			//p_seg_obb_instance_->Segment(scene_transformed, model_instance_transformed, object_scan_instance);
		}
		else
		{
			object_transform = Eigen::Matrix4f::Identity();
		}
	}
	else if ("Mask" == instance_method)
	{
		cv::imshow("output_mask", object_mask);
		cv::waitKey(0);
		object_instance_number = 1;
		p_sensor_->DepthtoPointCloud(object_depth, object_mask, cloud_object_instance);
		p_sensor_->ShowPointCloud(cloud_object_instance, "object_instance");
	}
	else
		pcl::copyPointCloud(*cloud_scene, *cloud_object_instance);

	//use model pose
	if (use_model_pose && object_instance_number > 0)
	{
		object_transform = object_transform * object_transform_init;
	}

	//registration refine
	if (refine_model_num > 0 && object_instance_number > 0)
	{
		//OBB Segmentation
		pcl::transformPointCloud(*model_refine_a, *model_refine_a_transformed, object_transform);
		p_refine_seg_obb_->Segment(cloud_object_instance, model_refine_a_transformed, obb_a);
		if (refine_model_num > 1)
		{
			pcl::transformPointCloud(*model_refine_b, *model_refine_b_transformed, object_transform);
			p_refine_seg_obb_->Segment(cloud_object_instance, model_refine_b_transformed, obb_b);
		}
		//Boundary  
		p_refine_seg_bound_->Segment(obb_a, nullptr, boundary_obb_a);
		p_refine_seg_bound_->Segment(model_refine_a_transformed, nullptr, boundary_refine_a);
		if (refine_model_num > 1)
		{
			p_refine_seg_bound_->Segment(obb_b, nullptr, boundary_obb_b);
			p_refine_seg_bound_->Segment(model_refine_b_transformed, nullptr, boundary_refine_b);
		}
		//Euclidean
		p_refine_seg_eucli_->Segment(boundary_obb_a, nullptr, euclidean_obb_a);
		p_refine_seg_eucli_->Segment(boundary_refine_a, nullptr, euclidean_refine_a);
		if (refine_model_num > 1)
		{
			p_refine_seg_eucli_->Segment(boundary_obb_b, nullptr, euclidean_obb_b);
			p_refine_seg_eucli_->Segment(boundary_refine_b, nullptr, euclidean_refine_b);
		}
		*euclidean_obb = *euclidean_obb_a + *euclidean_obb_b;
		*euclidean_refine = *euclidean_refine_a + *euclidean_refine_b;
		//refine_registration
		if ("IA_ICP" == refine_registration)
		{
			p_refine_regist_sacia_->Align(euclidean_refine, euclidean_obb, sac_output, sac_transform);
			p_refine_regist_lmicp_->Align(sac_output, euclidean_obb, icp_output, object_transform_refine);
			object_transform = object_transform_refine * sac_transform * object_transform;
		}
		else if("ICP" == refine_registration)
		{
			p_refine_regist_lmicp_->Align(euclidean_obb, euclidean_refine, icp_output, object_transform_refine);
			object_transform = object_transform_refine * object_transform;
		}
		//output
		LOG(INFO) << "transformation matrix after refine: \n " << object_transform;
		cout << endl << object_transform << endl;
	}

	//output
	object_pose = object_transform;


	//visulization
	//TODO: thread 
	if (debug_visualization)
	{
		//p_sensor_->ShowPose(object_scan, object_transform, "pose");
		boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer("pose"));
		pcl_viewer->setCameraPosition(-0.3, 0, -0.3, 0, 0, 1, -1, 0, 0); //视点 方向 上方向
		//pcl_viewer->addCoordinateSystem(0.1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler_refine(euclidean_refine, 0, 255, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(object_model, 0, 255, 0);
		if (nullptr != cloud_scene)
		{
			pcl_viewer->addPointCloud(cloud_scene);
			if (refine_model_num > 0)
			{
				pcl_viewer->addPointCloud(euclidean_refine, color_handler_refine, "object");
				pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object");
			}
			else
			{
				pcl::transformPointCloud(*object_model, *object_model_transformed, object_transform);
				pcl_viewer->addPointCloud(object_model_transformed, color_handler, "object");
				pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "object");
			}
		}
		PointCloud::Ptr axis(new PointCloud);
		PointCloud::Ptr pose(new PointCloud);
		axis->points.push_back(pcl::PointXYZ(0, 0, 0));
		axis->points.push_back(pcl::PointXYZ(0.1, 0, 0));
		axis->points.push_back(pcl::PointXYZ(0, 0.1, 0));
		axis->points.push_back(pcl::PointXYZ(0, 0, 0.1));
		pcl::transformPointCloud(*axis, *pose, object_transform);

		pcl_viewer->addLine(pose->points[0], pose->points[1], 1.0f, 0.0f, 0.0f, "x vector");
		pcl_viewer->addLine(pose->points[0], pose->points[2], 0.0f, 1.0f, 0.0f, "y vector");
		pcl_viewer->addLine(pose->points[0], pose->points[3], 0.0f, 0.0f, 1.0f, "z vector");
		pcl_viewer->spin();
	}
	return true;
}

IPoseEstimation * GetInstance(std::string config_file)
{
	IPoseEstimation* p_pose_estimation_ = new PoseEstimation(config_file);
	return p_pose_estimation_;
}

