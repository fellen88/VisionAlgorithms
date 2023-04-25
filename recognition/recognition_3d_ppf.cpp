#include"stdafx.h"
#include "recognition_3d_ppf.h"

Recognition3DPPF::Recognition3DPPF()
{
	p_dataprocess_ = GetCameraData();

	
}

Recognition3DPPF::~Recognition3DPPF()
{
}

bool Recognition3DPPF::SetParameters(const std::string config)
{
	JsonOutType json_reader;
	json_reader = p_dataprocess_->ReadJsonFile(config, "Sample3D", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	subsampling_leaf_size = Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f);

	json_reader = p_dataprocess_->ReadJsonFile(config, "NormalSearchRadius", "float");
	if (json_reader.success)
		search_radius = json_reader.json_float;

	json_reader = p_dataprocess_->ReadJsonFile(config, "PositionClusteringThreshold", "float");
	if (json_reader.success)
		position_clustering_threshold = json_reader.json_float;

	json_reader = p_dataprocess_->ReadJsonFile(config, "RotationClusteringThreshold", "int");
	if (json_reader.success)
		rotation_Clustering_threshold = json_reader.json_int;

	json_reader = p_dataprocess_->ReadJsonFile(config, "PositionClusteringThreshold_Train", "float");
	if (json_reader.success)
		position_clustering_threshold_train = json_reader.json_float;

	json_reader = p_dataprocess_->ReadJsonFile(config, "RotationClusteringThreshold_Train", "int");
	if (json_reader.success)
		rotation_Clustering_threshold_train = json_reader.json_int;

	json_reader = p_dataprocess_->ReadJsonFile(config, "SceneReferencePointSamplingRate", "int");
	if (json_reader.success)
		point_sampling_rate = json_reader.json_int;

	json_reader = p_dataprocess_->ReadJsonFile(config, "PPF_Visualization", "bool");
	if (json_reader.success)
		ppf_visualization = json_reader.json_bool;

	return true;
}

bool Recognition3DPPF::Recognize(const PointCloud::Ptr cloud_scene,
	                             const std::vector<PointCloud::Ptr> cloud_models, 
																Eigen::Matrix4f& transformation, size_t& output_number)
{
	PointCloudWithNormals::Ptr cloud_scene_normals(new PointCloudWithNormals());
	PointCloud::Ptr cloud_scene_temp(new PointCloud());
	PointCloud::Ptr cloud_model_temp(new PointCloud());
	
	pcl::copyPointCloud(*cloud_scene, *cloud_scene_temp);
	pcl::copyPointCloud(*cloud_models[0], *cloud_model_temp);
	
	p_dataprocess_->DownSample(cloud_scene_temp, subsampling_leaf_size);
	p_dataprocess_->CalculateNormals(cloud_scene_temp, search_radius, cloud_scene_normals);

	PCL_INFO("Registering PPF models to scene ...\n");
	for (std::size_t model_i = 0; model_i < cloud_models.size(); ++model_i)
	{
		pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
		// set parameters for the PPF registration procedure
		ppf_registration.setSceneReferencePointSamplingRate(point_sampling_rate);
		ppf_registration.setPositionClusteringThreshold(position_clustering_threshold);
		ppf_registration.setRotationClusteringThreshold(rotation_Clustering_threshold / 180.0f * float(M_PI));
		ppf_registration.setSearchMethod(hashmap_search_vector[model_i]);
		ppf_registration.setInputSource(cloud_models_with_normals[model_i]);
		ppf_registration.setInputTarget(cloud_scene_normals);

		PointCloudWithNormals cloud_output_subsampled;
		ppf_registration.align(cloud_output_subsampled);

		PointCloud::Ptr cloud_output_subsampled_xyz(new PointCloud());
		for (const auto& point : cloud_output_subsampled.points)
			cloud_output_subsampled_xyz->points.emplace_back(point.x, point.y, point.z);

		Eigen::Matrix4f mat = ppf_registration.getFinalTransformation();
		Eigen::Affine3f final_transformation(mat);

		PointCloud::Ptr cloud_output(new PointCloud());
		pcl::transformPointCloud(
			*cloud_model_temp, *cloud_output, final_transformation);

		if (ppf_visualization)
		{
		  const std::string mode_name = "model_" + std::to_string(model_i);
			pcl::visualization::PCLVisualizer viewer("PPF Object Recognition - Results");
			viewer.setBackgroundColor(0, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<PointT> scene_color(cloud_scene, 0, 255, 0); //设置点云显示颜色
			pcl::visualization::PointCloudColorHandlerCustom<PointT> output_color(cloud_output, 255 ,0, 0); //设置点云显示颜色
			viewer.addPointCloud(cloud_scene, scene_color);
			viewer.addPointCloud(cloud_output, output_color, mode_name);
			viewer.spin();
			PCL_INFO("Showing model %s\n", mode_name.c_str());
		}
	}
	return true;
}

bool Recognition3DPPF::TrainModel(std::vector<PointCloud::Ptr> cloud_models)
{
	PCL_INFO("Training PPF Models ...\n");

	for (const auto& cloud_model : cloud_models)
	{
		PointCloudWithNormals::Ptr cloud_model_normals(new PointCloudWithNormals());
		PointCloud::Ptr cloud_model_temp(new PointCloud());
	  pcl::copyPointCloud(*cloud_model, *cloud_model_temp);
		p_dataprocess_->DownSample(cloud_model_temp, subsampling_leaf_size);
		p_dataprocess_->CalculateNormals(cloud_model_temp, search_radius, cloud_model_normals);
		cloud_models_with_normals.push_back(cloud_model_normals);

		pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf(new pcl::PointCloud<pcl::PPFSignature>());
		pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
		ppf_estimator.setInputCloud(cloud_model_normals);
		ppf_estimator.setInputNormals(cloud_model_normals);
		ppf_estimator.compute(*cloud_model_ppf);

		pcl::PPFHashMapSearch::Ptr hashmap_search(
			new pcl::PPFHashMapSearch(rotation_Clustering_threshold_train / 180.0f * float(M_PI), position_clustering_threshold_train));
		hashmap_search->setInputFeatureCloud(cloud_model_ppf);
		hashmap_search_vector.push_back(hashmap_search);
	}
	return true;
}

IRecognition* GetRecognition3DPPF()
{
	IRecognition* p_irecognition = new Recognition3DPPF();
	return p_irecognition;
} 