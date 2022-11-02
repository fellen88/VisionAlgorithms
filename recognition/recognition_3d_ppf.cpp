#include"stdafx.h"
#include "recognition_3d_ppf.h"

Recognition3DPPF::Recognition3DPPF()
{
	p_dataprocess_ = GetCameraData();

}

Recognition3DPPF::~Recognition3DPPF()
{
}

bool Recognition3DPPF::Compute(int a)
{
	//PointCloud<PointNormal>::Ptr cloud_scene_input =
	//	subsampleAndCalculateNormals(cloud_scene);
	//std::vector<PointCloud<PointNormal>::Ptr> cloud_models_with_normals;

	//PCL_INFO("Training models ...\n");
	//std::vector<PPFHashMapSearch::Ptr> hashmap_search_vector;
	//for (const auto& cloud_model : cloud_models) {
	//	PointCloud<PointNormal>::Ptr cloud_model_input =
	//		subsampleAndCalculateNormals(cloud_model);
	//	cloud_models_with_normals.push_back(cloud_model_input);

	//	PointCloud<PPFSignature>::Ptr cloud_model_ppf(new PointCloud<PPFSignature>());
	//	PPFEstimation<PointNormal, PointNormal, PPFSignature> ppf_estimator;
	//	ppf_estimator.setInputCloud(cloud_model_input);
	//	ppf_estimator.setInputNormals(cloud_model_input);
	//	ppf_estimator.compute(*cloud_model_ppf);

	//	PPFHashMapSearch::Ptr hashmap_search(
	//		new PPFHashMapSearch(5.0f / 180.0f * float(M_PI), 0.001f));
	//	hashmap_search->setInputFeatureCloud(cloud_model_ppf);
	//	hashmap_search_vector.push_back(hashmap_search);
	//}

	//visualization::PCLVisualizer viewer("PPF Object Recognition - Results");
	//viewer.setBackgroundColor(0, 0, 0);
	//viewer.addPointCloud(cloud_scene);
	//viewer.spinOnce(1000);
	//PCL_INFO("Registering models to scene ...\n");
	//for (std::size_t model_i = 0; model_i < cloud_models.size(); ++model_i)
	//{

	//	PPFRegistration<PointNormal, PointNormal> ppf_registration;
	//	// set parameters for the PPF registration procedure
	//	ppf_registration.setSceneReferencePointSamplingRate(10);
	//	ppf_registration.setPositionClusteringThreshold(0.001f);
	//	ppf_registration.setRotationClusteringThreshold(5.0f / 180.0f * float(M_PI));
	//	ppf_registration.setSearchMethod(hashmap_search_vector[model_i]);
	//	ppf_registration.setInputSource(cloud_models_with_normals[model_i]);
	//	ppf_registration.setInputTarget(cloud_scene_input);

	//	PointCloud<PointNormal> cloud_output_subsampled;
	//	ppf_registration.align(cloud_output_subsampled);

	//	PointCloud<PointXYZ>::Ptr cloud_output_subsampled_xyz(new PointCloud<PointXYZ>());
	//	for (const auto& point : cloud_output_subsampled.points)
	//		cloud_output_subsampled_xyz->points.emplace_back(point.x, point.y, point.z);

	//	Eigen::Matrix4f mat = ppf_registration.getFinalTransformation();
	//	Eigen::Affine3f final_transformation(mat);

	//	PointCloud<PointXYZ>::Ptr cloud_output(new PointCloud<PointXYZ>());
	//	pcl::transformPointCloud(
	//		*cloud_models[model_i], *cloud_output, final_transformation);

	//	const std::string mode_name = "model_" + std::to_string(model_i);
	//	visualization::PointCloudColorHandlerRandom<PointXYZ> random_color(cloud_output->makeShared());
	//	viewer.addPointCloud(cloud_output, random_color, mode_name);
	//	viewer.spin();
	//	//PCL_INFO("Showing model %s\n", mode_name.c_str());
	//}
	return false;
}

IRecognition* GetRecognition3DPPF()
{
	IRecognition* p_irecognition = new Recognition3DPPF();
	return p_irecognition;
} 