/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "stdafx.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h> 
#include <pcl/console/parse.h>
#include <pcl/keypoints/harris_3d.h>

#include "recognition_3d_cg.h"

using namespace gpd;

Recognition3DCG::Recognition3DCG():
	key_points_("uniform"),
	p_dataprocess_ (GetCameraData())
{
}

Recognition3DCG::~Recognition3DCG()
{
}

bool gpd::Recognition3DCG::SetParameters(const std::string config_file)
{
	//Visualization 
	JsonOutType json_reader;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "show_result", "bool");
	if (json_reader.success)
		show_result_ = json_reader.json_bool;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "show_keypoints", "bool");
	if (json_reader.success)
		show_keypoints_ = json_reader.json_bool;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "show_correspondences", "bool");
	if (json_reader.success)
		show_correspondences_ = json_reader.json_bool;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "show_rotated_model", "bool");
	if (json_reader.success)
		show_rotated_model_ = json_reader.json_bool;
	//Recognition
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "sample_3d", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	subsampling_leaf_size = Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f);
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "key_points", "string");
	if (json_reader.success)
		key_points_ = json_reader.json_string;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "model_ss", "float");
	if (json_reader.success)
		model_ss_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "scene_ss", "float");
	if (json_reader.success)
		scene_ss_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "use_hough", "bool");
	if (json_reader.success)
		use_hough_ = json_reader.json_bool;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "rf_rad", "float");
	if (json_reader.success)
		rf_rad_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "descr_rad", "float");
	if (json_reader.success)
		descr_rad_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "cg_size", "float");
	if (json_reader.success)
		cg_size_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "cg_thresh", "int");
	if (json_reader.success)
		cg_thresh_ = json_reader.json_int;
	//ICP
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "use_icp", "bool");
	if (json_reader.success)
		use_icp_ = json_reader.json_bool;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "icp_max_iter", "float");
	if (json_reader.success)
		icp_max_iter_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "icp_corr_distance", "float");
	if (json_reader.success)
		icp_corr_distance_ = json_reader.json_float;
	//Hypothesis Verification
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "use_hv", "bool");
	if (json_reader.success)
		use_hv_ = json_reader.json_bool;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_resolution", "float");
	if (json_reader.success)
		hv_resolution_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_occupancy_grid_resolution", "float");
	if (json_reader.success)
		hv_occupancy_grid_resolution_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_clutter_reg", "float");
	if (json_reader.success)
		hv_clutter_reg_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_inlier_th", "float");
	if (json_reader.success)
		hv_inlier_th_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_occlusion_th", "float");
	if (json_reader.success)
		hv_occlusion_th_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_rad_clutter", "float");
	if (json_reader.success)
		hv_rad_clutter_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_regularizer", "float");
	if (json_reader.success)
		hv_regularizer_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_rad_normals", "float");
	if (json_reader.success)
		hv_rad_normals_ = json_reader.json_float;
	json_reader = p_dataprocess_->ReadJsonFile(config_file, "hv_detect_clutter", "float");
	if (json_reader.success)
		hv_detect_clutter_ = json_reader.json_float;

	return false;
}

bool gpd::Recognition3DCG::Recognize(const PointCloud::Ptr cloud_scene, const std::vector<PointCloud::Ptr> cloud_model,
	Eigen::Matrix4f& output_transformation, size_t& output_number)
{
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_uniform_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_uniform_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr scene_harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr model_harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>);
	/**
	 * Downsample Clouds
	 */
	PointCloud::Ptr cloud_scene_temp(new PointCloud());
	PointCloud::Ptr cloud_model_temp(new PointCloud());
	pcl::copyPointCloud(*cloud_scene, *cloud_scene_temp);
	pcl::copyPointCloud(*cloud_model[0], *cloud_model_temp);
	
	p_dataprocess_->UniformSampling(cloud_scene_temp, sample_3d, cloud_scene_temp);
	p_dataprocess_->UniformSampling(cloud_model_temp, sample_3d, cloud_model_temp);
	pcl::copyPointCloud(*cloud_scene_temp, *scene);
	pcl::copyPointCloud(*cloud_model_temp, *model);

	/**
	 * Compute Normals
	 */
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);

	/**
	 * Extract keypoints
	 */

	if ("harris" == key_points_)
	{
		/* 声明Harris 关键点 检测 对象  */
		pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI, pcl::Normal>* harris_detector = new pcl::HarrisKeypoint3D<PointType, pcl::PointXYZI, pcl::Normal>;
		const float r_normal = 0.01;
		const float r_keypoint = 0.01;
		/*设置相关参数*/
		harris_detector->setRadius(r_normal);
		harris_detector->setRadiusSearch(r_keypoint);
		harris_detector->setNonMaxSupression(true);
		/*计算关键点*/
		harris_detector->setInputCloud(scene);
		harris_detector->compute(*scene_harris_keypoints);
		std::cout << "scene_harris_keypoints number:" << scene_harris_keypoints->size() << std::endl;
		harris_detector->setInputCloud(model);
		harris_detector->compute(*model_harris_keypoints);
		std::cout << "model_harris_keypoints number:" << model_harris_keypoints->size() << std::endl;

		pcl::copyPointCloud(*scene_harris_keypoints, *scene_keypoints);
		pcl::copyPointCloud(*model_harris_keypoints, *model_keypoints);

	}
	else if ("uniform" == key_points_)
	{
		pcl::UniformSampling<PointType> uniform_sampling;
		uniform_sampling.setInputCloud(model);
		uniform_sampling.setRadiusSearch(model_ss_);
		uniform_sampling.filter(*model_uniform_keypoints);
		std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_uniform_keypoints->size() << std::endl;

		uniform_sampling.setInputCloud(scene);
		uniform_sampling.setRadiusSearch(scene_ss_);
		uniform_sampling.filter(*scene_uniform_keypoints);
		std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_uniform_keypoints->size() << std::endl;
		
		pcl::copyPointCloud(*scene_uniform_keypoints, *scene_keypoints);
		pcl::copyPointCloud(*model_uniform_keypoints, *model_keypoints);
	}
	else
	{
		LOG(ERROR) << "keypoints config error !";
	}

	/**
	 *  Compute Descriptor for keypoints
	 */
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setNumberOfThreads(8);
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);

	/**
	 *  Find Model-Scene Correspondences with KdTree
	 */
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);
	std::vector<int> model_good_keypoints_indices;
	std::vector<int> scene_good_keypoints_indices;
	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0]))  //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
			model_good_keypoints_indices.push_back(corr.index_query);
			scene_good_keypoints_indices.push_back(corr.index_match);
		}
	}
	pcl::PointCloud<PointType>::Ptr model_good_kp(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_good_kp(new pcl::PointCloud<PointType>());
	pcl::copyPointCloud(*model_keypoints, model_good_keypoints_indices, *model_good_kp);
	pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

	/**
	 *  Clustering
	 */
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector < pcl::Correspondences > clustered_corrs;
	
	//  Using Hough3D
	if (use_hough_)
	{
		//  Compute (Keypoints) Reference Frames only for Hough
		pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
		pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);

		rf_est.setInputCloud(model_keypoints);
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model);
		rf_est.compute(*model_rf);

		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene);
		rf_est.compute(*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);

		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);

		clusterer.recognize(rototranslations, clustered_corrs);
	}
	else	// Using GeometricConsistency

	{
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		gc_clusterer.recognize(rototranslations, clustered_corrs);
	}
	/**
	 *  Visualization
	 */
	pcl::visualization::PCLVisualizer::Ptr viewer;
	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_model_good_kp(new pcl::PointCloud<PointType>());

	if (show_result_ && (rototranslations.size() <= 0))
	{
		viewer.reset((new pcl::visualization::PCLVisualizer("Recogniton CG")));
		viewer->setCameraPosition(-0.3, 0, -0.3, 0, 0, 1, -1, 0, 0); //视点 方向 上方向
		viewer->addCoordinateSystem(0.1);

		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_good_kp, *off_model_good_kp, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

		//show scene (red)
		CloudStyle sceneStyle = style_white;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, sceneStyle.r, sceneStyle.g, sceneStyle.b);
		viewer->addPointCloud(scene, scene_color_handler, "scene_cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sceneStyle.size, "scene_cloud");
		//show model (green+blue)
		CloudStyle modelStyle = style_indigo;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
		viewer->addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
		//show keypoints of instancs and model
		if (show_keypoints_)
		{
			CloudStyle goodKeypointStyle = style_yellow;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler(off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
				goodKeypointStyle.b);
			viewer->addPointCloud(off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

			pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler(scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
				goodKeypointStyle.b);
			viewer->addPointCloud(scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
		}

		viewer->spin();
	}

	/**
	 * Stop if no instances
	 */
	if (rototranslations.size() <= 0)
	{
		std::cout << "*** No instances found! ***" << std::endl;
		output_transformation = Eigen::Matrix4f::Identity();
		output_number = 0;
		return false;
	}
	else
	{
	  // Output results
		std::cout << "Recognized Instances: " << rototranslations.size() << std::endl << std::endl;
		//for (std::size_t i = 0; i < rototranslations.size(); ++i)
		//{
		//	std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		//	std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

		//	// Print the rotation matrix and translation vector
		//	Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		//	Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

		//	printf("\n");
		//	printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		//	printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		//	printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		//	printf("\n");
		//	printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
		//}
	}

	/**
	 * Generates clouds for each instances found
	 */
	std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

	for (std::size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
		instances.push_back(rotated_model);
	}

	/**
	 * ICP
	 */
	std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
	if (use_icp_)
	{
		std::cout << "--- ICP ---------" << std::endl;

		for (std::size_t i = 0; i < rototranslations.size(); ++i)
		{
			pcl::IterativeClosestPoint<PointType, PointType> icp;
			icp.setMaximumIterations(icp_max_iter_);
			icp.setMaxCorrespondenceDistance(icp_corr_distance_);
			icp.setInputTarget(scene);
			icp.setInputSource(instances[i]);
			pcl::PointCloud<PointType>::Ptr registered(new pcl::PointCloud<PointType>);
			icp.align(*registered);
			registered_instances.push_back(registered);
			std::cout << "Instance " << i << " ";
			std::cout << "Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
			if (icp.hasConverged())
			{
				std::cout << "Aligned!" << std::endl;
			}
			else
			{
				std::cout << "Not Aligned!" << std::endl;
			}
		}

		std::cout << "-----------------" << std::endl << std::endl;
	}

	/**
	 * Hypothesis Verification
	 */
		std::cout << "--- Hypotheses Verification ---" << std::endl;
		std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

	if (use_hv_)
	{
		pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

		GoHv.setSceneCloud(scene);  // Scene Cloud
		GoHv.addModels(registered_instances, true);  //Models to verify
		GoHv.setResolution(hv_resolution_);
		GoHv.setResolutionOccupancyGrid(hv_occupancy_grid_resolution_);
		GoHv.setInlierThreshold(hv_inlier_th_);
		GoHv.setOcclusionThreshold(hv_occlusion_th_);
		GoHv.setRegularizer(hv_regularizer_);
		GoHv.setRadiusClutter(hv_rad_clutter_);
		GoHv.setClutterRegularizer(hv_clutter_reg_);
		GoHv.setDetectClutter(hv_detect_clutter_);
		GoHv.setRadiusNormals(hv_rad_normals_);

		GoHv.verify();
		GoHv.getMask(hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

		for (int i = 0; i < hypotheses_mask.size(); i++)
		{
			if (hypotheses_mask[i])
			{
				std::cout << "Instance " << i << " is GOOD! <---" << std::endl;
			}
			else
			{
				std::cout << "Instance " << i << " is bad!" << std::endl;
			}
		}
		std::cout << "-------------------------------" << std::endl;
	}

	if (show_result_)
	{
		viewer.reset(new pcl::visualization::PCLVisualizer("Hypothesis Verification"));
		viewer->setCameraPosition(-0.3, 0, -0.3, 0, 0, 1, -1, 0, 0); //视点 方向 上方向
		viewer->addCoordinateSystem(0.1);

		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_good_kp, *off_model_good_kp, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

		//show scene (red)
		CloudStyle sceneStyle = style_white;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_color_handler(scene, sceneStyle.r, sceneStyle.g, sceneStyle.b);
		viewer->addPointCloud(scene, scene_color_handler, "scene_cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, sceneStyle.size, "scene_cloud");
		//show model (green+blue)
		CloudStyle modelStyle = style_indigo;
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
		viewer->addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
		//show keypoints of instancs and model
		if (show_keypoints_)
		{
			CloudStyle goodKeypointStyle = style_yellow;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler(off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
				goodKeypointStyle.b);
			viewer->addPointCloud(off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

			pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler(scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
				goodKeypointStyle.b);
			viewer->addPointCloud(scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
		}

		for (std::size_t i = 0; i < instances.size(); ++i)
		{
			//show result of hypotheses
			std::stringstream ss_instance;
			ss_instance << "instance_" << i;
			CloudStyle registeredStyles = style_green;
			if (use_hv_)
			{
				registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
			}
			ss_instance << "_registered" << std::endl;
			pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler(registered_instances[i], registeredStyles.r,
				registeredStyles.g, registeredStyles.b);
			viewer->addPointCloud(registered_instances[i], registered_instance_color_handler, ss_instance.str());
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str());
			//show rotated_model of each instance
			if (show_rotated_model_)
			{
				CloudStyle clusterStyle = style_blue;
				pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler(instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
				viewer->addPointCloud(instances[i], instance_color_handler, ss_instance.str());
				viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str());
			}
			//show correspondences of each instance
			if (show_correspondences_)
			{
				for (std::size_t j = 0; j < clustered_corrs[i].size(); ++j)
				{
					std::stringstream ss_line;
					ss_line << "correspondence_line" << i << "_" << j;
					PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
					PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);
					//drawing a line for each pair of clustered correspondences found between the model and the scene
					viewer->addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
				}
			}
		}

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}
	}

	//Output Result
	if (use_hv_)
	{
		for (int i = 0; i < hypotheses_mask.size(); i++)
		{
			if (hypotheses_mask[i])
			{
				std::cout << "Instance " << i << " is Final Output! <---" << std::endl;
				std::cout << "-------------------------------" << std::endl;
				output_transformation = rototranslations[i];
				output_number = 1;
				return true;
			}
		}	
		{
			std::cout << "No Instance Output! <---" << std::endl;
			std::cout << "-------------------------------" << std::endl;
			output_transformation = Eigen::Matrix4f::Identity();
			output_number = 0;
			return false;
		}

	}
	else if(rototranslations.size() > 0)
	{
		std::cout << "Instance " << 0 << " is Final Output! <---" << std::endl;
		std::cout << "-------------------------------" << std::endl;
		output_transformation = rototranslations[0];
		output_number = 1;
		return true;
	}
	else
	{
		std::cout << "No Instance Output! <---" << std::endl;
		std::cout << "-------------------------------" << std::endl;
		output_transformation = Eigen::Matrix4f::Identity();
		output_number = 0;
		return false;
	}

}

bool gpd::Recognition3DCG::TrainModel(std::vector<PointCloud::Ptr> cloud_models)
{
	return false;
}

IRecognition* GetRecognition3DCG()
{
	IRecognition* p_irecognition = new Recognition3DCG();
	return p_irecognition;
} 

