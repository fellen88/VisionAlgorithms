#ifndef RECOGNITION_3D_PPF_H
#define RECOGNITION_3D_PPF_H

#include "recognition_3d.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/vision_camera_data.lib")
#endif

class Recognition3DPPF : public Recognition3D
{
public:
	ICameraData* p_dataprocess_;

	Recognition3DPPF(std::string config);
	~Recognition3DPPF();

	std::vector<PointCloudWithNormals::Ptr> cloud_models_with_normals;
	float sample_3d;
	Eigen::Vector4f subsampling_leaf_size;
	float search_radius;
	float position_clustering_threshold;
	int rotation_Clustering_threshold;
	int point_sampling_rate;
	std::vector<pcl::PPFHashMapSearch::Ptr> hashmap_search_vector;
	
	bool Compute(const PointCloud::Ptr cloud_scene, const std::vector<PointCloud::Ptr> cloud_models);
	bool TrainPPFModel(std::vector<PointCloud::Ptr> cloud_models);

};

#endif

#pragma once
