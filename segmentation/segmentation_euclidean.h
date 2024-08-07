#ifndef SEGMENTATION_EUCLIDEAN_H_
#define SEGMENTATION_EUCLIDEAN_H_

#include"isegmentation.h"
#include "../camera_data/camera_data.h"

#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif


class SegmentationEuclidean : public ISegmentation
{
public:
	SegmentationEuclidean();
	~SegmentationEuclidean();
	bool Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg);
	bool SetParameters(const std::string config_file);

private:
	std::shared_ptr<ICameraData> p_seg_cameradata_;
	float uniform_sampling;
	Eigen::Vector4f subsampling_leaf_size;
	float cluster_tolerance;
	float min_cluster_size;
	float max_cluster_size;
	bool visualization_eucli;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	// 定义颜色数组，用于可视化
	static unsigned char colors[20*3];
};

#endif
