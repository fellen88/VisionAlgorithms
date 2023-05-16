#ifndef SEGMENTATION_SAC_H_
#define SEGMENTATION_SAC_H_

#include"isegmentation.h"
#include "../camera_data/camera_data.h"

#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif


class SegmentationSAC : public ISegmentation	
{
public:
	SegmentationSAC();
	~SegmentationSAC();
	bool Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg);
	bool SetParameters(const std::string config_file);
private:
	std::shared_ptr<ICameraData> p_seg_cameradata_;
	float uniform_sampling;
	Eigen::Vector4f subsampling_leaf_size;
	float distance_threshold;
};

#endif
