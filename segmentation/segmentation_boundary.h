#ifndef SEGMENTATION_BOUNDARY_H_
#define SEGMENTATION_BOUNDARY_H_

#include"isegmentation.h"
#include "../camera_data/camera_data.h"

#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif

class SegmentationBoundary : public ISegmentation
{
public:
	SegmentationBoundary();
	~SegmentationBoundary();
	bool Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg);
	bool SetParameters(const std::string config_file);
private:
	std::shared_ptr<ICameraData> p_seg_cameradata_;
	float sample_3d;
	Eigen::Vector4f subsampling_leaf_size;
	float angle_threshold;
	float normal_radius;
	float boundary_radius;
};

#endif
