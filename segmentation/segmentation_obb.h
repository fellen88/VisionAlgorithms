#ifndef SEGMENTATION_OBB_H_
#define SEGMENTATION_OBB_H_

#include"isegmentation.h"
#include "../camera_data/camera_data.h"

#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/vision_camera_data.lib")
#endif

class SegmentationOBB : public ISegmentation
{
public:
	SegmentationOBB();
	~SegmentationOBB();
	bool Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg);
	bool SetParameters(const std::string config_file);
private:
	std::shared_ptr<ICameraData> p_obb_cameradata_;
	float sample_3d;
	bool debug_visualization;
	float l_offset;
	float w_offset;
	float h_offset;
	Eigen::Vector4f subsampling_leaf_size;

	void CalulateOBB(const PointCloud::Ptr cloud_in);
};

#endif