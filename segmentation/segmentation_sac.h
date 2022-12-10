#ifndef SEGMENTATION_SAC_H_
#define SEGMENTATION_SAC_H_

#define __SEGMEANTATION_EXPORT
#include"isegmentation.h"
#include "../camera_data/camera_data.h"

#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/vision_camera_data.lib")
#endif

class SegmentationSAC : public ISegmentation	
{
public:
	SegmentationSAC(const std::string config_path);
	~SegmentationSAC();
	bool segment(PointCloud::Ptr cloud_scene);
private:
	ICameraData* p_seg_cameradata_;
	float sample_3d;
	Eigen::Vector4f subsampling_leaf_size;
	float distance_threshold;
};

#endif
