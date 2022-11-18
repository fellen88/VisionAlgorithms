#ifndef SEGMENTATION_SAC_H
#define SEGMENTATION_SAC_H

#include"isegmentation.h"
#include "../camera_data/camera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/grasp_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/grasp_camera_data.lib")
#endif

class SegmentationSAC : public ISegmentation	
{
public:

	SegmentationSAC(std::string config_path);
	~SegmentationSAC();
	
	ICameraData* p_seg_cameradata_;
	float distance_threshold;
	bool segment(PointCloud::Ptr cloud_scene);

};

#endif
