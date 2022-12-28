#ifndef I_SEGMENTATION_H
#define I_SEGMENTATION_H

#include "stdafx.h"

#ifdef __SEGMEANTATION_EXPORT
#define __SEGMENTATION_API __declspec(dllexport)
#else
#define __SEGMENTATION_API __declspec(dllimport)
#endif

class ISegmentation
{
	public:
	virtual ~ISegmentation() = 0;
	virtual bool segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model = nullptr, PointCloud::Ptr cloud_seg = nullptr) = 0;
};

__SEGMENTATION_API ISegmentation*  GetSegmentationSAC(const std::string config_path);
__SEGMENTATION_API ISegmentation*  GetSegmentationOBB(const std::string config_path);

#endif


