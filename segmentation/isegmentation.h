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
	virtual bool Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model = nullptr, PointCloud::Ptr cloud_seg = nullptr) = 0;
	virtual bool SetParameters(const std::string config_file) = 0;
};

__SEGMENTATION_API ISegmentation*  GetSegmentationSAC();
__SEGMENTATION_API ISegmentation*  GetSegmentationOBB();
__SEGMENTATION_API ISegmentation*  GetSegmentationBoundary();
__SEGMENTATION_API ISegmentation*  GetSegmentationEuclidean();

#endif


