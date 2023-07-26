#ifndef I_SEGMENTATION_H
#define I_SEGMENTATION_H

#include "stdafx.h"

#ifdef SEGMENTATION_EXPORTS
#define SEGMENTATION_API __declspec(dllexport)
#else
#define SEGMENTATION_API __declspec(dllimport)
#endif

class ISegmentation
{
	public:
	virtual ~ISegmentation() = 0;
	virtual bool Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model = nullptr, PointCloud::Ptr cloud_seg = nullptr) = 0;
	virtual bool SetParameters(const std::string config_file) = 0;

	bool usage;
private:
};

SEGMENTATION_API ISegmentation*  GetSegmentationSAC();
SEGMENTATION_API ISegmentation*  GetSegmentationOBB();
SEGMENTATION_API ISegmentation*  GetSegmentationBoundary();
SEGMENTATION_API ISegmentation*  GetSegmentationEuclidean();

#endif


