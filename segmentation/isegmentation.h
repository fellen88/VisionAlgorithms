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
	virtual bool segment(PointCloud::Ptr cloud_scene) = 0;
};

__SEGMENTATION_API ISegmentation* APIENTRY GetSegmentationSAC(const std::string config_path);
#endif


