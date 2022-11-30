#ifndef ISEGMENTATION_H
#define ISEGMENTATION_H

#include "stdafx.h"
class ISegmentation
{
	public:
	virtual bool segment(PointCloud::Ptr cloud_scene) = 0;
};

extern "C" __declspec(dllexport) ISegmentation* APIENTRY GetSegmentationSAC(const std::string config_path);
#endif


