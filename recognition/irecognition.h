#ifndef IRECOGNITION_H
#define IRECOGNITION_H

#include "stdafx.h"
#include "../camera_data/camera_data.h"
class IRecognition 
{
	public:
	virtual bool SetParameters(const std::string config_file) = 0;
	virtual bool Compute(const PointCloud::Ptr cloud_scene, const std::vector<PointCloud::Ptr> cloud_models, Eigen::Matrix4f transformation) = 0;
	virtual bool TrainPPFModel(std::vector<PointCloud::Ptr> cloud_models) = 0;
};

extern "C" __declspec(dllexport) IRecognition* APIENTRY GetRecognition3DPPF();

#endif
