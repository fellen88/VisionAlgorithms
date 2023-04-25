#ifndef IRECOGNITION_H
#define IRECOGNITION_H

#include "stdafx.h"
#include "../camera_data/camera_data.h"

#ifdef RECOGNITION_EXPORTS
#define RECOGNITION_API __declspec(dllexport)
#else
#define RECOGNITION_API __declspec(dllimport)
#endif

class IRecognition 
{
	public:
	virtual bool SetParameters(const std::string config_file) = 0;
	virtual bool Recognize(const PointCloud::Ptr cloud_scene, const std::vector<PointCloud::Ptr> cloud_model,
		Eigen::Matrix4f& transformation, size_t& output_number) = 0;
	virtual bool TrainModel(std::vector<PointCloud::Ptr> cloud_models) = 0;
};

RECOGNITION_API IRecognition* APIENTRY GetRecognition3DPPF();
RECOGNITION_API IRecognition* APIENTRY GetRecognition3DCG();

#endif
