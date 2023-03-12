#ifndef RECOGNITION_3D_H
#define RECOGNITION_3D_H

#include "irecognition.h"

class Recognition3D : public IRecognition
{
public:
	Recognition3D();
	~Recognition3D();
	virtual bool SetParameters(const std::string config_file) = 0;
	virtual bool Compute(const PointCloud::Ptr cloud_scene, const std::vector<PointCloud::Ptr> cloud_models, Eigen::Matrix4f transformation) = 0;
	virtual bool TrainPPFModel(std::vector<PointCloud::Ptr> cloud_models) = 0;

};

#endif

