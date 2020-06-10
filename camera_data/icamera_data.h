#pragma once
#include "stdafx.h"

class ICameraData
{
	public:
	
	virtual bool GetSharedMemImages() = 0;
	virtual bool SetParameters() = 0;
	virtual bool DepthtoPointCloud() = 0;
	virtual bool LoadPointCloud(PointCloud::Ptr object_model, std::string file_name) = 0;
	virtual bool LoadImage(cv::Mat image, std::string file_name) = 0;
	virtual void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name) = 0;
	virtual void ShowImage(const cv::Mat image, std::string window_name) = 0;

	virtual void test() = 0;
};

extern "C" __declspec(dllexport) ICameraData* APIENTRY GetCameraData();
