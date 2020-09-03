#pragma once
#include "stdafx.h"

struct JsonOutType
{
	bool success;
	int json_int;
	float json_float;
	double json_double;
	bool json_bool;
	std::string json_string;
};

class ICameraData
{
	public:
	
	virtual bool GetSharedMemImages(cv::Mat color, cv::Mat depth, cv::Mat mask, std::string label) = 0;
	virtual bool SetParameters(std::string JsonFilePath) = 0;
	virtual bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud) = 0;
	virtual bool LoadPointCloud(std::string file_name, PointCloud::Ptr object_model) = 0;
	virtual bool Load2DImage(cv::Mat image, std::string file_name) = 0;
	virtual void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name) = 0;
	virtual void ShowImage(const cv::Mat image, std::string window_name) = 0;
 	virtual JsonOutType ReadJsonFile(std::string file_name, std::string key_name, const char* out_type) = 0;
	virtual JsonOutType ReadJsonString(std::string json_string, std::string key_name, const char* out_type) = 0;

	virtual void test() = 0;
};

extern "C" __declspec(dllexport) ICameraData* APIENTRY GetCameraData();
