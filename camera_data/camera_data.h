#pragma once
#include "icamera_data.h"

class CameraData :public ICameraData
{
  public:

	// π≤œÌƒ⁄¥Ê÷∏’Î
	LPVOID pcolorBuffer;                                  
	LPVOID pdepthBuffer;                                  
	LPVOID mask_buffer;
	LPVOID mask_collision_buffer;
	LPVOID label_buffer;
	LPVOID point_buffer;


	HANDLE hcolorMap;
	HANDLE hdepthMap;
	HANDLE mask_map;
	HANDLE mask_collision_map;
	HANDLE label_map;
	HANDLE point_map;

	bool isOpenCameraMapping;
	bool isOpenMaskMapping;
	bool DebugVisualization;

	DWORD BUF_SIZE;

	enum CameraState
	{
		DISCONNECTED = 0,
		CONNECTED
	};

	enum PictureState
	{
		FIRST,
		WRITING,
		WRITED,
		READING,
		READED
	};

	CameraData();
	~CameraData();

	bool GetCameraImages(cv::Mat& color, cv::Mat& depth);
	bool GetMaskAndLabel(cv::Mat& mask, cv::Mat& mask_collision, std::string& label);
	bool SetParameters(std::string JsonFilePath);
	bool LoadPointCloud(std::string file_name, PointCloud::Ptr object_model);
	bool Load2DImage(cv::Mat image, std::string file_name);
	void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name);
	void ShowImage(const cv::Mat image, std::string window_name);
	
	JsonOutType ReadJsonObject(Json::Value json_object, std::string key_name, const char* out_type);
	JsonOutType ReadJsonFile(std::string file_name, std::string key_name, const char* out_type);
	JsonOutType ReadJsonString(std::string json_string, std::string key_name, const char* out_type);
	bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud);
	bool DepthtoPointCloud_Collision(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud);
};


