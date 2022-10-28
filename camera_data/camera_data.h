#pragma once
#include "icamera_data.h"
#include "json/json.h"

class CameraData :public ICameraData
{
  public:
	// π≤œÌƒ⁄¥Ê÷∏’Î
	LPVOID pcolorBuffer;                                  
	LPVOID pdepthBuffer;                                  
	LPVOID mask_buffer;
	LPVOID label_buffer;

	HANDLE hcolorMap;
	HANDLE hdepthMap;
	HANDLE mask_map;
	HANDLE label_map;

	bool isOpenCameraMapping;
	bool isOpenMaskMapping;
	bool DebugVisualization;

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

	int ucharToMat(uchar *p2, cv::Mat& src, int flag);

	bool GetCameraImages(cv::Mat& color, cv::Mat& depth);
	bool GetMaskAndLabel(cv::Mat& mask, std::string label);
	bool SetParameters(std::string JsonFilePath);
	bool LoadPointCloud(std::string file_name, PointCloud::Ptr object_model);
	bool Load3DModel(std::string file_name, PointCloud::Ptr object_model);
	bool Load2DImage(cv::Mat image, std::string file_name);
	void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name);
	void ShowPointCloud_NonBlocking(const PointCloud::Ptr pointcloud, std::string window_name);
	void ShowImage(const cv::Mat image, std::string window_name);
	
	JsonOutType ReadJsonFile(std::string file_name, std::string key_name, const char* out_type);
	JsonOutType ReadJsonString(std::string json_string, std::string key_name, const char* out_type);
	bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud);
};


