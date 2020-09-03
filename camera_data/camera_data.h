#pragma once
#include "icamera_data.h"
#include "json/json.h"

class CameraData :public ICameraData
{
  public:

	int image_height_;
	int image_width_;
	float fx_;
	float fy_;
	float cx_;
	float cy_;
	int scale_factor_;
	
	// π≤œÌƒ⁄¥Ê÷∏’Î
	LPVOID pcolorBuffer;                                  
	LPVOID pdepthBuffer;                                  
	LPVOID cameraStateBuffer;
	LPVOID pictureStateBuffer;

	HANDLE hcolorMap;
	HANDLE hdepthMap;
	HANDLE hcameraMap;
	HANDLE hpictureMap;

	bool isOpenFileMapping;
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

	bool GetSharedMemImages(cv::Mat color, cv::Mat depth, cv::Mat mask, std::string label);
	bool SetParameters(std::string JsonFilePath);
	bool LoadPointCloud(std::string file_name, PointCloud::Ptr object_model);
	bool Load2DImage(cv::Mat image, std::string file_name);
	void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name);
	void ShowImage(const cv::Mat image, std::string window_name);
	
	JsonOutType ReadJsonFile(std::string file_name, std::string key_name, const char* out_type);
	JsonOutType ReadJsonString(std::string json_string, std::string key_name, const char* out_type);
	bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud);
	void test();
};


