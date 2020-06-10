#pragma once
#include "icamera_data.h"

class CameraData :public ICameraData
{
  public:
	void test();

	int WIDTH;
	int HEIGHT; 
	
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

	bool GetSharedMemImages();
	bool SetParameters();
	bool DepthtoPointCloud();
	bool LoadPointCloud(PointCloud::Ptr object_model, std::string file_name);
	bool LoadImage(cv::Mat image, std::string file_name);
	void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name);
	void ShowImage(const cv::Mat image, std::string window_name);
};

