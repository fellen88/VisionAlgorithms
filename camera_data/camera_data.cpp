// camera_data.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "camera_data.h"

#define COLORMEMORYNAME "color"
#define DEPTHMEMORYNAME "depth"
#define CAMERASTATE "cameraState"
#define PICTURESTATE "pictureState"

CameraData::CameraData()
{
	LOG(INFO) << "CameraData() ";
	WIDTH = 848;
	HEIGHT = 480;
	
	cameraStateBuffer = nullptr;
	pictureStateBuffer = nullptr;
	pcolorBuffer = nullptr;                                   // 共享内存指针
	pdepthBuffer = nullptr;                                   // 共享内存指针

	isOpenFileMapping = false;

	hcameraMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)CAMERASTATE);
	hpictureMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)PICTURESTATE);
	hcolorMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)COLORMEMORYNAME);
	hdepthMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)DEPTHMEMORYNAME);
	if (NULL != hcolorMap && NULL != hdepthMap && NULL != hcameraMap && NULL != hpictureMap)
	{
		// 打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		pcolorBuffer = ::MapViewOfFile(hcolorMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		pdepthBuffer = ::MapViewOfFile(hdepthMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		cameraStateBuffer = ::MapViewOfFile(hcameraMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		pictureStateBuffer = ::MapViewOfFile(hpictureMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		isOpenFileMapping = true;
	}
}

CameraData::~CameraData()
{
	// 解除文件映射，关闭内存映射文件对象句柄
	::UnmapViewOfFile(pcolorBuffer);
	::CloseHandle(hcolorMap);
	::UnmapViewOfFile(pdepthBuffer);
	::CloseHandle(hdepthMap);
	::UnmapViewOfFile(cameraStateBuffer);
	::CloseHandle(cameraStateBuffer);
	::UnmapViewOfFile(pictureStateBuffer);
	::CloseHandle(pictureStateBuffer);
}

int CameraData::ucharToMat(uchar *p2, cv::Mat& src, int flag)
{
	int img_width = src.cols;
	int img_height = src.rows;
	//cv::Mat img(Size(img_width, img_height), CV_8UC3);
	for (int i = 0; i < img_width * img_height * 3; i++)
	{
		src.at<cv::Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i % 3] = p2[i];//BGR格式
		//src.at<Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i % 3] = p2[i];//换为RGB使用
	}
	flag = 1;
	return flag;
}

bool CameraData::GetSharedMemImages()
{
	if (false == isOpenFileMapping)
	{
		return false;
	}
	CameraState cameraState;
	memcpy(&cameraState, cameraStateBuffer, sizeof(CameraState));
	if (CameraState::DISCONNECTED == cameraState)
	{
		Sleep(10);
		return false;
	}
	PictureState picture;
	memcpy(&picture, pictureStateBuffer, sizeof(PictureState));
	if (PictureState::WRITED == picture)
	{
		picture = PictureState::READING;
		memcpy(pictureStateBuffer, &picture, sizeof(PictureState));

		uchar *p1 = (uchar*)malloc(sizeof(uchar)*HEIGHT*WIDTH * 3);
		memcpy(p1, pcolorBuffer, sizeof(uchar)*HEIGHT*WIDTH * 3);
		cv::Mat color(cv::Size(WIDTH, HEIGHT), CV_8UC3);
		ucharToMat(p1, color, 0);
		cv::imshow("color", color);

		uchar *p2 = (uchar*)malloc(sizeof(uchar)*HEIGHT*WIDTH * 3);
		memcpy(p2, pdepthBuffer, sizeof(uchar)*HEIGHT*WIDTH * 3);
		cv::Mat depth(cv::Size(WIDTH, HEIGHT), CV_8UC3);
		ucharToMat(p2, depth, 0);
		cv::imshow("depth", depth);

		cv::waitKey(1);
		delete[] p1;
		delete[] p2;
		picture = PictureState::READED;
		memcpy(pictureStateBuffer, &picture, sizeof(PictureState));
	}
	return true;
}

bool CameraData::SetParameters()
{
	return false;
}

bool CameraData::DepthtoPointCloud()
{
	if (1)
	{
		return false;
	}
	return true;
}

bool CameraData::LoadPointCloud(PointCloud::Ptr object_model, std::string file_name)
{
	pcl::PCDReader reader;
	if (reader.read(file_name, *object_model)  < 0)
	{
		return false;
	}
	return true;
}

bool CameraData::LoadImage(cv::Mat image, std::string file_name)
{
	image = cv::imread(file_name);

	if (image.data == NULL)
	{
		return false;
	}
	return true;
}

void CameraData::ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer(window_name));
	view->addPointCloud(pointcloud);
  view->spin ();
}

void CameraData::ShowImage(const cv::Mat image, std::string window_name)
{
	cv::imshow(window_name, image);
	cv::waitKey(0);
}

void CameraData::test()
{
	int a = 1;
	int b = 0;
	//int c = a / b;
}

ICameraData* GetCameraData()
{
	ICameraData* p_iregistration = new CameraData();
	return p_iregistration;
}
