// camera_data.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include <fstream>
#include "camera_data.h"

#define COLORMEMORYNAME "color"
#define DEPTHMEMORYNAME "depth"
#define CAMERASTATE "cameraState"
#define PICTURESTATE "pictureState"

std::string JsonFilePath = "plugins//PoseEstimation//pose_estimation.json";

CameraData::CameraData()
{
	LOG(INFO) << "CameraData() ";
	image_width_= 848;
	image_height_ = 480;
	
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

		uchar *p1 = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_* 3);
		memcpy(p1, pcolorBuffer, sizeof(uchar)*image_height_*image_width_* 3);
		cv::Mat color(cv::Size(image_width_, image_height_), CV_8UC3);
		ucharToMat(p1, color, 0);
		cv::imshow("color", color);

		uchar *p2 = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_* 3);
		memcpy(p2, pdepthBuffer, sizeof(uchar)*image_height_*image_width_* 3);
		cv::Mat depth(cv::Size(image_width_, image_height_), CV_8UC3);
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

JsonOutType CameraData::ReadJsonFile(std::string file_name, std::string key_name, const char* out_type)
{
	JsonOutType json_out_type;
	Json::Value json_object;
	Json::Reader reader;
	ifstream json_file;
	json_file.open(file_name, ios::binary);

	if (!reader.parse(json_file, json_object))
	{
		cout << "json open error: " << GetLastError << endl;
		json_file.close();
		json_out_type.success = false;
		return json_out_type;
	}
	else if(0 == strcmp(out_type, "string"))
	{
		if (!json_object[key_name].isNull())
    {
			std::string strValue= json_object[key_name].asString(); 
			std::cout << strValue<< std::endl; 
    }
		json_file.close();
		return json_out_type;
	}
	return json_out_type;
}

JsonOutType CameraData::ReadJsonString(std::string json_string, std::string key_name, const char * out_type)
{
	JsonOutType json_out_type;
	Json::Value json_object;
	Json::Reader reader;
	if (!reader.parse(json_string, json_object))
	{
		LOG(ERROR) << "read json string failed";
		json_out_type.success = false;
		return json_out_type;
	}
	else if(0 == strcmp(out_type, "string"))
	{
		if (!json_object[key_name].isNull())
    {
			std::string strValue= json_object[key_name].asString(); 
			std::cout << strValue<< std::endl; 
    }
		return json_out_type;
	}
	return json_out_type;
}
 
bool CameraData::SetParameters()
{
	if (ReadJsonFile(JsonFilePath, "ImageHeight", "int").success)
	{
		image_height_ = ReadJsonFile(JsonFilePath, "ImageHeight", "int").json_int;
	}
	else
	{
		LOG(ERROR) << "set ImageHeight from json error";
		return false;
	}
	if(ReadJsonFile(JsonFilePath, "ImageWidth", "int").success)
	{
		image_width_ = ReadJsonFile(JsonFilePath, "ImageWidth", "int").json_int;
	}
	else
	{
		LOG(ERROR) << "set ImageWidth from json error";
		return false;
	}
	if (ReadJsonFile(JsonFilePath, "Fx", "float").success)
	{
		fx_ = ReadJsonFile(JsonFilePath, "Fx", "float").json_float;
	}
	else
	{
		LOG(ERROR) << "set Fx from json error";
		return false;
	}
	if(ReadJsonFile(JsonFilePath, "Fy", "float").success)
	{
		fy_ = ReadJsonFile(JsonFilePath, "Fy", "float").json_float;
	}
	else
	{
		LOG(ERROR) << "set Fy from json error";
		return false;
	}
	if (ReadJsonFile(JsonFilePath, "Cx", "float").success)
	{
		cx_ = ReadJsonFile(JsonFilePath, "Cx", "float").json_float;
	}
	else
	{
		LOG(ERROR) << "set Cx from json error";
		return false;
	}
	if (ReadJsonFile(JsonFilePath, "Cy", "float").success)
	{
		cy_ = ReadJsonFile(JsonFilePath, "Cy", "float").json_float;
	}
	else
	{
		LOG(ERROR) << "set Cy from json error";
		return false;
	}
	return true;
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
