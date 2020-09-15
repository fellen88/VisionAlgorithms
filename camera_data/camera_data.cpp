// camera_data.cpp : 定义 DLL 应用程序的导出函数。

#include "stdafx.h"
#include <fstream>
#include "camera_data.h"

#define COLORMEMORYNAME L"color"
#define DEPTHMEMORYNAME L"depth"

CameraData::CameraData()
{
	LOG(INFO) << "CameraData::CameraData() ";
	
	pcolorBuffer = nullptr;                                   // 共享内存指针
	pdepthBuffer = nullptr;                                   // 共享内存指针
	isOpenFileMapping = false;

	hcolorMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)COLORMEMORYNAME);
	hdepthMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)DEPTHMEMORYNAME);
	if (NULL != hcolorMap && NULL != hdepthMap)
	{
		//打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		pcolorBuffer = ::MapViewOfFile(hcolorMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		pdepthBuffer = ::MapViewOfFile(hdepthMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		isOpenFileMapping = true;
		LOG(INFO) << "OpenCameraMapping ok! ";
	}
}

CameraData::~CameraData()
{
	// 解除文件映射，关闭内存映射文件对象句柄
	::UnmapViewOfFile(pcolorBuffer);
	::CloseHandle(hcolorMap);
	::UnmapViewOfFile(pdepthBuffer);
	::CloseHandle(hdepthMap);
}

void Uchar2Mat(uchar* p, cv::Mat &src) {
	int img_width = src.cols;
	int img_height = src.rows;
	if (src.channels() == 3) {
		for (int i = 0; i < img_width * img_height * 3; i++)
		{
			src.at<cv::Vec3b>(i / (img_width * 3), (i % (img_width * 3)) / 3)[i % 3] = p[i];//BGR格式

		}
	}

	else if (src.channels() == 1)
	{
		for (int i = 0; i < img_width * img_height; i++)
		{
			src.at<uchar>(i / (img_width), i % (img_width)) = p[i];

		}
	}
	return;
}

bool CameraData::GetCameraImages(cv::Mat &image_color, cv::Mat& image_depth)
{
	if (false == isOpenFileMapping)
	{
		LOG(ERROR) << "OpenFileMapping error! ";
		LOG(ERROR) << "hcolorMap: " << hcolorMap;
		LOG(ERROR) << "hdepthMap: " << hdepthMap;
		return false;
	}
	uchar *p1 = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_* 3);
	memcpy(p1, pcolorBuffer, sizeof(uchar)*image_height_*image_width_* 3);
	cv::Mat color(cv::Size(image_width_, image_height_), CV_8UC3);
	for (int i = 0; i < image_width_ * image_height_ * 3; i++)
	{
		color.at<cv::Vec3b>(i / (image_width_ * 3), (i % (image_width_ * 3)) / 3)[i % 3] = p1[i];//BGR格式
	}

	image_color = color.clone();

	ushort *p2 = (ushort*)malloc(sizeof(ushort)*image_height_*image_width_);
	memcpy(p2, pdepthBuffer, sizeof(ushort)*image_height_*image_width_);
	cv::Mat depth(cv::Size(image_width_, image_height_), CV_16UC1);
	for (int i = 0; i < image_width_ * image_height_ * 3; i++)
	{
		depth.at<ushort>(i / (image_width_), i % (image_width_)) = p2[i];
	}
	image_depth = depth;

	delete[] p1;
	delete[] p2;
	
	return true;
}

bool CameraData::InitMaskSharedMem()
{
	mask_buffer = nullptr;                                   // 共享内存指针
	label_buffer = nullptr;                                   // 共享内存指针

	mask_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"mask");
	label_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"mask_label");
	if (NULL != mask_map && NULL != label_map)
	{
		//打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		mask_buffer = ::MapViewOfFile(mask_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    label_buffer = ::MapViewOfFile(label_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		LOG(INFO) << "OpenMaskMapping ok! ";
		return true;
	}
	return false;
}

bool CameraData::GetMaskAndLabel(cv::Mat & image_mask, std::string label)
{
	image_mask = cv::Mat::ones(cv::Size(image_width_, image_height_), CV_8UC1);
	uchar *p_mask = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_ * 3);
	memcpy(p_mask, pcolorBuffer, sizeof(uchar)*image_height_*image_width_ * 3);
	cv::Mat mask(cv::Size(image_width_, image_height_), CV_8UC1);
	for (int i = 0; i < image_width_ * image_height_ * 3; i++)
	{
		mask.at<cv::Vec3b>(i / (image_width_ * 3), (i % (image_width_ * 3)) / 3)[i % 3] = p_mask[i];//BGR格式
	}

	image_mask = mask.clone();

	char *p_label = (char*)malloc(sizeof(char) * 50);

	delete[] p_mask;
	delete[] p_label;
	return true;
}

bool CameraData::LoadPointCloud(std::string file_name, PointCloud::Ptr object_model)
{
	pcl::PCDReader reader;
	if (reader.read(file_name, *object_model)  < 0)
	{
		return false;
	}
	return true;
}

bool CameraData::Load2DImage(cv::Mat image, std::string file_name)
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
	if (nullptr != pointcloud)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer(window_name));
		view->addPointCloud(pointcloud);
		view->spin ();
	}
	else
	{
		LOG(ERROR) << window_name << " is a nullptr pointlcoud!";
	}
}

void CameraData::ShowImage(const cv::Mat image, std::string window_name)
{
	if (!image.empty())
	{
		cv::imshow(window_name, image);
		cv::waitKey(0);
	}
	else {
		LOG(ERROR) << window_name << " is empty image!";
	}
}

JsonOutType CameraData::ReadJsonFile(std::string file_name, std::string key_name, const char* out_type)
{
	JsonOutType json_out_type;
	Json::Value json_object;
	Json::Reader reader;
	ifstream json_file;
	json_file.open(file_name, ios::binary);

	if (false == reader.parse(json_file, json_object))
	{
		LOG(ERROR) << "read json file error！";
		json_file.close();
		json_out_type.success = false;
		return json_out_type;
	}
	else if(0 == strcmp(out_type, "string"))
	{
		if (!json_object[key_name].isNull())
    {
			json_out_type.success = true;
			json_out_type.json_string = json_object[key_name].asString();
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_string;
    }
		json_file.close();
		return json_out_type;
	}
	if(0 == strcmp(out_type, "int"))
	{
		if (!json_object[key_name].isNull())
    {
			json_out_type.success = true;
			json_out_type.json_int = json_object[key_name].asInt(); 
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_int;
    }
		json_file.close();
		return json_out_type;
	}
	if(0 == strcmp(out_type, "float"))
	{
		if (!json_object[key_name].isNull())
    {
			json_out_type.success = true;
			json_out_type.json_float = json_object[key_name].asFloat(); 
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_float;
    }
		json_file.close();
		return json_out_type;
	}
	if(0 == strcmp(out_type, "bool"))
	{
		if (!json_object[key_name].isNull())
    {
			json_out_type.success = true;
			json_out_type.json_bool = json_object[key_name].asBool(); 
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_bool;
    }
		json_file.close();
		return json_out_type;
	}
	if(0 == strcmp(out_type, "double"))
	{
		if (!json_object[key_name].isNull())
    {
			json_out_type.success = true;
			json_out_type.json_double = json_object[key_name].asDouble(); 
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_double;
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
	else if (0 == strcmp(out_type, "string"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_string = json_object[key_name].asString();
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_string;
		}
		return json_out_type;
	}
	if (0 == strcmp(out_type, "int"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_int = json_object[key_name].asInt();
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_int;
		}
		return json_out_type;
	}
	if (0 == strcmp(out_type, "float"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_float = json_object[key_name].asFloat();
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_float;
		}
		return json_out_type;
	}
	if (0 == strcmp(out_type, "bool"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_bool = json_object[key_name].asBool();
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_bool;
		}
		return json_out_type;
	}
	if (0 == strcmp(out_type, "double"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_double = json_object[key_name].asDouble();
			LOG(INFO) << key_name << " (json key) : " << json_out_type.json_double;
		}
		return json_out_type;
	}
	return json_out_type;
}
 
bool CameraData::SetParameters(std::string JsonFilePath)
{
	JsonOutType json_out_type = { false, 0, 0.0, 0.0, false,"" };
	json_out_type = ReadJsonFile(JsonFilePath, "ImageHeight", "int");
	if (json_out_type.success)
	{
		image_height_ = json_out_type.json_int;
	}
	else
	{
		LOG(ERROR) << "set ImageHeight from json error";
		return false;
	}

	json_out_type = ReadJsonFile(JsonFilePath, "ImageWidth", "int");
	if(json_out_type.success)
	{
		image_width_ = json_out_type.json_int;
	}
	else
	{
		LOG(ERROR) << "set ImageWidth from json error";
		return false;
	}

	json_out_type = ReadJsonFile(JsonFilePath, "Fx", "float");
	if (json_out_type.success)
	{
		fx_ = json_out_type.json_float;
	}
	else
	{
		LOG(ERROR) << "set Fx from json error";
		return false;
	}

	json_out_type = ReadJsonFile(JsonFilePath, "Fy", "float");
	if(json_out_type.success)
	{
		fy_ = json_out_type.json_float;
	}
	else
	{
		LOG(ERROR) << "set Fy from json error";
		return false;
	}

	json_out_type = ReadJsonFile(JsonFilePath, "Cx", "float");
	if (json_out_type.success)
	{
		cx_ = json_out_type.json_float;
	}
	else
	{
		LOG(ERROR) << "set Cx from json error";
		return false;
	}

	json_out_type = ReadJsonFile(JsonFilePath, "Cy", "float");
	if (json_out_type.success)
	{
		cy_ = json_out_type.json_float;
	}
	else
	{
		LOG(ERROR) << "set Cy from json error";
		return false;
	}

	json_out_type = ReadJsonFile(JsonFilePath, "ScaleFactor3D", "int");
	if (json_out_type.success)
	{
		scale_factor_ = json_out_type.json_int;
	}
	else
	{
		LOG(ERROR) << "set ScaleFactor3D from json error";
		return false;
	}

	return true;
}

bool CameraData::DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr CloudMask)
{
	for(int rows = 0; rows < image_height_; rows++)
    {
    for(int cols = 0; cols < image_width_; cols++ )
    {
		float d = 0;
		if (!Depth.empty())
		{
			//获取深度图中对应点的深度值
			d = Depth.at<uchar>(rows, cols);
		}
		else
		{
			LOG(ERROR) << "depth image empty!";
			return false;
		}

      //有效范围内的点
		//if ((d > 0.5*scale_factor_) && (d < 1.5*scale_factor_))
		{
			//判断mask中是否是物体的点
			if (!Mask.empty())
			{
				unsigned char mask_value = Mask.at<unsigned char>(rows, cols);
				if (mask_value == 0)
					continue;
			}
			else
			{
				LOG(ERROR) << ("mask image empty!");
			    return false;
			}
			//计算这个点的空间坐标
			pcl::PointXYZ PointWorld;
			PointWorld.z = double(d) / scale_factor_;
			//ROS_INFO("D = %f", d);
			PointWorld.x = (rows - cx_)*PointWorld.z / fx_;
			PointWorld.y = (cols - cy_)*PointWorld.z / fy_;
			CloudMask->points.push_back(PointWorld);
      }
    }
  }
  //设置点云属性，采用无序排列方式存储点云
  CloudMask->height = 1;
  CloudMask->width = CloudMask->points.size();
  LOG(INFO) << "mask cloud size = " << CloudMask->width;
  if(0 == CloudMask->points.size())
  {
    LOG(ERROR)<<("Mask points number = 0 !!!");
    return false;
  }
  //去除NaN点
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*CloudMask, *CloudMask, nan_indices);
  CloudMask->is_dense = false;
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
