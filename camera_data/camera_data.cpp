﻿// camera_data.cpp : 定义 DLL 应用程序的导出函数。

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
	mask_buffer = nullptr;                                   // 共享内存指针
	mask_collision_buffer = nullptr;                                   // 共享内存指针
	label_buffer = nullptr;                                   // 共享内存指针
	isOpenCameraMapping = false;
	isOpenMaskMapping = false;

	hcolorMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)COLORMEMORYNAME);
	hdepthMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)DEPTHMEMORYNAME);
	mask_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"mask");
	mask_collision_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"collision_area");
	label_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"mask_label");
	if (NULL != hcolorMap && NULL != hdepthMap)
	{
		//打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		pcolorBuffer = ::MapViewOfFile(hcolorMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		pdepthBuffer = ::MapViewOfFile(hdepthMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		isOpenCameraMapping = true;
		LOG(INFO) << "OpenCameraMapping ok! ";
	}
	if (NULL != mask_map && NULL != label_map && NULL != mask_collision_map)
	{
		//打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		mask_buffer = ::MapViewOfFile(mask_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		mask_collision_buffer = ::MapViewOfFile(mask_collision_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    label_buffer = ::MapViewOfFile(label_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		isOpenMaskMapping = true;
		LOG(INFO) << "OpenMaskMapping ok! ";
	}

  //************写共享内存****************//
	BUF_SIZE = 1280*720*4*3;
	// 创建共享文件句柄 
	point_map = CreateFileMapping(
			INVALID_HANDLE_VALUE,   // 物理文件句柄
			NULL,                   // 默认安全级别
			PAGE_READWRITE,         // 可读可写
			0,                      // 高位文件大小
			BUF_SIZE,               // 低位文件大小
			L"points_collision"          // 共享内存名称
			);

	// 映射缓存区视图 , 得到指向共享内存的指针
	point_buffer = MapViewOfFile(
		point_map,            // 共享内存的句柄
		FILE_MAP_ALL_ACCESS, // 可读写许可
		0,
		0,
		BUF_SIZE
	);
}

CameraData::~CameraData()
{
	LOG(INFO) << "CameraData::~CameraData() ";
	// 解除文件映射，关闭内存映射文件对象句柄
	::UnmapViewOfFile(pcolorBuffer);
	::CloseHandle(hcolorMap);

	::UnmapViewOfFile(pdepthBuffer);
	::CloseHandle(hdepthMap);

	::UnmapViewOfFile(mask_buffer);
	::CloseHandle(mask_map);

	::UnmapViewOfFile(label_buffer);
	::CloseHandle(label_map);

	::UnmapViewOfFile(point_buffer);
	::CloseHandle(point_map);
}

bool CameraData::GetCameraImages(cv::Mat &image_color, cv::Mat& image_depth)
{
	if (false == isOpenCameraMapping)
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
	for (int i = 0; i < image_width_ * image_height_; i++)
	{
		depth.at<ushort>(i / (image_width_), i % (image_width_)) = p2[i];
	}
	image_depth = depth.clone();

	delete[] p1;
	delete[] p2;
	
	return true;
}

bool CameraData::GetMaskAndLabel(cv::Mat & image_mask, cv::Mat& image_mask_collision, std::string& label)
{
	if (false == isOpenMaskMapping)
	{
		LOG(ERROR) << "OpenMaskMapping error! ";
		LOG(ERROR) << "mask_map: " << mask_map;
		LOG(ERROR) << "mask_collision_map: " << mask_collision_map;
		LOG(ERROR) << "label_map: " << label_map;
		return false;
	}

	//image_mask = cv::Mat::ones(cv::Size(image_width_, image_height_), CV_8UC1);
	uchar *p_mask = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_);
	memcpy(p_mask, mask_buffer, sizeof(uchar)*image_height_*image_width_);
	cv::Mat mask(cv::Size(image_width_, image_height_), CV_8UC1);
	for (int i = 0; i < image_width_ * image_height_; i++)
	{
		mask.at<uchar>(i / (image_width_), i % (image_width_))= p_mask[i];//BGR格式
	}
	image_mask = mask.clone();

	image_mask_collision = cv::Mat::ones(cv::Size(image_width_, image_height_), CV_8UC1);
	uchar *p_mask_collision = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_);
	memcpy(p_mask_collision, mask_collision_buffer, sizeof(uchar)*image_height_*image_width_);
	cv::Mat mask_collision(cv::Size(image_width_, image_height_), CV_8UC1);
	for (int i = 0; i < image_width_ * image_height_; i++)
	{
		mask_collision.at<uchar>(i / (image_width_), i % (image_width_))= p_mask_collision[i];//BGR格式
	}
	image_mask_collision = mask_collision.clone();

	char *p_label = (char*)malloc(sizeof(char) * 50);
	memcpy(p_label, label_buffer, sizeof(char) * 50);
	label = p_label;

	delete[] p_mask;
	delete[] p_mask_collision;
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

JsonOutType CameraData::ReadJsonObject(Json::Value json_object, std::string key_name, const char* out_type)
{
	JsonOutType json_out_type;
	if (0 == strcmp(out_type, "string"))
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
	if (0 == strcmp(out_type, "object"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_object = json_object[key_name];
		}
		return json_out_type;
	}
	return json_out_type;

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
	if (0 == strcmp(out_type, "object"))
	{
		if (!json_object[key_name].isNull())
		{
			json_out_type.json_object = json_object[key_name];
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
						d = Depth.at<ushort>(rows, cols);
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
						PointWorld.x = (cols - cx_)*PointWorld.z / fx_;
						PointWorld.y = (rows - cy_)*PointWorld.z / fy_;
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

bool CameraData::DepthtoPointCloud_Collision(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr CloudMask)
{
	int count = 0;
	for(int rows = 0; rows < image_height_; rows++)
    {
			for(int cols = 0; cols < image_width_; cols++ )
			{
					float d = 0;
					if (!Depth.empty())
					{
						//获取深度图中对应点的深度值
						d = Depth.at<ushort>(rows, cols);
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
						PointWorld.x = (cols - cx_)*PointWorld.z / fx_;
						PointWorld.y = (rows - cy_)*PointWorld.z / fy_;

						*((float*)point_buffer + count) = PointWorld.x;
						count++;
						*((float*)point_buffer + count) = PointWorld.y;
						count++;
						*((float*)point_buffer + count) = PointWorld.z;
						count++;

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

ICameraData* GetCameraData()
{
	ICameraData* p_iregistration = new CameraData();
	return p_iregistration;
}
