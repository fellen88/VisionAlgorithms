// camera_data.cpp : 定义 DLL 应用程序的导出函数。

#include "stdafx.h"
#include <fstream>
#include "camera_data.h"

#define COLORMEMORYNAME L"color"
#define DEPTHMEMORYNAME L"depth"

CameraData::CameraData()
{
	//LOG(INFO) << "CameraData::CameraData() ";
	
	pcolorBuffer = nullptr;                                   // 共享内存指针
	pdepthBuffer = nullptr;                                   // 共享内存指针
	mask_buffer = nullptr;                                   // 共享内存指针
	label_buffer = nullptr;                                   // 共享内存指针
	isOpenCameraMapping = false;
	isOpenMaskMapping = false;

	hcolorMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)COLORMEMORYNAME);
	hdepthMap = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)DEPTHMEMORYNAME);
	mask_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"mask");
	label_map = ::OpenFileMapping(FILE_MAP_ALL_ACCESS, 0, (LPCWSTR)L"mask_label");
	if (NULL != hcolorMap && NULL != hdepthMap)
	{
		//打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		pcolorBuffer = ::MapViewOfFile(hcolorMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		pdepthBuffer = ::MapViewOfFile(hdepthMap, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		isOpenCameraMapping = true;
		LOG(INFO) << "OpenCameraMapping ok! ";
	}
	if (NULL != mask_map && NULL != label_map)
	{
		//打开成功，映射对象的一个视图，得到指向共享内存的指针，显示出里面的数据
		mask_buffer = ::MapViewOfFile(mask_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    label_buffer = ::MapViewOfFile(label_map, FILE_MAP_ALL_ACCESS, 0, 0, 0);
		isOpenMaskMapping = true;
		LOG(INFO) << "OpenMaskMapping ok! ";
	}
}

CameraData::~CameraData()
{
	// 解除文件映射，关闭内存映射文件对象句柄
	::UnmapViewOfFile(pcolorBuffer);
	::CloseHandle(hcolorMap);

	::UnmapViewOfFile(pdepthBuffer);
	::CloseHandle(hdepthMap);

	::UnmapViewOfFile(mask_buffer);
	::CloseHandle(mask_map);

	::UnmapViewOfFile(label_buffer);
	::CloseHandle(label_map);
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

bool CameraData::GetMaskAndLabel(cv::Mat & image_mask, std::string label)
{
	if (false == isOpenMaskMapping)
	{
		LOG(ERROR) << "OpenMaskMapping error! ";
		LOG(ERROR) << "mask_map: " << mask_map;
		LOG(ERROR) << "label_map: " << label_map;
		return false;
	}

	image_mask = cv::Mat::ones(cv::Size(image_width_, image_height_), CV_8UC1);
	uchar *p_mask = (uchar*)malloc(sizeof(uchar)*image_height_*image_width_);
	memcpy(p_mask, mask_buffer, sizeof(uchar)*image_height_*image_width_);
	cv::Mat mask(cv::Size(image_width_, image_height_), CV_8UC1);
	for (int i = 0; i < image_width_ * image_height_; i++)
	{
		mask.at<uchar>(i / (image_width_), i % (image_width_))= p_mask[i];//BGR格式
	}

	image_mask = mask.clone();

	char *p_label = (char*)malloc(sizeof(char) * 50);
	label = p_label;

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

bool CameraData::LoadPLY(std::string file_name, PointCloud::Ptr object_model)
{
	pcl::PLYReader reader;
	if (0 == reader.read<pcl::PointXYZ>(file_name, *object_model))
		return true;
	else
	return false;
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
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(window_name));
		viewer->addPointCloud(pointcloud);
		viewer->addCoordinateSystem(0.1);
		//viewer->initCameraParameters();
		viewer->spin ();
	}
	else
	{
		LOG(ERROR) << window_name << " is a nullptr pointlcoud!";
	}
}

void CameraData::ShowPointCloud_NonBlocking(const PointCloud::Ptr pointcloud, size_t spin_time,
	std::string cloudname, 	unsigned char r, unsigned char g, unsigned char b)
{
	if (nullptr != pointcloud)
	{
		if (nullptr == viewer_nonblock)
		{
			viewer_nonblock.reset(new pcl::visualization::PCLVisualizer("viewer_nonblock"));
		}
		//viewer_nonblock->removePointCloud(cloudname); //根据给定的ID，从屏幕中去除一个点云。参数是ID
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloudcolor(pointcloud, r, g, b); //设置点云显示颜色
		viewer_nonblock->addPointCloud(pointcloud, cloudcolor, cloudname);
		viewer_nonblock->spinOnce(spin_time);
	}
	else
	{
		LOG(ERROR) << "show pointcloud nonblocking is a nullptr pointlcoud!";
	}
}

void CameraData::ShowPose(const PointCloud::Ptr pointcloud, Eigen::Matrix4f &pose_matrix, std::string window_name)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer(new pcl::visualization::PCLVisualizer(window_name));
	pcl_viewer->setCameraPosition(-0.3, 0, -0.3, 0, 0, 1, -1, 0, 0); //视点 方向 上方向
	if (nullptr != pointcloud)
	{
		pcl_viewer->addPointCloud(pointcloud);
		//pcl_viewer->addCoordinateSystem(0.1);
	}
	PointCloud::Ptr axis(new PointCloud);
	PointCloud::Ptr pose(new PointCloud);
	axis->points.push_back(pcl::PointXYZ(0, 0, 0));
	axis->points.push_back(pcl::PointXYZ(0.1, 0, 0));
	axis->points.push_back(pcl::PointXYZ(0, 0.1, 0));
	axis->points.push_back(pcl::PointXYZ(0, 0, 0.1));
	pcl::transformPointCloud(*axis, *pose, pose_matrix);

	pcl_viewer->addLine(pose->points[0], pose->points[1], 1.0f, 0.0f, 0.0f, "x vector");
	pcl_viewer->addLine(pose->points[0], pose->points[2], 0.0f, 1.0f, 0.0f, "y vector");
	pcl_viewer->addLine(pose->points[0], pose->points[3], 0.0f, 0.0f, 1.0f, "z vector");
	pcl_viewer->spin();
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

JsonOutType CameraData::ReadJsonFile(const std::string file_name, const std::string key_name, const char* out_type)
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

JsonOutType CameraData::ReadJsonString(const std::string json_string, const std::string key_name, const char * out_type)
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
 
bool CameraData::SetParameters(const std::string JsonFilePath)
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

void CameraData::ConvertPointsMMtoM(PointCloud::Ptr pointcloud)
{
	for (int i = 0; i < pointcloud->size(); i++)
	{
		pointcloud->points[i].x = pointcloud->points[i].x / 1000;
		pointcloud->points[i].y = pointcloud->points[i].y / 1000;
		pointcloud->points[i].z = pointcloud->points[i].z / 1000;
	}
}
void CameraData::Matrix2EulerAngle(Eigen::Matrix4f& matrix, Eigen::Vector3f& euler_angle)
{
	Eigen::Matrix3f Rotation_matrix;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			Rotation_matrix(i, j) = matrix(i, j);
		}
	euler_angle = Rotation_matrix.eulerAngles(2, 1, 0);//顺序Z, Y, X
}

void CameraData::VectorPointstoPCL(std::vector<double> points_normals, PointCloud::Ptr pointcloud, PointCloudWithNormals::Ptr pointcloud_normals)
{
	pointcloud->width = points_normals.size() / 6;
	pointcloud->height = 1;
	pointcloud->is_dense = false;
	pointcloud->points.resize(points_normals.size() / 6);
	//pointcloud_normals->width = points_normals.size() / 6;
	//pointcloud_normals->height = 1;
	//pointcloud_normals->is_dense = false;
	//pointcloud_normals->resize(points_normals.size()/6);

	LOG(INFO) << "PointCloud Size: " << pointcloud->size();
	LOG(INFO) << "Points_Normals Size: "<< points_normals.size();

	for (size_t i = 0; i < pointcloud->size(); i++)
	{
		pointcloud->points[i].x = points_normals[i*6]/1000;
		pointcloud->points[i].y = points_normals[i*6 + 1]/1000;
		pointcloud->points[i].z = points_normals[i*6 + 2]/1000;
	
	/*	pointcloud_normals->points[i].x = points_normals[j]/1000;
		pointcloud_normals->points[i].y = points_normals[j + 1]/1000;
		pointcloud_normals->points[i].z = points_normals[j + 2]/1000;
		pointcloud_normals->points[i].normal_x = points_normals[j + 3];
		pointcloud_normals->points[i].normal_y = points_normals[j + 4];
		pointcloud_normals->points[i].normal_z = points_normals[j + 5];*/
	}

	LOG(INFO) << "Vector to PCL OK ";
}

void CameraData::DownSample(const PointCloud::Ptr cloud, const Eigen::Vector4f subsampling_leaf_size)
{
	if (subsampling_leaf_size[1] < 0.000001)
		return;
	pcl::VoxelGrid<pcl::PointXYZ> subsampling_filter;
	subsampling_filter.setInputCloud(cloud);
	subsampling_filter.setLeafSize(subsampling_leaf_size);
	subsampling_filter.filter(*cloud);
}

//UniformSampling
void CameraData::UniformSampling(const PointCloud::Ptr cloud, const float search_radius, PointCloud::Ptr cloud_keypoints)
{
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling_filter;
	uniform_sampling_filter.setInputCloud(cloud);
	uniform_sampling_filter.setRadiusSearch(search_radius);
	uniform_sampling_filter.filter(*cloud_keypoints);
}

void CameraData::CalculateNormals(const PointCloud::Ptr cloud, const float search_radius, PointCloudWithNormals::Ptr cloud_normals)
{
	PointNormals::Ptr normals(new PointNormals());
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
	normal_estimation_filter.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation_filter.setSearchMethod(search_tree);
	normal_estimation_filter.setRadiusSearch(search_radius);
	normal_estimation_filter.compute(*normals);

	concatenateFields(*cloud, *normals, *cloud_normals);
}

void CameraData::CurvaturesEstimation(const PointCloud::Ptr cloud, const float search_radius, PointCloudWithNormals::Ptr cloud_normals, PointCloudWithCurvatures::Ptr cloud_curvatures)
{
	//计算法向
	PointNormals::Ptr normals(new PointNormals());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
	normal_estimation_filter.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation_filter.setSearchMethod(search_tree);
	normal_estimation_filter.setRadiusSearch(search_radius);
	normal_estimation_filter.compute(*normals);
	//concatenateFields(*cloud, *normals, *cloud_normals);
	//计算曲率-------------------------------------------------------------------------------------
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>pc;
	pc.setInputCloud(cloud);
	pc.setInputNormals(normals);
	pc.setSearchMethod(search_tree);
	//pc.setRadiusSearch(search_radius);
	pc.setKSearch(5);
	pc.compute(*cloud_curvatures);

	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>(cloud, normals, cloud_curvatures, 5, 0.001, "cloud_curvatures");
	viewer.spin();
}

void CameraData::RemoveInvalidPoints(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out)
{
	LOG(INFO) << "points number before remove invalid points:" << cloud_in->size();
	//LOG(INFO) << "is dense :" << cloud_in->is_dense;
	cloud_in->is_dense = false;
	std::vector<int> mapping;
	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, mapping);

	cloud_out->points.resize(cloud_in->points.size());
	size_t j = 0;
	for (size_t i = 0; i < cloud_in->points.size(); i++)
	{
		if (cloud_in->points[i].z > 0.3 && cloud_in->points[i].z < 0.6)
		{
			cloud_out->points[j] = cloud_in->points[i];
			j++;
		}
	}
	cloud_out->points.resize(j);
	LOG(INFO) << "points number after remove invalid points:" << cloud_out->size();
}

void CameraData::EdgeWithNormal(const PointCloud::Ptr cloud_in, const float search_radius, const float curvature_thredhold, PointCloudWithNormals::Ptr cloud_normals, PointCloud::Ptr cloud_out)
{
	CalculateNormals(cloud_in, search_radius, cloud_normals);
	cloud_out->points.resize(cloud_normals->points.size());
	size_t j = 0;
	for (size_t i = 0; i < cloud_normals->size(); i++)
	{
		if (cloud_normals->points[i].curvature > curvature_thredhold)
		{
			cloud_out->points[j].x = cloud_normals->points[i].x;
			cloud_out->points[j].y = cloud_normals->points[i].y;
			cloud_out->points[j].z = cloud_normals->points[i].z;
			j++;
		}
	}
	cloud_out->points.resize(j);
}

//获取路径名中的子目录:strPath为路径名，strSubPath为输出的子目录，
//nSearch为从尾向前检索的级别(默认为1级)
bool CameraData::GetSubPath(const std::string& strPath, std::string& strSubPath, int nSearch)
{
	if (-1 == nSearch || strPath.empty())
		return false;
	std::string::size_type nPos1 = std::string::npos;
	nPos1 = strPath.find_last_of("\\");
	if (nPos1 != -1)
	{
		strSubPath = strPath.substr(nPos1 + 1, strPath.length() - nPos1);
		int nNewSearch = nSearch > 1 ? nSearch - 1 : -1;
		GetSubPath(strPath.substr(0, nPos1), strSubPath, nNewSearch);
	}
	return true;
}
bool CameraData::GetPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, PointCloud::Ptr object_scan)
{
	//sensor offine/online mode
	if (sensor_offline)
	{
		if (false == LoadPLY(ScanFileName, object_scan))
		{
			LOG(ERROR) << "LoadPointCloud Error!";
			return false;
		}
		else
			LOG(INFO) << "Load PLY on sensor off mode";
		ConvertPointsMMtoM(object_scan);
	}
	else
	{
		if (object_points.size() < 1000)
		{
			LOG(ERROR) << "input pointcloud size < 1000";
			return false;
		}
		else
		{
			LOG(INFO) << "Read pointcloud data on sensor on mode";
			pcl::copyPointCloud(object_points, *object_scan);
			ConvertPointsMMtoM(object_scan);
		}
	}

}

ICameraData* GetCameraData()
{
	ICameraData* p_icamera_data_ = new CameraData();
	return p_icamera_data_;
}
