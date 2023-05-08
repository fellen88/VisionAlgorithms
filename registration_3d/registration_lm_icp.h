#ifndef REGISTRATION_H
#define REGISTRATION_H  

#include "stdafx.h"
#include "iregistration_3d.h"
#include "../camera_data/camera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif

//��׼
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

// 定义新的点表达方�?< x, y, z, curvature > 坐标+曲率
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT> //继承关系
{
	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
	MyPointRepresentation()
	{
		//指定维数
		nr_dimensions_ = 4;
	}
	//重载函数copyToFloatArray，以定义�?己的特征向量
	virtual void copyToFloatArray(const PointNormalT &p, float * out) const
	{
		//< x, y, z, curvature > 坐标xyz和曲�?
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

class Registration3D : public IRegistration3D
{
  public:

  bool DEBUG_VISUALIZER;

	Eigen::Matrix4f final_transform;
	Eigen::Matrix4f icp_transform;
	PointCloud::Ptr icp_output;

	ICameraData* p_regist_cameradata_;
	float sample_3d;
	int max_interations;
	bool debug_visualization;
	float max_correspondence_distance;

  Registration3D();
	bool SetParameters(const std::string config_file);
  void Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform);
};

#endif