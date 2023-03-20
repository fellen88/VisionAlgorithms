#ifndef REGISTRATION_H
#define REGISTRATION_H  

#include "stdafx.h"
#include "features.h"
#include "iregistration_3d.h"
#include "../camera_data/camera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/vision_camera_data.lib")
#endif
//Åä×¼
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Registration3D : public IRegistration3D, public Features
{
  public:

  bool DEBUG_VISUALIZER;

	Eigen::Matrix4f final_transform;
	Eigen::Matrix4f sac_transform;
	Eigen::Matrix4f icp_transform;
	PointCloud::Ptr sac_output;
	PointCloud::Ptr icp_output;

	ICameraData* p_regist_cameradata_;
	float sample_3d;
	bool debug_visualization;
	float max_correspondence_distance;

  Registration3D();
	bool SetParameters(const std::string config_file);
  void SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, float downsample, bool debug_v);
  void LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample, bool debug_v);
};

#endif