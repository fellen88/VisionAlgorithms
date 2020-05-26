#ifndef REGISTRATION_H
#define REGISTRATION_H  

#include "stdafx.h"
#include "features.h"
#include "iregistration_3d.h"
//Åä×¼
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Registration3D : public IRegistration3D, public Features
{
  public:

  bool DEBUG_VISUALIZER;

  void SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, bool downsample);
  void LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);
};

#endif