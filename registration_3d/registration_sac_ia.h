#ifndef REGISTRATION_SAC_IA_H_
#define REGISTRATION_SAC_IA_H_  

#include "features.h"
#include "iregistration_3d.h"
#include "../camera_data/camera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif

//Åä×¼
#include <pcl/registration/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class RegistrationSACIA : public IRegistration3D, public Features
{
public:

	bool DEBUG_VISUALIZER;

	Eigen::Matrix4f final_transform;
	Eigen::Matrix4f sac_transform;
	PointCloud::Ptr sac_output;

	ICameraData* p_regist_cameradata_;
	float sample_3d;
	bool debug_visualization;

	RegistrationSACIA();
	bool SetParameters(const std::string config_file);
	void Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform);
};

#endif