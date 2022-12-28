#ifndef POSE_ESTIMATION_H_
#define POSE_ESTIMATION_H_

#include "ipose_estimation.h"
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"
#include"../recognition/irecognition.h"
#include"../segmentation/isegmentation.h"

class PoseEstimation: public IPoseEstimation
{
public:
	PoseEstimation(const char algorithm_version);
	~PoseEstimation();

	void SetParameters_A();
	bool Algorithm_A(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char viewpoint, std::vector<double>& object_pose);
	void SetParameters_B();
	bool Algorithm_B(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose);

private:
	ICameraData* p_sensor_;
	IRegistration3D* p_registration_;
	IRecognition* p_recognition_;
	ISegmentation* p_seg_sac_;
	ISegmentation* p_seg_obb_;

	bool pose_flag;
	bool debug_visualization;
	bool sensor_offline;
	float sample_3d;
	
	cv::Mat object_depth;
	cv::Mat object_color;
	cv::Mat object_mask;
	std::string object_label;

	PointCloud::Ptr object_model;
	PointCloud::Ptr object_scan;
	PointCloud::Ptr object_output;
	PointCloud::Ptr sac_output;
	PointCloud::Ptr obb_output;
	PointCloudWithNormals::Ptr object_scene_normal;

	Eigen::Matrix4f object_transform;
	Eigen::Matrix4f sac_transform;
	Eigen::Vector3f object_eulerangle;

	std::string seg_sac_config;
	std::string seg_obb_config;
	std::string ppf_config;
	std::string lmicp_config;
	std::vector<PointCloud::Ptr> cloud_models;
	
	std::string ModelFileName;
  std::string ScanFileName;
};

#endif
