#ifndef GP_MODEL_BASED_H_
#define GP_MODEL_BASED_H_

#include "grasp_pose.h"

#include"../camera_data/icamera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif

#include "../pose_estimation/ipose_estimation.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_pose_estimation.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_pose_estimation.lib")
#endif

namespace gpd  //grasp pose detector
{
	class GraspPoseModelBased : public GraspPose
	{
	public:
		GraspPoseModelBased(std::string config_file);
		~GraspPoseModelBased();

		void SetInputPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points);
		void SetInputMask(const cv::Mat& mask);
		void SetInputDepth(const cv::Mat& depth);
		void GetGraspPose(std::vector<double>* object_pose);

	private:
		pcl::PointCloud<pcl::PointXYZRGBNormal> object_point_cloud;
		Eigen::Matrix4f object_pose_matrix;

		unsigned char picking_method;
		Eigen::Vector3f object_eulerangle;
		std::shared_ptr<ICameraData> p_photoneo_;
		std::shared_ptr<IPoseEstimation> p_pose_estimation_;

	};
}

#endif