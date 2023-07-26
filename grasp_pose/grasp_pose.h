#ifndef GRASP_POSE_H_
#define GRASP_POSE_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

#ifdef GRASPPOSE_EXPORTS
#define GRASPPOSE_API __declspec(dllexport)
#else
#define GRASPPOSE_API __declspec(dllimport)
#endif

namespace  gpd  //grasp pose detector
{
	class GraspPose 
	{
	public:
		virtual ~GraspPose() = 0;

		virtual void SetInputPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points) = 0;
		virtual void SetInputMask(const cv::Mat& mask) = 0;
		virtual void SetInputDepth(const cv::Mat& depth) = 0;
		virtual void GetGraspPose(std::vector<double>* object_pose) = 0;
	};
}

GRASPPOSE_API  gpd::GraspPose * GetModelBasedPtr(std::string config_file);

#endif
