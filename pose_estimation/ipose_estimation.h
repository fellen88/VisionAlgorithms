#ifndef I_POSE_ESTIMATION_H_
#define I_POSE_ESTIMATION_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

#ifdef POSE_ESTIMATION_EXPORTS
#define POSE_ESTIMATION_API __declspec(dllexport)
#else
#define POSE_ESTIMATION_API __declspec(dllimport)
#endif

namespace gpd
{
	class IPoseEstimation
	{
	public:
		virtual ~IPoseEstimation() = 0;
		virtual void SetInputMask(const cv::Mat& mask) = 0;
		virtual void SetInputDepth(const cv::Mat& depth) = 0;
		virtual bool Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f & object_pose) = 0;
	};
}
POSE_ESTIMATION_API  gpd::IPoseEstimation* GetInstance(std::string config_file);

#endif
