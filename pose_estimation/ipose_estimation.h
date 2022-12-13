#ifndef I_POSE_ESTIMATION_H_
#define I_POSE_ESTIMATION_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifdef __POSE_ESTIMATION_EXPORT
#define __POSE_ESTIMATION_API __declspec(dllexport)
#else
#define __POSE_ESTIMATION_API __declspec(dllimport)
#endif

class IPoseEstimation
{
public:
	virtual ~IPoseEstimation() = 0;
	virtual bool Algorithm_A(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose) = 0;
	virtual bool Algorithm_B(std::vector<double> object_points, unsigned char view_point, std::vector<double>& object_pose) = 0;
};

__POSE_ESTIMATION_API  IPoseEstimation* GetInstance(char str);

#endif
