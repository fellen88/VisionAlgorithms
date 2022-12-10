#ifndef IPOSEESTIMATIONTION_H
#define IPOSEESTIMATIONTION_H

#include <vector>

class IPoseEstimation
{
	public:
	virtual ~IPoseEstimation() = 0;
	virtual bool Algorithm_A(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose) = 0;
	virtual bool Algorithm_B(std::vector<double> object_points, unsigned char view_point, std::vector<double>& object_pose) = 0;
};

extern "C" __declspec(dllexport) IPoseEstimation* GetInstance(char str);

#endif