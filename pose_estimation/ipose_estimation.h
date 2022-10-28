#ifndef IPOSEESTIMATIONTION_H
#define IPOSEESTIMATIONTION_H

//pcl头文件
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//pcl类型名简化
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::VFHSignature308> vfhFeathure;

class IPoseEstimation
{
	public:
	virtual bool LoadObjectModel() = 0;
	virtual bool Algorithm_Test() = 0;
	virtual bool Algorithm_A(PointCloud::Ptr cloud_scene, unsigned char viewpoint, Eigen::Matrix4f pose_object) = 0;
	virtual bool Algorithm_B() = 0;
	virtual bool Algorithm_C() = 0;
};

extern "C" __declspec(dllexport) IPoseEstimation* GetInstance(char str);

#endif