#ifndef I_POSE_ESTIMATION_H_
#define I_POSE_ESTIMATION_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifdef POSE_ESTIMATION_EXPORTS
#define POSE_ESTIMATION_API __declspec(dllexport)
#else
#define POSE_ESTIMATION_API __declspec(dllimport)
#endif

namespace val  //vision algorithm library
{
	class IPoseEstimation
	{
	public:
		enum GraspName
		{
			AccuracyGrasp,
			BinPicking,
		};
		virtual ~IPoseEstimation() = 0;
		virtual bool Compute_ModelBased(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f & object_pose) = 0;
		virtual bool Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>* object_pose) = 0;
	};
}
POSE_ESTIMATION_API  val::IPoseEstimation* GetInstance(char algorithm_vision, std::string config_file);

#endif
