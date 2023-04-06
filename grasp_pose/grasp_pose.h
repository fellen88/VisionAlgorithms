#ifndef GRASP_POSE_H_
#define GRASP_POSE_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifdef GRASPPOSE_EXPORTS
#define GRASPPOSE_API __declspec(dllexport)
#else
#define GRASPPOSE_API __declspec(dllimport)
#endif

namespace val  //vision algorithm library
{
	class GraspPose 
	{
	public:
		virtual ~GraspPose() = 0;

		virtual void SetInputPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points) = 0;
		//virtual void SetInputColorImage() = 0;
		//virtual void SetInputDepthImage() = 0;

		virtual void GetGraspPose(std::vector<double>* object_pose) = 0;
	};
}

GRASPPOSE_API  val::GraspPose * GetModelBasedPtr(std::string config_file);

#endif
