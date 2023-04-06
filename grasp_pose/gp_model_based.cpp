
#include "pch.h"
#include "framework.h"
#include "gp_model_based.h"

// Google glog
//#include "glog/logging.h"

using namespace val;

val::GraspPoseModelBased::GraspPoseModelBased(std::string config_file)
{
	p_photoneo_.reset(GetCameraData());
	p_pose_estimation_.reset(GetInstance(config_file));
}

val::GraspPoseModelBased::~GraspPoseModelBased()
{
}

void val::GraspPoseModelBased::SetInputPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points)
{
	pcl::copyPointCloud(object_points, object_point_cloud);
}

void val::GraspPoseModelBased::GetGraspPose(std::vector<double>* object_pose)
{
	p_pose_estimation_->Compute(object_point_cloud, object_pose_matrix);

	p_photoneo_->Matrix2EulerAngle(object_pose_matrix, object_eulerangle);

	object_pose->clear();
	object_pose->push_back(object_pose_matrix(0, 3));
	object_pose->push_back(object_pose_matrix(1, 3));
	object_pose->push_back(object_pose_matrix(2, 3));
	object_pose->push_back(object_eulerangle[2] * 180 / M_PI);
	object_pose->push_back(object_eulerangle[1] * 180 / M_PI);
	object_pose->push_back(object_eulerangle[0] * 180 / M_PI);
}

val::GraspPose * GetModelBasedPtr(std::string config_file)
{
	GraspPose* p_grasp_pose_ = new GraspPoseModelBased(config_file);
	return p_grasp_pose_;
}
