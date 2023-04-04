
#include "pch.h"
#include "framework.h"
#include "bin_picking.h"

// Google glog
#include "glog/logging.h"

using namespace val;

BinPicking::BinPicking(unsigned char algorithm, std::string config_file)
{
	switch (algorithm)
	{
	case ModelBased:
		p_pose_estimation_.reset(GetInstance(val::IPoseEstimation::BinPicking, config_file));
		break;

	case ModelBasedDL:
		break;

	case ModelFree:
		break;

	default:
		LOG(ERROR) << "Picking Method Error !";
		break;
	}
}

val::BinPicking::~BinPicking()
{
}

bool val::BinPicking::Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, std::vector<double>* object_pose)
{
	Eigen::Matrix4f object_pose_matrix;

	switch (grasp_method)
	{
	case ModelBased:
		p_pose_estimation_->Compute_ModelBased(object_points, object_pose_matrix);
		break;

	case ModelBasedDL:
		break;

	case ModelFree:
		break;

	default:
		LOG(ERROR) << "Picking Method Error !";
		break;
	}

	p_photoneo_->Matrix2EulerAngle(object_pose_matrix, object_eulerangle);

	object_pose->clear();
	object_pose->push_back(object_pose_matrix(0, 3));
	object_pose->push_back(object_pose_matrix(1, 3));
	object_pose->push_back(object_pose_matrix(2, 3));
	object_pose->push_back(object_eulerangle[2] * 180 / M_PI);
	object_pose->push_back(object_eulerangle[1] * 180 / M_PI);
	object_pose->push_back(object_eulerangle[0] * 180 / M_PI);

	return true;
}

IBinPicking * GetPtr(char algothrim_version, std::string config_file)
{
	IBinPicking* p_bin_picking_ = new BinPicking(algothrim_version, config_file);
	return p_bin_picking_;
}

