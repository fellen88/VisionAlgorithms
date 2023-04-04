#ifndef BIN_PICKING_H_
#define BIN_PICKING_H_

#include "ibin_picking.h"

#include"../camera_data/icamera_data.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/vision_camera_data.lib")
#endif

#include "../pose_estimation/ipose_estimation.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_pose_estimation.lib")
#else
#pragma comment (lib, "../X64/Release/vision_pose_estimation.lib")
#endif

namespace val  //vision algorithm library
{
	class BinPicking : public IBinPicking
	{
	public:
		BinPicking(unsigned char algorithm, std::string config_file);
		~BinPicking();

		bool Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, std::vector<double>* object_pose);

	private:
		unsigned char grasp_method;
		Eigen::Vector3f object_eulerangle;
		std::shared_ptr<ICameraData> p_photoneo_;
		std::shared_ptr<IPoseEstimation> p_pose_estimation_;

	};
}

#endif