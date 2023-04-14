#ifndef POSE_ESTIMATION_H_
#define POSE_ESTIMATION_H_

#include "ipose_estimation.h"
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"
#include"../recognition/irecognition.h"
#include"../segmentation/isegmentation.h"

namespace gpd
{
	class PoseEstimation : public IPoseEstimation
	{
	public:
		PoseEstimation(std::string config_file);
		~PoseEstimation();

		void EulerAngle2Matrix(Eigen::Vector3f & euler_angle, Eigen::Matrix4f & transformation_matrix);
		void UpdateParameters(std::string config);
		void Init_Compute(std::string config);
		bool Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f & object_pose);

	private:
		std::shared_ptr<ICameraData> p_sensor_;
		std::shared_ptr< IRegistration3D> p_regist_lmicp_;
		std::shared_ptr< IRegistration3D> p_refine_regist_lmicp_;
		std::shared_ptr< IRegistration3D> p_regist_sacia_;
		std::shared_ptr< IRegistration3D> p_refine_regist_sacia_;
		std::shared_ptr<IRecognition> p_recog_ppf_;
		std::shared_ptr<ISegmentation> p_seg_sac_;
		std::shared_ptr<ISegmentation> p_refine_seg_obb_;
		std::shared_ptr<ISegmentation> p_seg_obb_instance_;
		std::shared_ptr<ISegmentation> p_refine_seg_bound_;
		std::shared_ptr<ISegmentation> p_instance_seg_bound_;
		std::shared_ptr<ISegmentation> p_seg_eucli_;
		std::shared_ptr<ISegmentation> p_refine_seg_eucli_;

		bool debug_visualization;
		bool sensor_offline;
		int refine_model_num;
		std::string instance_seg;
		std::string instance_keypoint;
		float sample_3d;
		bool use_model_pose;
		float X, Y, Z, RX, RY, RZ;

		std::string project_file;
		std::string object_file;
		std::string config_file;
		std::string config_path;

		cv::Mat object_depth;
		cv::Mat object_color;
		cv::Mat object_mask;
		std::string object_label;

		PointCloud::Ptr object_model;
		PointCloud::Ptr object_model_part1;
		PointCloud::Ptr object_model_part2;
		PointCloud::Ptr object_scan;
		PointCloud::Ptr object_output;
		PointCloud::Ptr object_model_segsac;
		PointCloud::Ptr object_scan_segsac;
		PointCloud::Ptr object_model_downsample;
		PointCloud::Ptr object_scan_downsample;
		PointCloud::Ptr object_model_segeucli;
		PointCloud::Ptr object_scan_segeucli;
		PointCloud::Ptr object_model_instance;
		PointCloud::Ptr object_scan_instance;
		PointCloud::Ptr object_model_edge;
		PointCloud::Ptr object_scene_edge;
		PointCloud::Ptr object_model_preprocess;
		PointCloud::Ptr object_scan_preprocess;
		PointCloud::Ptr sac_output;
		PointCloud::Ptr obb_output;
		PointCloud::Ptr obb_part1;
		PointCloud::Ptr obb_part2;
		PointCloudWithNormals::Ptr object_scene_normal;
		PointCloudWithNormals::Ptr object_model_normal;
		PointCloudWithCurvatures::Ptr object_scene_curvature;
		PointCloudWithCurvatures::Ptr object_model_curvature;

		Eigen::Matrix4f object_transform;
		Eigen::Matrix4f object_transform_init;
		Eigen::Matrix4f object_transform_refine;
		Eigen::Matrix4f sac_transform;
		Eigen::Vector3f object_eulerangle;


		std::string	instance_recog_ppf_config;
		std::string instance_seg_obb_config;
		std::string keypoint_seg_boundary_config;
		std::string refine_regist_lmicp_config;
		std::string refine_regist_sacia_config;
		std::string refine_seg_boundary_config;
		std::string refine_seg_euclidean_config;
		std::string refine_seg_obb_config;
		std::string regist_lmicp_config;
		std::string regist_sacia_config;
		std::string seg_euclidean_config;
		std::string seg_sac_config;
		

		std::vector<PointCloud::Ptr> cloud_models;

		std::string ModelFileName;
		std::string ScanFileName;
	};
}

#endif
