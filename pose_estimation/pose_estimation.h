#ifndef POSE_ESTIMATION_H_
#define POSE_ESTIMATION_H_

#include "ipose_estimation.h"
#include"../camera_data/icamera_data.h"
#include"../registration_3d/iregistration_3d.h"
#include"../recognition/irecognition.h"
#include"../segmentation/isegmentation.h"

namespace val
{
	class PoseEstimation : public IPoseEstimation
	{
	public:
		PoseEstimation(unsigned char algorithm_vision, std::string config_file);
		~PoseEstimation();

		void UpdateParameters(std::string config);
		void Init_BinPicking(std::string config);
		bool Compute_BinPicking(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, Eigen::Matrix4f & object_pose);
		bool Compute(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, unsigned char view_point, std::vector<double>& object_pose);

	private:
		std::shared_ptr<ICameraData> p_sensor_;
		IRegistration3D* p_registration_;
		IRegistration3D* p_registration_refine_;
		std::shared_ptr<IRecognition> p_recog_ppf_;
		std::shared_ptr<ISegmentation> p_seg_sac_;
		std::shared_ptr<ISegmentation> p_seg_obb_;
		std::shared_ptr<ISegmentation> p_seg_obb_instance_;
		std::shared_ptr<ISegmentation> p_seg_bound_;
		std::shared_ptr<ISegmentation> p_seg_eucli_;
		std::shared_ptr<ISegmentation> p_seg_eucli_refine_;

		bool pose_flag;
		bool debug_visualization;
		bool sensor_offline;
		bool part_refine;
		std::string instance_seg;
		bool edge_normal;
		float sample_3d;
		float normal_search_radius;
		float curvature_thredhold;

		float DertaX;
		float DertaY;
		float DertaZ;
		float DertaRX;
		float DertaRY;
		float DertaRZ;


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

		std::string seg_sac_config;
		std::string seg_obb_config;
		std::string seg_obb_instance_config;
		std::string seg_bound_config;
		std::string seg_eucli_config;
		std::string seg_eucli_refine_config;
		std::string ppf_config;
		std::string lmicp_config;
		std::string lmicp_refine_config;
		std::vector<PointCloud::Ptr> cloud_models;

		std::string ModelFileName;
		std::string ModelPartFileName;
		std::string ScanFileName;

		unsigned char grasp_method;
	};
}

#endif
