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
		std::shared_ptr<IRecognition> p_recog_cg_;
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
		std::string instance_method;
		std::string refine_registration;
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

		std::vector<PointCloud::Ptr> cloud_models;
		PointCloud::Ptr object_model;
		PointCloud::Ptr object_model_transformed;
		PointCloud::Ptr model_refine_a;
		PointCloud::Ptr model_refine_b;
		PointCloud::Ptr cloud_scene;
		PointCloud::Ptr sac_output;
		PointCloud::Ptr icp_output;
		PointCloud::Ptr object_model_instance;
		PointCloud::Ptr object_scan_instance;
		//refine
		PointCloud::Ptr model_refine_a_transformed;
		PointCloud::Ptr model_refine_b_transformed;
		PointCloud::Ptr obb_a;
		PointCloud::Ptr obb_b;
		PointCloud::Ptr boundary_obb_a;
		PointCloud::Ptr boundary_obb_b;
		PointCloud::Ptr boundary_refine_a;
		PointCloud::Ptr boundary_refine_b;
		PointCloud::Ptr euclidean_obb_a;
		PointCloud::Ptr euclidean_obb_b;
		PointCloud::Ptr euclidean_refine_a;
		PointCloud::Ptr euclidean_refine_b;
		PointCloud::Ptr euclidean_obb;
		PointCloud::Ptr euclidean_refine;

		Eigen::Matrix4f object_transform;
		Eigen::Matrix4f object_transform_init;
		Eigen::Matrix4f object_transform_refine;
		Eigen::Matrix4f sac_transform;
		Eigen::Vector3f object_eulerangle;
		size_t object_instance_number;
		size_t grasp_count;


		std::string refine_regist_lmicp_config;
		std::string refine_regist_sacia_config;
		std::string refine_seg_boundary_config;
		std::string refine_seg_euclidean_config;
		std::string refine_seg_obb_config;
		std::string instance_cg_config;
		
		std::string ModelFileName;
		std::string ScanFileName;
	};
}

#endif
