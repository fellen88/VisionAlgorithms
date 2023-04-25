#ifndef RECOGNITION_3D_CG_H
#define RECOGNITION_3D_CG_H

#include "recognition_3d.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/gpd_camera_data.lib")
#else
#pragma comment (lib, "../X64/Release/gpd_camera_data.lib")
#endif

namespace gpd
{
	typedef pcl::PointXYZRGBA PointType;
	typedef pcl::Normal NormalType;
	typedef pcl::ReferenceFrame RFType;
	typedef pcl::SHOT352 DescriptorType;

	struct CloudStyle
	{
		double r;
		double g;
		double b;
		double size;

		CloudStyle(double r,
			double g,
			double b,
			double size) :
			r(r),
			g(g),
			b(b),
			size(size)
		{
		}
	};

	CloudStyle style_white(255.0, 255.0, 255.0, 2.0);
	CloudStyle style_red(255.0, 0.0, 0.0, 2.0);
	CloudStyle style_green(0.0, 255.0, 0.0, 2.0);
	CloudStyle style_blue(0.0, 0.0, 255.0, 2.0);
	CloudStyle style_cyan(93.0, 200.0, 217.0, 2.0);
	CloudStyle style_indigo(0.0, 255.0, 255.0, 2.0);
	CloudStyle style_violet(255.0, 0.0, 255.0, 8.0);
	CloudStyle style_yellow(255.0, 255.0, 0.0, 8.0);

	class Recognition3DCG : public Recognition3D
	{
	public:

		Recognition3DCG();
		~Recognition3DCG();
		bool SetParameters(const std::string config_file);
		bool Recognize(const PointCloud::Ptr cloud_scene, const std::vector<PointCloud::Ptr> cloud_model,
			Eigen::Matrix4f & transformation, size_t& output_number);
		bool TrainModel(std::vector<PointCloud::Ptr> cloud_models);
	private:
		ICameraData* p_dataprocess_;
	
		std::string model_filename_;
		std::string scene_filename_;
		//Algorithm params 
		bool show_result_;
		bool show_keypoints_;
		bool show_correspondences_;
		bool show_rotated_model_;
		bool use_hough_;
		float sample_3d;
		Eigen::Vector4f subsampling_leaf_size;
		float model_ss_;
		float scene_ss_;
		float rf_rad_;
		float descr_rad_;
		float cg_size_;
		float cg_thresh_;
		int icp_max_iter_;
		float icp_corr_distance_;

		float hv_resolution_;
		float hv_occupancy_grid_resolution_;
		float hv_clutter_reg_;
		float hv_inlier_th_;
		float hv_occlusion_th_;
		float hv_rad_clutter_;
		float hv_regularizer_;
		float hv_rad_normals_;
		bool hv_detect_clutter_;

		};
	}

	#endif
