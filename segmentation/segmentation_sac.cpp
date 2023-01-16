// segmentation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#include "segmentation_sac.h"

SegmentationSAC::SegmentationSAC()
{
	p_seg_cameradata_ = GetCameraData();
}

SegmentationSAC::~SegmentationSAC()
{
	if (p_seg_cameradata_ != nullptr)
	{
		delete p_seg_cameradata_;
	}
}

bool SegmentationSAC::segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg)
{
	//TODO:TEST
	p_seg_cameradata_->DownSample(cloud_scene, subsampling_leaf_size);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(distance_threshold);
	extract.setNegative(true);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	const auto nr_points = cloud_scene->size();
	while (cloud_scene->size() > 0.3 * nr_points) {
		seg.setInputCloud(cloud_scene);
		seg.segment(*inliers, *coefficients);
		PCL_INFO("Plane inliers: %zu\n", static_cast<std::size_t>(inliers->indices.size()));
		if (inliers->indices.size() < 5000)
			break;

		extract.setInputCloud(cloud_scene);
		extract.setIndices(inliers);
		extract.filter(*cloud_scene);
	}
	return true;
}

bool SegmentationSAC::SetParameters(const std::string config_file)
{
	distance_threshold = 0.005;
	JsonOutType json_reader;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "DistanceThreshold", "float");
	if (json_reader.success)
		distance_threshold = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "Sample3D", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	else
		return false;
	subsampling_leaf_size = Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f);

	return true;
}

ISegmentation* GetSegmentationSAC()
{
	ISegmentation* p_isegmentation_ = new SegmentationSAC();
	return p_isegmentation_;
}
