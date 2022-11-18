// segmentation.cpp : 定义 DLL 应用程序的导出函数。
//
#include "stdafx.h"
#include "segmentation_sac.h"

SegmentationSAC::SegmentationSAC(std::string config_path)
{
	p_seg_cameradata_ = GetCameraData();
	JsonOutType json_reader;
	distance_threshold = 0.005;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_path, "DistanceThreshold", "float");
	if (json_reader.success)
		distance_threshold = json_reader.json_float;
}

SegmentationSAC::~SegmentationSAC()
{
}

bool SegmentationSAC::segment(PointCloud::Ptr cloud_scene)
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	//TODO:CONFIG
	seg.setDistanceThreshold(distance_threshold);
	extract.setNegative(true);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	const auto nr_points = cloud_scene->size();
	while (cloud_scene->size() > 0.3 * nr_points) {
		seg.setInputCloud(cloud_scene);
		seg.segment(*inliers, *coefficients);
		PCL_INFO("Plane inliers: %zu\n", static_cast<std::size_t>(inliers->indices.size()));
		if (inliers->indices.size() < 50000)
			break;

		extract.setInputCloud(cloud_scene);
		extract.setIndices(inliers);
		extract.filter(*cloud_scene);
	}
	return true;
}

ISegmentation* GetSegmentationSAC(std::string config_path)
{
	ISegmentation* p_isegmentation_ = new SegmentationSAC(config_path);
	return p_isegmentation_;
}
