#include "stdafx.h"
#include "segmentation_obb.h"
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>


SegmentationOBB::SegmentationOBB()
{
	p_obb_cameradata_ = GetCameraData();
}

SegmentationOBB::~SegmentationOBB()
{
	if (p_obb_cameradata_ != nullptr)
	{
		delete p_obb_cameradata_;
	}
}

bool SegmentationOBB::segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg)
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	PointCloud::Ptr cloud_model_temp(new PointCloud());
	pcl::copyPointCloud(*cloud_model, *cloud_model_temp);
	p_obb_cameradata_->DownSample(cloud_model_temp, subsampling_leaf_size);
	feature_extractor.setInputCloud(cloud_model_temp);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	pcl::PointXYZ min_point(min_point_OBB.x - l_offset/2,
		min_point_OBB.y - w_offset/2,
		min_point_OBB.z - h_offset/2);
	pcl::PointXYZ max_point(max_point_OBB.x + l_offset/2,
		max_point_OBB.y + w_offset/2,
		max_point_OBB.z + h_offset/2);

	cloud_seg->clear();
	PointCloud::Ptr cloud_scene_rotation(new PointCloud());
	Eigen::Matrix4f rotation_matrix = Eigen::Matrix4f::Zero();
	for(size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 3; j++)
		{
			rotation_matrix(i, j) = rotational_matrix_OBB(i,j);
		}
	rotation_matrix(0, 3) = position_OBB.x;
	rotation_matrix(1, 3) = position_OBB.y;
	rotation_matrix(2, 3) = position_OBB.z;
	rotation_matrix(3, 3) = 1;
	pcl::transformPointCloud(*cloud_scene, *cloud_scene_rotation, rotation_matrix.inverse());

	//pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	//pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud_scene_rotation);//设置随机颜色
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_scene_rotation, RandomColor, "points");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");

	//Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	//Eigen::Quaternionf quat(rotational_matrix_OBB);
	//viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
	//Eigen::Vector3f position1(0, 0, 0);
	//Eigen::Quaternionf quat1(Eigen::Matrix3f::Identity());
	//viewer->addCube(position1, quat1, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB1");
	//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB1");
	
	for (size_t i = 0; i < cloud_scene_rotation->points.size(); i++)
	{
		if (cloud_scene_rotation->points[i].x > min_point.x
			&& cloud_scene_rotation->points[i].y > min_point.y
			&& cloud_scene_rotation->points[i].z > min_point.z
			&& cloud_scene_rotation->points[i].x < max_point.x
			&& cloud_scene_rotation->points[i].y < max_point.y
			&& cloud_scene_rotation->points[i].z < max_point.z)
		{
			cloud_seg->points.push_back(cloud_scene->points[i]);
		}
	}
	if (true == debug_visualization)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("SegOBB"));
		pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud_seg);//设置随机颜色
		viewer->addPointCloud<pcl::PointXYZ>(cloud_seg, RandomColor, "points");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "points");
	}

	return true;
}

bool SegmentationOBB::SetParameters(const std::string config_file)
{
	JsonOutType json_reader;
	json_reader = p_obb_cameradata_->ReadJsonFile(config_file, "Sample3D_OBB", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	else
		return false;
	json_reader = p_obb_cameradata_->ReadJsonFile(config_file, "DebugVisualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	else
		return false;
	json_reader = p_obb_cameradata_->ReadJsonFile(config_file, "L_Offset", "float");
	if (json_reader.success)
		l_offset = json_reader.json_float;
	else
		return false;
	json_reader = p_obb_cameradata_->ReadJsonFile(config_file, "W_Offset", "float");
	if (json_reader.success)
		w_offset = json_reader.json_float;
	else
		return false;
	json_reader = p_obb_cameradata_->ReadJsonFile(config_file, "H_Offset", "float");
	if (json_reader.success)
		h_offset = json_reader.json_float;
	else
		return false;

	subsampling_leaf_size = Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f);

	return true;
}

void SegmentationOBB::CalulateOBB(const PointCloud::Ptr cloud_in)
{

}

ISegmentation* GetSegmentationOBB()
{
	ISegmentation* p_isegmentation_ = new SegmentationOBB();
	return p_isegmentation_;
}
