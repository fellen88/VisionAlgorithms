#include "stdafx.h"
#include "segmentation_boundary.h"
#include <pcl/features/boundary.h>
SegmentationBoundary::SegmentationBoundary():
	p_seg_cameradata_(GetCameraData())
{
}

SegmentationBoundary::~SegmentationBoundary()
{
}

bool SegmentationBoundary::Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg)
{
	PointCloud::Ptr cloud_scene_temp(new PointCloud());
	p_seg_cameradata_->UniformSampling(cloud_scene, sample_3d, cloud_scene_temp);

	pcl::PointCloud<pcl::Boundary> boundaries; //保存边界估计结果
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //定义一个进行边界特征估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //定义一个法线估计的对象
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //保存法线估计的结果
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	normEst.setInputCloud(cloud_scene_temp);
	normEst.setRadiusSearch(normal_radius); //设置法线估计的半径//normEst.setKSearch(10);//表示计算点云法向量时，搜索的点云个数
	normEst.compute(*normals); //将法线估计结果保存至normals

	boundEst.setInputCloud(cloud_scene_temp); //设置输入的点云
	boundEst.setInputNormals(normals); //设置边界估计的法线，因为边界估计依赖于法线
	boundEst.setRadiusSearch(boundary_radius); //设置边界估计所需要的半径,//这里的Threadshold为一个浮点值，可取点云模型密度的10倍
	boundEst.setAngleThreshold(M_PI / angle_threshold); //边界估计时的角度阈值M_PI / 4  并计算k邻域点的法线夹角,若大于阈值则为边界特征点
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>)); //设置搜索方式KdTree
	boundEst.compute(boundaries); //将边界估计结果保存在boundaries

	//std::cerr << "AngleThreshold: " << M_PI / 4 << std::endl;
	//输出边界点的个数
	std::cerr << "boundaries: " << boundaries.points.size() << std::endl;
	//存储估计为边界的点云数据，将边界结果保存为pcl::PointXYZ类型
	cloud_seg->points.clear();
	for (int i = 0; i < cloud_scene_temp->points.size(); i++)
	{
		if (boundaries[i].boundary_point > 0)
		{
			cloud_seg->push_back(cloud_scene_temp->points[i]);
		}
	}
	return true;
}

bool SegmentationBoundary::SetParameters(const std::string config_file)
{
	JsonOutType json_reader;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "Sample3D_Boundary", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "NormalRadius", "float");
	if (json_reader.success)
		normal_radius = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "AngleThreshold", "float");
	if (json_reader.success)
		angle_threshold = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "BoundaryRadius", "float");
	if (json_reader.success)
		boundary_radius = json_reader.json_float;
	else
		return false;
	return true;
}
ISegmentation* GetSegmentationBoundary()
{
	ISegmentation* p_isegmentation_ = new SegmentationBoundary();
	return p_isegmentation_;
}
