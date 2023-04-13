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
	pcl::PointCloud<pcl::Boundary> boundaries; //����߽���ƽ��
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; //����һ�����б߽��������ƵĶ���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst; //����һ�����߹��ƵĶ���
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); //���淨�߹��ƵĽ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
	normEst.setInputCloud(cloud_scene);
	normEst.setRadiusSearch(normal_radius); //���÷��߹��Ƶİ뾶//normEst.setKSearch(10);//��ʾ������Ʒ�����ʱ�������ĵ��Ƹ���
	normEst.compute(*normals); //�����߹��ƽ��������normals

	boundEst.setInputCloud(cloud_scene); //��������ĵ���
	boundEst.setInputNormals(normals); //���ñ߽���Ƶķ��ߣ���Ϊ�߽���������ڷ���
	boundEst.setRadiusSearch(boundary_radius); //���ñ߽��������Ҫ�İ뾶,//�����ThreadsholdΪһ������ֵ����ȡ����ģ���ܶȵ�10��
	boundEst.setAngleThreshold(M_PI / angle_threshold); //�߽����ʱ�ĽǶ���ֵM_PI / 4  ������k�����ķ��߼н�,��������ֵ��Ϊ�߽�������
	boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>)); //����������ʽKdTree
	boundEst.compute(boundaries); //���߽���ƽ��������boundaries

	//std::cerr << "AngleThreshold: " << M_PI / 4 << std::endl;
	//����߽��ĸ���
	std::cerr << "boundaries: " << boundaries.points.size() << std::endl;
	//�洢����Ϊ�߽�ĵ������ݣ����߽�������Ϊpcl::PointXYZ����
	cloud_seg->points.clear();
	for (int i = 0; i < cloud_scene->points.size(); i++)
	{
		if (boundaries[i].boundary_point > 0)
		{
			cloud_seg->push_back(cloud_scene->points[i]);
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
	subsampling_leaf_size = Eigen::Vector4f(sample_3d, sample_3d, sample_3d, 0.0f);
	return true;
}
ISegmentation* GetSegmentationBoundary()
{
	ISegmentation* p_isegmentation_ = new SegmentationBoundary();
	return p_isegmentation_;
}
