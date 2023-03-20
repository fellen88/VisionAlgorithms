#include "stdafx.h"
#include "segmentation_euclidean.h"
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

SegmentationEuclidean::SegmentationEuclidean():
	p_seg_cameradata_(GetCameraData())
{

}

SegmentationEuclidean::~SegmentationEuclidean()
{
}

unsigned char SegmentationEuclidean::colors[20 * 3] = {
255, 0,   0,   // red 		1
0,   255, 0,   // green		2
0,   0,   255, // blue		3
255, 255, 0,   // yellow	4
0,   255, 255, // light blue5
255, 0,   255, // magenta   6
255, 255, 255, // white		7
255, 128, 0,   // orange	8
255, 153, 255, // pink		9
51,  153, 255, //			10
153, 102, 51,  //			11
128, 51,  153, //			12
153, 153, 51,  //			13
163, 38,  51,  //			14
204, 153, 102, //			15
204, 224, 255, //			16
128, 179, 255, //			17
206, 255, 0,   //			18
255, 204, 204, //			19
204, 255, 153, //			20
};

bool SegmentationEuclidean::Segment(PointCloud::Ptr cloud_scene, PointCloud::Ptr cloud_model, PointCloud::Ptr cloud_seg)
{
	// 创建kd树
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud_scene);

	// 设置分割参数
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(cluster_tolerance);	//设置近邻搜索的半径
	ec.setMinClusterSize(min_cluster_size);		//设置最小聚类点数
	ec.setMaxClusterSize(max_cluster_size);	//设置最大聚类点数
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_scene);
	ec.extract(cluster_indices);

	int v1(0);
	int v2(0);
	if (true == visualization_eucli)
	{
		//----- 可视化1/3-----↓
		viewer.reset(new pcl::visualization::PCLVisualizer("3D viewer"));
		viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1); //设置第一个视口在X轴、Y轴的最小值、最大值，取值在0-1之间
		viewer->setBackgroundColor(0.1, 0.1, 0.1, v1);
		viewer->addText("cloud_in", 10, 10, "v1 text", v1);
		viewer->addPointCloud<PointT>(cloud_scene, "cloud_in", v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_in", v1);

		viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
		viewer->addText("cloud_cluster", 10, 10, "v2 text", v2);
		//-----可视化1/3-----↑
	}
	// 执行欧式聚类分割，并保存分割结果
	int j = 0;
	cloud_seg->points.clear();
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.push_back(cloud_scene->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::stringstream ss;
		ss << "tree_" << j + 1 << ".pcd";
	/*	writer.write<PointT>(ss.str(), *cloud_cluster, true);
		cout << "-----" << ss.str() << "详情-----" << endl;
		cout << *cloud_cluster << endl;*/
		if (j == 0)
			pcl::copyPointCloud(*cloud_cluster, *cloud_seg);

		if (true == visualization_eucli)
		{
			//-----可视化2/3-----↓
			viewer->addPointCloud<PointT>(cloud_cluster, ss.str(), v2);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str(), v2);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[j * 3] / 255, colors[j * 3 + 1] / 255, colors[j * 3 + 2] / 255, ss.str(), v2);
			//-----可视化2/3-----↑
		}

		j++;
	}

	if (true == visualization_eucli)
	{
		//-----可视化3/3-----↓
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		//-----可视化3/3-----↑
	}
	return true;                                               
}

bool SegmentationEuclidean::SetParameters(const std::string config_file)
{
	JsonOutType json_reader;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "Sample3D_Euclidean", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "ClusterTolerance", "float");
	if (json_reader.success)
		cluster_tolerance = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "MinClusterSize", "float");
	if (json_reader.success)
		min_cluster_size = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "MaxClusterSize", "float");
	if (json_reader.success)
		max_cluster_size = json_reader.json_float;
	else
		return false;
	json_reader = p_seg_cameradata_->ReadJsonFile(config_file, "Visualization_Eucli", "bool");
	if (json_reader.success)
		visualization_eucli = json_reader.json_bool;
	else
		return false;

	return true;
}

ISegmentation* GetSegmentationEuclidean()
{
	ISegmentation* p_isegmentation_ = new SegmentationEuclidean();
	return p_isegmentation_;
}
