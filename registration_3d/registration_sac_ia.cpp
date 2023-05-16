
#include "stdafx.h"
#include "registration_sac_ia.h"
#include "features.h"
#include <math.h>

RegistrationSACIA::RegistrationSACIA() :
	sac_output(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_regist_cameradata_ = GetCameraData();
}

bool RegistrationSACIA::SetParameters(const std::string config_file)
{
	JsonOutType json_reader;
	json_reader = p_regist_cameradata_->ReadJsonFile(config_file, "Visualization", "bool");
	if (json_reader.success)
		visualization = json_reader.json_bool;
	else
		return false;
	json_reader = p_regist_cameradata_->ReadJsonFile(config_file, "UniformSampling", "float");
	if (json_reader.success)
		uniform_sampling = json_reader.json_float;
	else
		return false;
	return true;
}

void RegistrationSACIA::Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &sac_transform)
{
	sac_transform = Eigen::Matrix4f::Identity();
	if (cloud_src->points.size() < 100 || cloud_tgt->points.size() < 100)
	{
		return;
	}
	//为了一致性和速度，下采样
	PointCloud::Ptr source_filtered(new PointCloud); //创建点云指针
	PointCloud::Ptr target_filtered(new PointCloud);
	pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
	if (uniform_sampling > 0.00001) //下采样
	{
		p_regist_cameradata_->UniformSampling(cloud_src, uniform_sampling, source_filtered);
		p_regist_cameradata_->UniformSampling(cloud_tgt, uniform_sampling, target_filtered);
	}
	else //不下采样
	{
		source_filtered = cloud_src; //直接复制
		target_filtered = cloud_tgt;
	}
	////保存到PCD文件
	//pcl::io::savePCDFileASCII("model.pcd", *source_filtered);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfhFeature::Ptr source_fpfh;
	fpfhFeature::Ptr target_fpfh;
	{
		pcl::ScopeTime scope_time("*PrePairAlign");//计算算法运行时间
		source_fpfh = ComputeFpfh(source_filtered, tree);
		target_fpfh = ComputeFpfh(target_filtered, tree);
	}

	//TODO: sac_ia parameters
	//对齐(占用了大部分运行时间)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(10); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*output);

	sac_transform = sac_ia.getFinalTransformation();
	pcl::transformPointCloud(*cloud_src, *output, sac_transform);
	//可视化
	if (true == visualization)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("fpfh test"));
		int v1;
		int v2;
		view->createViewPort(0, 0.0, 0.5, 1.0, v1);
		view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		view->setBackgroundColor(255, 255, 255, v1);
		view->setBackgroundColor(255, 255, 255, v2);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(source_filtered, 250, 0, 0);
		view->addPointCloud(source_filtered, sources_cloud_color, "sources_cloud_v1", v1);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(target_filtered, 0, 250, 0);
		view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v1", v1);
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sources_cloud_v1");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>aligend_cloud_color(output, 255, 0, 0);
		view->addPointCloud(output, aligend_cloud_color, "aligend_cloud_v2", v2);
		view->addPointCloud(target_filtered, target_cloud_color, "target_cloud_v2", v2);
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_v2");
		view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v2");

		pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
		boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
		crude_cor_est.setInputSource(source_fpfh);
		crude_cor_est.setInputTarget(target_fpfh);
		//crude_cor_est.determineCorrespondences(cru_correspondences);
		crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
		//cout << "crude size is:" << cru_correspondences->size() << endl;
		view->addCorrespondences<pcl::PointXYZ>(source_filtered, target_filtered, *cru_correspondences, "correspondence", v1);//添加显示对应点对

		view->spin();
	}
}

IRegistration3D* GetRegistrationSACIA()
{
	IRegistration3D* p_iregistration = new RegistrationSACIA();
	return p_iregistration;
}