
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
	//Ϊ��һ���Ժ��ٶȣ��²���
	PointCloud::Ptr source_filtered(new PointCloud); //��������ָ��
	PointCloud::Ptr target_filtered(new PointCloud);
	pcl::VoxelGrid<PointT> grid; //VoxelGrid ��һ�������ĵ��ƣ��ۼ���һ���ֲ���3D������,���²������˲���������
	if (uniform_sampling > 0.00001) //�²���
	{
		p_regist_cameradata_->UniformSampling(cloud_src, uniform_sampling, source_filtered);
		p_regist_cameradata_->UniformSampling(cloud_tgt, uniform_sampling, target_filtered);
	}
	else //���²���
	{
		source_filtered = cloud_src; //ֱ�Ӹ���
		target_filtered = cloud_tgt;
	}
	////���浽PCD�ļ�
	//pcl::io::savePCDFileASCII("model.pcd", *source_filtered);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	fpfhFeature::Ptr source_fpfh;
	fpfhFeature::Ptr target_fpfh;
	{
		pcl::ScopeTime scope_time("*PrePairAlign");//�����㷨����ʱ��
		source_fpfh = ComputeFpfh(source_filtered, tree);
		target_fpfh = ComputeFpfh(target_filtered, tree);
	}

	//TODO: sac_ia parameters
	//����(ռ���˴󲿷�����ʱ��)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	sac_ia.setNumberOfSamples(20);  //����ÿ�ε���������ʹ�õ�������������ʡ��,�ɽ�ʡʱ��
	sac_ia.setCorrespondenceRandomness(10); //���ü���Э����ʱѡ����ٽ��ڵ㣬��ֵԽ��Э����Խ��ȷ�����Ǽ���Ч��Խ��.(��ʡ)
	sac_ia.align(*output);

	sac_transform = sac_ia.getFinalTransformation();
	pcl::transformPointCloud(*cloud_src, *output, sac_transform);
	//���ӻ�
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
		view->addCorrespondences<pcl::PointXYZ>(source_filtered, target_filtered, *cru_correspondences, "correspondence", v1);//�����ʾ��Ӧ���

		view->spin();
	}
}

IRegistration3D* GetRegistrationSACIA()
{
	IRegistration3D* p_iregistration = new RegistrationSACIA();
	return p_iregistration;
}