
#include "registration_lm_icp.h"
#include <math.h>

Registration3D::Registration3D() :
	icp_output(new pcl::PointCloud<pcl::PointXYZ>)
{
	p_regist_cameradata_ = GetCameraData();
}

bool Registration3D::SetParameters(const std::string config_file)
{
	JsonOutType json_reader;
	json_reader = p_regist_cameradata_->ReadJsonFile(config_file, "Sample3D_ICP", "float");
	if (json_reader.success)
		sample_3d = json_reader.json_float;
	else
		return false;
	json_reader = p_regist_cameradata_->ReadJsonFile(config_file, "DebugVisualization", "bool");
	if (json_reader.success)
		debug_visualization = json_reader.json_bool;
	else
		return false;
	json_reader = p_regist_cameradata_->ReadJsonFile(config_file, "MaxCorrespondenceDistance", "float");
	if (json_reader.success)
		max_correspondence_distance = json_reader.json_float;
	else
		return false;

	return true;
}

void Registration3D::Align(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform)
{
	final_transform = Eigen::Matrix4f::Identity();

	if (cloud_src->points.size() < 10 || cloud_tgt->points.size() < 10)
	{
		LOG(ERROR) << "points.size() < 10, return !";
		return;
	}
	//为了一致性和速度，下采样
	PointCloud::Ptr src(new PointCloud); //创建点云指针
	PointCloud::Ptr tgt(new PointCloud);
	pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
	if (sample_3d > 0.00001) //下采样
	{
		grid.setLeafSize(sample_3d, sample_3d, sample_3d); //设置体元网格的叶子大小
				//下采样 源点云
		grid.setInputCloud(cloud_src); //设置输入点云
		grid.filter(*src); //下采样和滤波，并存储在src中
				//下采样 目标点云
		grid.setInputCloud(cloud_tgt);
		grid.filter(*tgt);
	}
	else //不下采样
	{
		src = cloud_src; //直接复制
		tgt = cloud_tgt;
	}

	//计算曲面的法向量和曲率
	PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals); //创建源点云指针（注意点的类型包含坐标和法向量）
	PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals); //创建目标点云指针（注意点的类型包含坐标和法向量）
	pcl::NormalEstimation<PointT, PointNormalT> norm_est; //该对象用于计算法向量
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); //创建kd树，用于计算法向量的搜索方法
	norm_est.setSearchMethod(tree); //设置搜索方法
	norm_est.setKSearch(30); //设置最近邻的数量
	norm_est.setInputCloud(src); //设置输入云
	norm_est.compute(*points_with_normals_src); //计算法向量，并存储在points_with_normals_src
	pcl::copyPointCloud(*src, *points_with_normals_src); //复制点云（坐标）到points_with_normals_src（包含坐标和法向量）
	norm_est.setInputCloud(tgt); //这3行计算目标点云的法向量，同上
	norm_est.compute(*points_with_normals_tgt);
	pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

	//创建一个 自定义点表达方式的 实例
	MyPointRepresentation point_representation;
	//加权曲率维度，以和坐标xyz保持平衡
	float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha); //设置缩放值（向量化点时使用）

	//创建非线性ICP对象 并设置参数
	pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; //创建非线性ICP对象（ICP变体，使用Levenberg-Marquardt最优化）
	reg.setTransformationEpsilon(0.000001); //设置容许的最大误差（迭代最优化）
	//***** 注意：根据自己数据库的大小调节该参数
	reg.setMaxCorrespondenceDistance(max_correspondence_distance);  //设置对应点之间的最大距离,在配准过程中，忽略大于该阈值的点  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation)); //设置点表达
	//设置源点云和目标点云
	reg.setInputSource(points_with_normals_src); //版本不符合，使用下面的语句
	//reg.setInputCloud (points_with_normals_src); //设置输入点云（待变换的点云）
	reg.setInputTarget(points_with_normals_tgt); //设置目标点云
	reg.setMaximumIterations(5); //设置内部优化的迭代次数

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
	PointCloudWithNormals::Ptr reg_result = points_with_normals_src; //用于存储结果（坐标+法向量）

	int NumIteration = 0;
	for (int i = 0; i < 200; ++i) //迭代
	{
		//PCL_INFO ("Iteration Nr. %d.\n", i); //命令行显示迭代的次数
		points_with_normals_src = reg_result; //
		//估计
		reg.setInputCloud (points_with_normals_src); //重新设置输入点云（待变换的点云），因为经过上一次迭代，已经发生变换了
		reg.align(*reg_result); //对齐（配准）两个点云

		Ti = reg.getFinalTransformation() * Ti; //累积（每次迭代的）变换矩阵
		//如果这次变换和上次变换的误差比阈值小，通过减小最大的对应点距离的方法来进一步细化
		// if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
		//    break;
		//如果这次变换和上次变换的误差比阈值小，通过减小最大的对应点距离的方法来进一步细化
		if (fabs((double)(reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
			reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.0000001); //减小对应点之间的最大距离（上面设置过）
		if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < 0.0000001)
			break;
	// std::cout<<"getLastIncrementalTransformation"<<reg.getLastIncrementalTransformation ()<<endl;
	 //std::cout<<"prev"<<prev<<endl;
	 //std::cout<<"getLastIncrementalTransformation.sum: "<<fabs((reg.getLastIncrementalTransformation() - prev).sum())<<endl;
		prev = reg.getLastIncrementalTransformation(); //上一次变换的误差
		NumIteration = i;
		//显示当前配准状态，在窗口的右视区，简单的显示源点云和目标点云
		/*if(true == DEBUG_VISUALIZER)
		{
			pointcloud_visualizer_.showCloudsRight(points_with_normals_tgt, points_with_normals_src);
		}*/
	}

	PCL_INFO ("Iteration Nr. %d.\n", NumIteration); //命令行显示迭代的次数
	targetToSource = Ti.inverse(); //计算从目标点云到源点云的变换矩阵
	pcl::transformPointCloud(*tgt, *output, targetToSource); //将目标点云 变换回到 源点云帧

	//add the source to the transformed target
	//*output += *cloud_src; // 拼接点云图（的点）点数数目是两个点云的点数和
	final_transform = targetToSource; //最终的变换。目标点云到源点云的变换矩阵

	if (true == debug_visualization)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Registration3D"));
		viewer->removePointCloud("source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
		viewer->removePointCloud("target");
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0); //设置点云显示颜色，下同
		pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_h(src, 255, 0, 0);
		viewer->addPointCloud(output, cloud_tgt_h, "target"); //添加点云数据，下同
		viewer->addPointCloud(src, cloud_src_h, "source");
	  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
	  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");
		//PCL_INFO ("Press q to clear the screen.\n");
		viewer->spin();
	}
}

IRegistration3D* GetRegistrationLMICP()
{
	IRegistration3D* p_iregistration = new Registration3D();
	return p_iregistration;
}