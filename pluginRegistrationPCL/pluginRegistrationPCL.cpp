#include "pluginRegistrationPCL.h"
#include <math.h>

pluginRegistrationPCL::pluginRegistrationPCL():
    sac_output(new pcl::PointCloud<pcl::PointXYZ>),
	  icp_output(new pcl::PointCloud<pcl::PointXYZ>)
{
}

pluginRegistrationPCL::~pluginRegistrationPCL()
{

}

bool pluginRegistrationPCL::DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr CloudMask)
{
	for(int rows = 0; rows < image_height_; rows++)
    {
			for(int cols = 0; cols < image_width_; cols++ )
			{
					float d = 0;
					if (!Depth.empty())
					{
						//获取深度图中对应点的深度值
						d = Depth.at<ushort>(rows, cols);
					}
					else
					{
						//LOG(ERROR) << "depth image empty!";
						return false;
					}

					//有效范围内的点
					//if ((d > 0.5*scale_factor_) && (d < 1.5*scale_factor_))
					{
						//判断mask中是否是物体的点
						if (!Mask.empty())
						{
							unsigned char mask_value = Mask.at<unsigned char>(rows, cols);
							if (mask_value == 0)
								continue;
						}
						else
						{
							//LOG(ERROR) << ("mask image empty!");
								return false;
						}
						//计算这个点的空间坐标
						pcl::PointXYZ PointWorld;
						PointWorld.z = double(d) / scale_factor_;
						PointWorld.x = (rows - cx_)*PointWorld.z / fx_;
						PointWorld.y = (cols - cy_)*PointWorld.z / fy_;
						CloudMask->points.push_back(PointWorld);
					}
			}
  }
  //设置点云属性，采用无序排列方式存储点云
  CloudMask->height = 1;
  CloudMask->width = CloudMask->points.size();
  //LOG(INFO) << "mask cloud size = " << CloudMask->width;
  if(0 == CloudMask->points.size())
  {
    //LOG(ERROR)<<("Mask points number = 0 !!!");
    return false;
  }
  //去除NaN点
  std::vector<int> nan_indices;
  pcl::removeNaNFromPointCloud(*CloudMask, *CloudMask, nan_indices);
  CloudMask->is_dense = false;
  return true;
}

fpfhFeature::Ptr pluginRegistrationPCL::ComputeFpfh(const PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
    pointnormal::Ptr point_normal(new pointnormal);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
    est_normal.setInputCloud(input_cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(20);
    est_normal.compute(*point_normal);
    fpfhFeature::Ptr fpfh(new fpfhFeature);
    //pcl::FPFHEstimation<pcl::PointXYZ,pcl::Normal,pcl::FPFHSignature33> est_fpfh;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(4);
    est_fpfh.setInputCloud(input_cloud);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(20);
    est_fpfh.compute(*fpfh);

    return fpfh;
}

void pluginRegistrationPCL::SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &SAC_transform, float downsample, bool debug_v)
{
    //为了一致性和速度，下采样
    PointCloud::Ptr source_filtered(new PointCloud); //创建点云指针
    PointCloud::Ptr target_filtered(new PointCloud);
    pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
    if (downsample) //下采样
    {
        grid.setLeafSize (downsample, downsample, downsample); //设置体元网格的叶子大小
            //下采样 源点云
        grid.setInputCloud (cloud_src); //设置输入点云
        grid.filter (*source_filtered); //下采样和滤波，并存储在src中
            //下采样 目标点云
        grid.setInputCloud (cloud_tgt);
        grid.filter (*target_filtered);
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

	//对齐(占用了大部分运行时间)
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(source_filtered);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(target_filtered);
	sac_ia.setTargetFeatures(target_fpfh);
	//sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(10); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*output);

	SAC_transform = sac_ia.getFinalTransformation();

	//可视化
	if (true == debug_v)
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

void pluginRegistrationPCL::LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample, bool debug_v)
{
    //为了一致性和速度，下采样
    PointCloud::Ptr src (new PointCloud); //创建点云指针
    PointCloud::Ptr tgt (new PointCloud);
    pcl::VoxelGrid<PointT> grid; //VoxelGrid 把一个给定的点云，聚集在一个局部的3D网格上,并下采样和滤波点云数据
    if (downsample) //下采样
    {
        grid.setLeafSize (downsample, downsample, downsample); //设置体元网格的叶子大小
            //下采样 源点云
        grid.setInputCloud (cloud_src); //设置输入点云
        grid.filter (*src); //下采样和滤波，并存储在src中
            //下采样 目标点云
        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
    }
    else //不下采样
    {
        src = cloud_src; //直接复制
        tgt = cloud_tgt;
    }

    //计算曲面的法向量和曲率
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals); //创建源点云指针（注意点的类型包含坐标和法向量）
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals); //创建目标点云指针（注意点的类型包含坐标和法向量）
    pcl::NormalEstimation<PointT, PointNormalT> norm_est; //该对象用于计算法向量
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ()); //创建kd树，用于计算法向量的搜索方法
    norm_est.setSearchMethod (tree); //设置搜索方法
    norm_est.setKSearch (30); //设置最近邻的数量
    norm_est.setInputCloud (src); //设置输入云
    norm_est.compute (*points_with_normals_src); //计算法向量，并存储在points_with_normals_src
    pcl::copyPointCloud (*src, *points_with_normals_src); //复制点云（坐标）到points_with_normals_src（包含坐标和法向量）
    norm_est.setInputCloud (tgt); //这3行计算目标点云的法向量，同上
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //创建一个 自定义点表达方式的 实例
    MyPointRepresentation point_representation;
    //加权曲率维度，以和坐标xyz保持平衡
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues (alpha); //设置缩放值（向量化点时使用）

    //创建非线性ICP对象 并设置参数
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; //创建非线性ICP对象（ICP变体，使用Levenberg-Marquardt最优化）
    reg.setTransformationEpsilon (1e-6); //设置容许的最大误差（迭代最优化）
    //reg.setTransformationEpsilon (0.01); //设置容许的最大误差（迭代最优化）
    //***** 注意：根据自己数据库的大小调节该参数
    reg.setMaxCorrespondenceDistance (2);  //设置对应点之间的最大距离（2m）,在配准过程中，忽略大于该阈值的点  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation)); //设置点表达
    //设置源点云和目标点云
    reg.setInputSource (points_with_normals_src); //版本不符合，使用下面的语句
    //reg.setInputCloud (points_with_normals_src); //设置输入点云（待变换的点云）
    reg.setInputTarget (points_with_normals_tgt); //设置目标点云
    reg.setMaximumIterations (2); //设置内部优化的迭代次数

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src; //用于存储结果（坐标+法向量）

    int NumIteration = 0;
    for (int i = 0; i < 200; ++i) //迭代
    {
        //pcl::ScopeTime scope_time("ICP Iteration");
        //PCL_INFO ("Iteration Nr. %d.\n", i); //命令行显示迭代的次数
        //保存点云，用于可视化
        points_with_normals_src = reg_result; //
        //估计
        reg.setInputSource (points_with_normals_src);
        //reg.setInputCloud (points_with_normals_src); //重新设置输入点云（待变换的点云），因为经过上一次迭代，已经发生变换了
        reg.align (*reg_result); //对齐（配准）两个点云

        Ti = reg.getFinalTransformation () * Ti; //累积（每次迭代的）变换矩阵
        //如果这次变换和上次变换的误差比阈值小，通过减小最大的对应点距离的方法来进一步细化
        // if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
        //    break;

              //如果这次变换和上次变换的误差比阈值小，通过减小最大的对应点距离的方法来进一步细化
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001); //减小对应点之间的最大距离（上面设置过）
        prev = reg.getLastIncrementalTransformation (); //上一次变换的误差
        //std::cout<<"getLastIncrementalTransformation"<<reg.getLastIncrementalTransformation ()<<endl;
        //std::cout<<"getLastIncrementalTransformation.sum: "<<reg.getLastIncrementalTransformation ().sum()<<endl;
        NumIteration = i;
        //显示当前配准状态，在窗口的右视区，简单的显示源点云和目标点云
        /*if(true == DEBUG_VISUALIZER)
        {
          pointcloud_visualizer_.showCloudsRight(points_with_normals_tgt, points_with_normals_src);
        }*/
    }

    //PCL_INFO ("Iteration Nr. %d.\n", NumIteration); //命令行显示迭代的次数
    targetToSource = Ti.inverse(); //计算从目标点云到源点云的变换矩阵
    pcl::transformPointCloud (*cloud_tgt, *output, targetToSource); //将目标点云 变换回到 源点云帧

    //add the source to the transformed target
    //*output += *cloud_src; // 拼接点云图（的点）点数数目是两个点云的点数和
    final_transform = targetToSource; //最终的变换。目标点云到源点云的变换矩阵

    if(true == debug_v)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pluginRegistrationPCL"));
        viewer->removePointCloud ("source"); //根据给定的ID，从屏幕中去除一个点云。参数是ID
        viewer->removePointCloud ("target");
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0); //设置点云显示颜色，下同
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
        viewer->addPointCloud (output, cloud_tgt_h, "target"); //添加点云数据，下同
        viewer->addPointCloud (cloud_src, cloud_src_h, "source");
        //PCL_INFO ("Press q to clear the screen.\n");
        viewer->spin ();
    }
}

void pluginRegistrationPCL::ComputeTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, float downsample, bool debug_v)
{
	{
        pcl::ScopeTime scope_time("*SAC_IA");//计算算法运行时间
		SAC_IA(cloud_src, cloud_tgt, sac_output, sac_transform, downsample, debug_v);
	}
	{
        pcl::ScopeTime scope_time("*LM_ICP");//计算算法运行时间
		LM_ICP(cloud_tgt, sac_output, icp_output, icp_transform, downsample, debug_v);
	}

	final_transform = icp_transform * sac_transform;
}

Eigen::Matrix4f pluginRegistrationPCL::GetTransformation()
{
	return final_transform;
}