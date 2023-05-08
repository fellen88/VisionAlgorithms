// stdafx.h: 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 项目特定的包含文件

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // 从 Windows 头文件中排除极少使用的内容
// Windows 头文件
#include <windows.h>

// Google glog
#include "glog/logging.h"

// OpenCV
#include "opencv2/opencv.hpp"

//PCL 点/点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//pcl文件输入/输出
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//pcl滤波
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

//pcl法向量
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

//pcl曲率
#include<pcl/features/principal_curvatures.h>

//pcl	变换
#include<pcl/common/transforms.h>

//pcl可视化
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h> 
#include <pcl/visualization/pcl_plotter.h>

//pcl类型名简化
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::Normal> PointNormals;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> PointCloudWithCurvatures;