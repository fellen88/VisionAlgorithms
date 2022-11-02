// stdafx.h: 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 项目特定的包含文件
//

#pragma once

#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // 从 Windows 头文件中排除极少使用的内容
// Windows 头文件
#include <windows.h>

// Google glog
#include "glog/logging.h"
#pragma comment(lib,"glog.lib")
//PCL
//点/点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//pcl文件输入/输出
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//pcl segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::PointCloud<pcl::Normal> pointnormal;

