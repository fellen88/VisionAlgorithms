
#ifndef MYPOINTREPRESENTATION_H
#define MYPOINTREPRESENTATION_H

#include "stdafx.h"

// 定义新的点表达方�?< x, y, z, curvature > 坐标+曲率
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT> //继承关系
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
  public:
  MyPointRepresentation ()
  {
    //指定维数
    nr_dimensions_ = 4;
  }
  //重载函数copyToFloatArray，以定义�?己的特征向量
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    //< x, y, z, curvature > 坐标xyz和曲�?
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

class Features
{
public:
  fpfhFeature::Ptr ComputeFpfh(const PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
};

# endif