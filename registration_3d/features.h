
#ifndef MYPOINTREPRESENTATION_H
#define MYPOINTREPRESENTATION_H

#include "stdafx.h"

class Features
{
public:
  fpfhFeature::Ptr ComputeFpfh(const PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
};

# endif