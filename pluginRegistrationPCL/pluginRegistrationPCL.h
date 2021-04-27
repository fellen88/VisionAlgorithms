#ifndef PLUGINRREGISTRATIONPCL_H
#define PLUGINRREGISTRATIONPCL_H

#include <QtCore/qglobal.h>

#include "pluginInterfaceRegistration/pluginInterfaceRegistration.h"

class pluginRegistrationPCL: public pluginInterfaceRegistration
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.amin.pluginInterfaceRegistration")
    Q_INTERFACES(pluginInterfaceRegistration)
public:
    pluginRegistrationPCL();
    ~pluginRegistrationPCL();
    
    bool DEBUG_VISUALIZER;

	Eigen::Matrix4f final_transform;
	Eigen::Matrix4f sac_transform;
	Eigen::Matrix4f icp_transform;
	PointCloud::Ptr sac_output;
	PointCloud::Ptr icp_output;

    bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud);
    fpfhFeature::Ptr ComputeFpfh(const PointCloud::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree);
	void SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, float downsample, bool debug_v);
	void LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample, bool debug_v);
    void ComputeTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, float downsample, bool debug_v);
    Eigen::Matrix4f GetTransformation();
};

#endif // PLUGINRREGISTRATIONPCL_H
