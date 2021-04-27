#ifndef PLUGININTERFACEREGISTRATION_H
#define PLUGININTERFACEREGISTRATION_H
#include "pluginInterfaceRegistration_global.h"

#include <QMetaType>
#include <QObject>
#include <QString>

#include "MyPointRepresentation.h"
//配准
#include <pcl/registration/icp.h> 
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>

//#include "opencv2/opencv.hpp"

//Q_DECLARE_METATYPE(cv::Mat)


class PLUGININTERFACEREGISTRATION_EXPORT pluginInterfaceRegistration : public QObject
{
  Q_OBJECT

public:
    pluginInterfaceRegistration() {}
    virtual ~pluginInterfaceRegistration() {}
public slots:
    virtual void SAC_IA(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr transformed_cloud, Eigen::Matrix4f &SAC_transform, float downsample, bool debug_v) = 0;
    virtual void LM_ICP (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample, bool debug_v) = 0;
    virtual void DCP(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, float downsample , bool debug_v) = 0;
    virtual void ComputeTransformation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, float downsample, bool debug_v) = 0;
    virtual Eigen::Matrix4f GetTransformation() = 0;
signals:
  //void isDone(cv::Mat image);
//public:
  //cv::Mat outputImagePlugin;
};

#define PLUGININTERFACREGISTRATION_IID "com.amin.pluginInterfaceRegistration"
Q_DECLARE_INTERFACE(pluginInterfaceRegistration, PLUGININTERFACREGISTRATION_IID)

#endif // PLUGININTERFACECEGEGISTRATION_H
