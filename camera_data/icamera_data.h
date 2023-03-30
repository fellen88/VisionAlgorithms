#ifndef CAMERADATA_H
#define CAMERADATA_H

#include "stdafx.h"

#ifdef CAMERADATA_EXPORTS
#define CAMERADATA_API __declspec(dllexport)
#else
#define CAMERADATA_API __declspec(dllimport)
#endif

struct JsonOutType
{
	bool success;
	int json_int;
	float json_float;
	double json_double;
	bool json_bool;
	std::string json_string;
};

class ICameraData
{
	public:

	int image_height_;
	int image_width_;
	float fx_;
	float fy_;
	float cx_;
	float cy_;
	int scale_factor_;

	virtual ~ICameraData() = 0;
	virtual bool GetCameraImages(cv::Mat& color, cv::Mat &depth) = 0;
	virtual bool GetMaskAndLabel(cv::Mat& mask, std::string label) = 0;
	virtual bool SetParameters(std::string JsonFilePath) = 0;
	virtual bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud) = 0;
	virtual bool LoadPointCloud(std::string file_name, PointCloud::Ptr object_model) = 0;
	virtual bool LoadPLY(std::string file_name, PointCloud::Ptr object_model) = 0;
	virtual bool Load2DImage(cv::Mat image, std::string file_name) = 0;
	virtual void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name) = 0;
	virtual void ShowPointCloud_NonBlocking(const PointCloud::Ptr pointcloud, size_t spin_time,
		std::string cloudname,unsigned char r, unsigned char g, unsigned char b) = 0;
	virtual void ShowImage(const cv::Mat image, std::string window_name) = 0;
 	virtual JsonOutType ReadJsonFile(std::string file_name, std::string key_name, const char* out_type) = 0;
	virtual JsonOutType ReadJsonString(std::string json_string, std::string key_name, const char* out_type) = 0;
	virtual void ConvertPointsMMtoM(PointCloud::Ptr pointcloud) = 0;
	virtual void Matrix2EulerAngle(Eigen::Matrix4f& matrix, Eigen::Vector3f& eulerangle) = 0;
	virtual void VectorPointstoPCL(std::vector<double> points_normals, PointCloud::Ptr pointcloud, PointCloudWithNormals::Ptr pointcloud_normals) = 0;
	virtual void DownSample(const PointCloud::Ptr cloud, const Eigen::Vector4f subsampling_leaf_size) = 0;
	virtual void CalculateNormals(const PointCloud::Ptr cloud, const float search_radius, PointCloudWithNormals::Ptr cloud_normal) = 0;
	virtual void CurvaturesEstimation(const PointCloud::Ptr cloud, const float search_radius, PointCloudWithNormals::Ptr cloud_normals, PointCloudWithCurvatures::Ptr cloud_curvatures) = 0;
	virtual void EdgeWithNormal(const PointCloud::Ptr cloud_in, const float search_radius, const float curvature_thredhold, PointCloudWithNormals::Ptr cloud_normals, PointCloud::Ptr cloud_out) = 0;
	virtual void RemoveInvalidPoints(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out) = 0;
	virtual bool GetSubPath(const std::string& strPath, std::string& strSubPath, int nSearch) = 0;
};

CAMERADATA_API ICameraData* APIENTRY GetCameraData();

#endif
