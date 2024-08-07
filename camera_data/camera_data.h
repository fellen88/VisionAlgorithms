#pragma once
#include "icamera_data.h"
#include "json/json.h"

class CameraData :public ICameraData
{
  public:
	LPVOID pcolorBuffer;                                  
	LPVOID pdepthBuffer;                                  
	LPVOID mask_buffer;
	LPVOID label_buffer;

	HANDLE hcolorMap;
	HANDLE hdepthMap;
	HANDLE mask_map;
	HANDLE label_map;

	bool isOpenCameraMapping;
	bool isOpenMaskMapping;
	bool DebugVisualization;

	enum CameraState
	{
		DISCONNECTED = 0,
		CONNECTED
	};

	enum PictureState
	{
		FIRST,
		WRITING,
		WRITED,
		READING,
		READED
	};

	CameraData();
	~CameraData();

	int ucharToMat(uchar *p2, cv::Mat& src, int flag);

	bool GetCameraImages(cv::Mat& color, cv::Mat& depth);
	bool GetMaskAndLabel(cv::Mat& mask, std::string label);
	bool SetParameters(const std::string JsonFilePath);
	bool LoadPointCloud(std::string file_name, PointCloud::Ptr object_model);
	bool LoadPLY(std::string file_name, PointCloud::Ptr object_model);
	bool Load2DImage(cv::Mat image, std::string file_name);
	void ShowPointCloud(const PointCloud::Ptr pointcloud, std::string window_name);
	void ShowPointCloud_NonBlocking(const PointCloud::Ptr pointcloud, size_t spin_time, std::string cloudname, unsigned char r, unsigned char g, unsigned char b);
	void ShowImage(const cv::Mat image, std::string window_name);
	void ShowPose(const PointCloud::Ptr pointcloud, Eigen::Matrix4f &pose, std::string window_name);
	
	JsonOutType ReadJsonFile(const std::string file_name, const std::string key_name, const char* out_type);
	JsonOutType ReadJsonString(const std::string json_string,const std::string key_name, const char* out_type);
	bool DepthtoPointCloud(cv::Mat Depth, cv::Mat Mask, PointCloud::Ptr pointcloud);
	void ConvertPointsMMtoM(PointCloud::Ptr pointcloud);
	void Matrix2EulerAngle(Eigen::Matrix4f& matrix, Eigen::Vector3f& eulerangle);
	void VectorPointstoPCL(std::vector<double> points_normals, PointCloud::Ptr pointcloud, PointCloudWithNormals::Ptr pointcloud_normals);
	void DownSample(const PointCloud::Ptr cloud, const Eigen::Vector4f subsampling_leaf_size);
	void UniformSampling(const PointCloud::Ptr cloud, const float search_radius, PointCloud::Ptr cloud_keypoints);
	void CalculateNormals(const PointCloud::Ptr cloud, const float search_radius, PointCloudWithNormals::Ptr cloud_normal);
	void CurvaturesEstimation(const PointCloud::Ptr cloud, const float search_radius, PointCloudWithNormals::Ptr cloud_normals, PointCloudWithCurvatures::Ptr cloud_curvatures);
	void RemoveInvalidPoints(const PointCloud::Ptr cloud_in, PointCloud::Ptr cloud_out);
	void EdgeWithNormal(const PointCloud::Ptr cloud_in, const float search_radius, const float curvature_thredhold, PointCloudWithNormals::Ptr cloud_normals, PointCloud::Ptr cloud_out);
	bool GetSubPath(const std::string& strPath, std::string& strSubPath, int nSearch);

	bool GetPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>& object_points, PointCloud::Ptr object_scan);

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_nonblock;
	bool sensor_offline;
	std::string ScanFileName;
};

