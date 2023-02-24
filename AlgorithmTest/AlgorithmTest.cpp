#include <iostream>
#include <vector>
#include <pcl/io/ply_io.h>

#include "../pose_estimation/ipose_estimation.h"
#ifdef _DEBUG
#pragma comment (lib, "../X64/Debug/vision_pose_estimation.lib")
#else
#pragma comment (lib, "../X64/Release/vision_pose_estimation.lib")
#endif

int main()
{
	//位姿估计算法使用变量定义
	pcl::PointCloud<pcl::PointXYZRGBNormal> object_points;
	std::vector<double> object_pose;
	unsigned char view_point = 0;

	//3D视觉算法调用:初始化实例
	std::string project_name = "BYD";
	//std::string project_name = "AccuracyGrasp";
	std::string config_byd_1 = ".\\" + project_name + "\\Config_1\\accuracy_grasp.json";
	std::shared_ptr<val::IPoseEstimation> p_byd_1_(GetInstance(val::IPoseEstimation::AccuracyGrasp, config_byd_1));

	//std::string config_byd_2 = ".\\BYD\\Config_2\\accuracy_grasp.json";
	//std::shared_ptr<val::IPoseEstimation> p_byd_2_(GetInstance(val::IPoseEstimation::AccuracyGrasp, config_byd_2));

	//加载测试点云数据
	pcl::io::loadPLYFile(project_name + "\\PointCloud\\test_1.ply", object_points);
	char input;

	while (true)
	{
		std::cout << "input 's' to start /'r' to reset parameters / 'e' to end :" << std::endl;
		std::cin >> input;
		if (input == 's')
		{
			//采集拍照点1 目标场景点云
			view_point = 1;
			//3D视觉算法调用:计算目标位姿
			p_byd_1_->Compute(object_points, view_point, object_pose);
		}
		else if (input == 'r')
			p_byd_1_.reset(GetInstance(val::IPoseEstimation::AccuracyGrasp, config_byd_1));
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

