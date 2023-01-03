// AlgorithmTest.cpp : This file contains the 'main' function. Program execution begins and ends there.

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
	std::shared_ptr<IPoseEstimation> p_pose_estimation_(GetInstance('B'));

	//加载测试点云数据
	pcl::io::loadPLYFile("PointCloud/cylinder_multiple.ply", object_points);

	while (true)
	{
		char input;
		std::cout << "input 's' to start / input 'e' to end :" << std::endl;
		std::cin >> input;
		if (input == 's')
		{
			//采集拍照点1 目标场景点云
			view_point = 1;
			//3D视觉算法调用:计算目标位姿
			p_pose_estimation_->Algorithm_B(object_points, view_point, object_pose);
		}
		else if (input == 'e')
			break;
		else
			std::cout << "input error !" << std::endl;
	}
}

